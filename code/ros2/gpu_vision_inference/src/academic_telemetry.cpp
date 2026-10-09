#include "gpu_vision_inference/academic_telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"

namespace gpu_vision_inference
{

namespace
{

constexpr double kBytesPerKibibyte = 1024.0;

void write_csv_number(std::ostream & stream, const double value)
{
  if (std::isfinite(value)) {
    stream << value;
  }
}

std::string csv_escape(const std::string & value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped{"\""};
  for (const char character : value) {
    if (character == '\"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += '\"';
  return escaped;
}

}  // namespace

AcademicTelemetryLogger::AcademicTelemetryLogger(
  Configuration configuration, rclcpp::Logger logger)
: configuration_(std::move(configuration)), logger_(std::move(logger))
{
  if (configuration_.statistics_capacity == 0U) {
    throw std::invalid_argument("telemetry statistics_capacity must be greater than zero");
  }
  if (!configuration_.enabled) {
    accepting_ = false;
    return;
  }
  const size_t initial_statistics_reserve = std::min<size_t>(
    configuration_.statistics_capacity, 8192U);
  transport_samples_.reserve(initial_statistics_reserve);
  decode_samples_.reserve(initial_statistics_reserve);
  inference_samples_.reserve(initial_statistics_reserve);
  end_to_end_samples_.reserve(initial_statistics_reserve);
  if (configuration_.output_path.empty()) {
    throw std::invalid_argument("telemetry output_path must not be empty when logging is enabled");
  }
  if (configuration_.format != "csv" && configuration_.format != "jsonl") {
    throw std::invalid_argument("telemetry format must be 'csv' or 'jsonl'");
  }
  if (configuration_.run_id.empty()) {
    throw std::invalid_argument("telemetry run_id must not be empty");
  }
  if (configuration_.queue_capacity == 0U || configuration_.flush_interval_records == 0U) {
    throw std::invalid_argument("telemetry queue and flush sizes must be greater than zero");
  }

  const std::filesystem::path output_path(configuration_.output_path);
  if (output_path.has_parent_path()) {
    std::filesystem::create_directories(output_path.parent_path());
  }
  const bool existing_nonempty_file =
    configuration_.append && std::filesystem::exists(output_path) &&
    std::filesystem::file_size(output_path) > 0U;
  const auto mode = configuration_.append ?
    (std::ios::out | std::ios::app) : (std::ios::out | std::ios::trunc);
  output_.open(output_path, mode);
  if (!output_.is_open()) {
    throw std::runtime_error("failed to open telemetry output: " + configuration_.output_path);
  }
  output_.imbue(std::locale::classic());
  output_ << std::fixed << std::setprecision(6);
  if (configuration_.format == "csv" && !existing_nonempty_file) {
    write_csv_header();
  }
  writer_thread_ = std::thread(&AcademicTelemetryLogger::writer_loop, this);
}

AcademicTelemetryLogger::~AcademicTelemetryLogger()
{
  stop();
}

void AcademicTelemetryLogger::enqueue(FrameMetrics metrics) noexcept
{
  if (!configuration_.enabled) {
    return;
  }
  try {
    metrics.run_id = configuration_.run_id;

    {
      std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
      ++records_enqueued_;
      if (metrics.status == "processed") {
        if (statistics_frame_count_ >= configuration_.statistics_capacity) {
          statistics_samples_lost_.fetch_add(1);
        } else {
          ++statistics_frame_count_;
          if (std::isfinite(metrics.transport_latency_ms)) {
            transport_samples_.push_back(metrics.transport_latency_ms);
          }
          if (std::isfinite(metrics.decode_ms)) {
            decode_samples_.push_back(metrics.decode_ms);
          }
          if (std::isfinite(metrics.inference_ms)) {
            inference_samples_.push_back(metrics.inference_ms);
          }
          if (std::isfinite(metrics.end_to_end_ms)) {
            end_to_end_samples_.push_back(metrics.end_to_end_ms);
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      if (!accepting_ || queue_.size() >= configuration_.queue_capacity) {
        records_lost_.fetch_add(1);
        return;
      }
      queue_.push_back(std::move(metrics));
    }
    queue_condition_.notify_one();
  } catch (const std::exception & exception) {
    records_lost_.fetch_add(1);
    statistics_samples_lost_.fetch_add(1);
    RCLCPP_ERROR(logger_, "Telemetry enqueue failed: %s", exception.what());
  } catch (...) {
    records_lost_.fetch_add(1);
    statistics_samples_lost_.fetch_add(1);
    RCLCPP_ERROR(logger_, "Telemetry enqueue failed with an unknown error.");
  }
}

void AcademicTelemetryLogger::stop()
{
  if (!configuration_.enabled) {
    return;
  }
  bool notify_writer = false;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!stop_requested_) {
      accepting_ = false;
      stop_requested_ = true;
      notify_writer = true;
    }
  }
  if (notify_writer) {
    queue_condition_.notify_all();
  }
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }
  if (output_.is_open()) {
    output_.flush();
    output_.close();
  }
}

TelemetryStatistics AcademicTelemetryLogger::statistics() const
{
  std::lock_guard<std::mutex> lock(statistics_mutex_);
  TelemetryStatistics result;
  result.records_enqueued = records_enqueued_;
  result.records_lost = records_lost_.load();
  result.statistics_samples_lost = statistics_samples_lost_.load();
  result.transport_latency = distribution(transport_samples_);
  result.decode_duration = distribution(decode_samples_);
  result.inference_duration = distribution(inference_samples_);
  result.end_to_end_latency = distribution(end_to_end_samples_);
  return result;
}

void AcademicTelemetryLogger::write_summary_json(const PipelineCounters & counters)
{
  if (!configuration_.enabled) {
    return;
  }
  const TelemetryStatistics telemetry = statistics();
  std::ofstream summary(configuration_.output_path + ".summary.json", std::ios::out | std::ios::trunc);
  if (!summary.is_open()) {
    writer_failed_.store(true);
    RCLCPP_ERROR(logger_, "Failed to write telemetry summary sidecar.");
    return;
  }
  summary.imbue(std::locale::classic());
  summary << std::fixed << std::setprecision(6);
  const auto write_distribution = [&summary](
    const std::string & name, const DistributionStatistics & values, const bool trailing_comma)
    {
      summary << "    \"" << name << "\": {\"samples\": " << values.sample_count
              << ", \"mean_ms\": ";
      AcademicTelemetryLogger::write_json_number(summary, values.mean_ms);
      summary << ", \"p95_ms\": ";
      AcademicTelemetryLogger::write_json_number(summary, values.p95_ms);
      summary << ", \"p99_ms\": ";
      AcademicTelemetryLogger::write_json_number(summary, values.p99_ms);
      summary << "}" << (trailing_comma ? "," : "") << "\n";
    };

  const uint64_t total_dropped = counters.dds_dropped_frames + counters.overwritten_frames +
    counters.stale_frames + counters.invalid_frames + counters.processing_failures;
  summary << "{\n"
          << "  \"run_id\": \"" << escape_json(configuration_.run_id) << "\",\n"
          << "  \"metrics_format\": \"" << configuration_.format << "\",\n"
          << "  \"decode_backend\": \"" << escape_json(counters.decode_backend) << "\",\n"
          << "  \"inference_backend\": \"" << escape_json(counters.inference_backend) << "\",\n"
          << "  \"expected_frame_rate_hz\": ";
  write_json_number(summary, counters.expected_frame_rate_hz);
  summary << ",\n"
          << "  \"records_enqueued\": " << telemetry.records_enqueued << ",\n"
          << "  \"records_lost\": " << telemetry.records_lost << ",\n"
          << "  \"statistics_samples_lost\": " << telemetry.statistics_samples_lost << ",\n"
          << "  \"received_frames\": " << counters.received_frames << ",\n"
          << "  \"processed_frames\": " << counters.processed_frames << ",\n"
          << "  \"dds_dropped_frames\": " << counters.dds_dropped_frames << ",\n"
          << "  \"overwritten_frames\": " << counters.overwritten_frames << ",\n"
          << "  \"stale_frames\": " << counters.stale_frames << ",\n"
          << "  \"invalid_frames\": " << counters.invalid_frames << ",\n"
          << "  \"processing_failures\": " << counters.processing_failures << ",\n"
          << "  \"total_dropped_frames\": " << total_dropped << ",\n"
          << "  \"average_fps\": ";
  write_json_number(summary, counters.average_fps);
  summary << ",\n  \"percentile_estimator\": \"Hyndman-Fan type 7\",\n"
          << "  \"distributions\": {\n";
  write_distribution("transport_latency", telemetry.transport_latency, true);
  write_distribution("decode_duration", telemetry.decode_duration, true);
  write_distribution("inference_duration", telemetry.inference_duration, true);
  write_distribution("end_to_end_latency", telemetry.end_to_end_latency, false);
  summary << "  }\n}\n";
  summary.flush();
  if (!summary.good()) {
    writer_failed_.store(true);
    RCLCPP_ERROR(logger_, "Telemetry summary sidecar entered a failed state.");
  }
}

bool AcademicTelemetryLogger::healthy() const noexcept
{
  return !writer_failed_.load() && records_lost_.load() == 0U &&
         statistics_samples_lost_.load() == 0U;
}

void AcademicTelemetryLogger::writer_loop() noexcept
{
  try {
    size_t records_since_flush = 0U;
    while (true) {
      FrameMetrics metrics;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this]() {
          return stop_requested_ || !queue_.empty();
        });
        if (queue_.empty() && stop_requested_) {
          break;
        }
        metrics = std::move(queue_.front());
        queue_.pop_front();
      }

      if (configuration_.format == "csv") {
        write_csv_record(metrics);
      } else {
        write_json_record(metrics);
      }
      ++records_since_flush;
      if (records_since_flush >= configuration_.flush_interval_records) {
        output_.flush();
        records_since_flush = 0U;
      }
      if (!output_.good()) {
        throw std::runtime_error("telemetry output stream entered a failed state");
      }
    }
  } catch (const std::exception & exception) {
    writer_failed_.store(true);
    uint64_t unpersisted = 1U;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      unpersisted += static_cast<uint64_t>(queue_.size());
      queue_.clear();
      accepting_ = false;
      stop_requested_ = true;
    }
    records_lost_.fetch_add(unpersisted);
    RCLCPP_ERROR(logger_, "Academic telemetry writer failed: %s", exception.what());
  } catch (...) {
    writer_failed_.store(true);
    uint64_t unpersisted = 1U;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      unpersisted += static_cast<uint64_t>(queue_.size());
      queue_.clear();
      accepting_ = false;
      stop_requested_ = true;
    }
    records_lost_.fetch_add(unpersisted);
    RCLCPP_ERROR(logger_, "Academic telemetry writer failed with an unknown error.");
  }
}

void AcademicTelemetryLogger::write_csv_header()
{
  output_ << "run_id,frame_sequence,sequence_source,acquisition_time_ns,receipt_time_ns,"
             "frame_size_bytes,frame_size_kib,transport_latency_ms,inter_arrival_ms,jitter_ms,"
             "queue_delay_ms,frame_age_at_inference_ms,decode_ms,detector_preprocess_ms,"
             "inference_ms,detector_postprocess_ms,annotation_encode_ms,end_to_end_ms,"
             "dds_gap_delta,dds_gap_total,overwrite_total,stale_total,invalid_total,"
             "processing_failure_total,pipeline_drop_total,detection_count,decode_backend,"
             "inference_backend,status\n";
}

void AcademicTelemetryLogger::write_csv_record(const FrameMetrics & metrics)
{
  output_ << csv_escape(metrics.run_id) << ',' << metrics.frame_sequence << ','
          << csv_escape(metrics.sequence_source) << ',' << metrics.acquisition_time_ns << ','
          << metrics.receipt_time_ns << ',' << metrics.frame_size_bytes << ',';
  write_csv_number(output_, static_cast<double>(metrics.frame_size_bytes) / kBytesPerKibibyte);
  output_ << ',';
  write_csv_number(output_, metrics.transport_latency_ms);
  output_ << ',';
  write_csv_number(output_, metrics.inter_arrival_ms);
  output_ << ',';
  write_csv_number(output_, metrics.jitter_ms);
  output_ << ',';
  write_csv_number(output_, metrics.queue_delay_ms);
  output_ << ',';
  write_csv_number(output_, metrics.frame_age_at_inference_ms);
  output_ << ',';
  write_csv_number(output_, metrics.decode_ms);
  output_ << ',';
  write_csv_number(output_, metrics.detector_preprocess_ms);
  output_ << ',';
  write_csv_number(output_, metrics.inference_ms);
  output_ << ',';
  write_csv_number(output_, metrics.detector_postprocess_ms);
  output_ << ',';
  write_csv_number(output_, metrics.annotation_encode_ms);
  output_ << ',';
  write_csv_number(output_, metrics.end_to_end_ms);
  output_ << ',' << metrics.dds_gap_delta << ',' << metrics.dds_gap_total << ','
          << metrics.overwrite_total << ',' << metrics.stale_total << ','
          << metrics.invalid_total << ',' << metrics.processing_failure_total << ','
          << metrics.pipeline_drop_total << ',' << metrics.detection_count << ','
          << csv_escape(metrics.decode_backend) << ',' << csv_escape(metrics.inference_backend)
          << ',' << csv_escape(metrics.status) << '\n';
}

void AcademicTelemetryLogger::write_json_record(const FrameMetrics & metrics)
{
  output_ << "{\"run_id\":\"" << escape_json(metrics.run_id)
          << "\",\"frame_sequence\":" << metrics.frame_sequence
          << ",\"sequence_source\":\"" << escape_json(metrics.sequence_source)
          << "\",\"acquisition_time_ns\":" << metrics.acquisition_time_ns
          << ",\"receipt_time_ns\":" << metrics.receipt_time_ns
          << ",\"frame_size_bytes\":" << metrics.frame_size_bytes
          << ",\"frame_size_kib\":";
  write_json_number(output_, static_cast<double>(metrics.frame_size_bytes) / kBytesPerKibibyte);
  output_ << ",\"transport_latency_ms\":";
  write_json_number(output_, metrics.transport_latency_ms);
  output_ << ",\"inter_arrival_ms\":";
  write_json_number(output_, metrics.inter_arrival_ms);
  output_ << ",\"jitter_ms\":";
  write_json_number(output_, metrics.jitter_ms);
  output_ << ",\"queue_delay_ms\":";
  write_json_number(output_, metrics.queue_delay_ms);
  output_ << ",\"frame_age_at_inference_ms\":";
  write_json_number(output_, metrics.frame_age_at_inference_ms);
  output_ << ",\"decode_ms\":";
  write_json_number(output_, metrics.decode_ms);
  output_ << ",\"detector_preprocess_ms\":";
  write_json_number(output_, metrics.detector_preprocess_ms);
  output_ << ",\"inference_ms\":";
  write_json_number(output_, metrics.inference_ms);
  output_ << ",\"detector_postprocess_ms\":";
  write_json_number(output_, metrics.detector_postprocess_ms);
  output_ << ",\"annotation_encode_ms\":";
  write_json_number(output_, metrics.annotation_encode_ms);
  output_ << ",\"end_to_end_ms\":";
  write_json_number(output_, metrics.end_to_end_ms);
  output_ << ",\"dds_gap_delta\":" << metrics.dds_gap_delta
          << ",\"dds_gap_total\":" << metrics.dds_gap_total
          << ",\"overwrite_total\":" << metrics.overwrite_total
          << ",\"stale_total\":" << metrics.stale_total
          << ",\"invalid_total\":" << metrics.invalid_total
          << ",\"processing_failure_total\":" << metrics.processing_failure_total
          << ",\"pipeline_drop_total\":" << metrics.pipeline_drop_total
          << ",\"detection_count\":" << metrics.detection_count
          << ",\"decode_backend\":\"" << escape_json(metrics.decode_backend)
          << "\",\"inference_backend\":\"" << escape_json(metrics.inference_backend)
          << "\",\"status\":\"" << escape_json(metrics.status) << "\"}\n";
}

std::string AcademicTelemetryLogger::escape_json(const std::string & value)
{
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
        break;
    }
  }
  return output.str();
}

DistributionStatistics AcademicTelemetryLogger::distribution(
  const std::vector<double> & samples)
{
  DistributionStatistics result;
  if (samples.empty()) {
    result.mean_ms = std::numeric_limits<double>::quiet_NaN();
    result.p95_ms = std::numeric_limits<double>::quiet_NaN();
    result.p99_ms = std::numeric_limits<double>::quiet_NaN();
    return result;
  }
  std::vector<double> sorted_samples = samples;
  std::sort(sorted_samples.begin(), sorted_samples.end());
  double sum = 0.0;
  for (const double sample : sorted_samples) {
    sum += sample;
  }
  result.sample_count = sorted_samples.size();
  result.mean_ms = sum / static_cast<double>(sorted_samples.size());
  result.p95_ms = percentile(sorted_samples, 0.95);
  result.p99_ms = percentile(sorted_samples, 0.99);
  return result;
}

double AcademicTelemetryLogger::percentile(
  const std::vector<double> & sorted_samples, const double probability)
{
  if (sorted_samples.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (sorted_samples.size() == 1U) {
    return sorted_samples.front();
  }
  // Hyndman-Fan type 7, matching NumPy's default linear quantile estimator.
  const double position = probability * static_cast<double>(sorted_samples.size() - 1U);
  const auto lower = static_cast<size_t>(std::floor(position));
  const auto upper = static_cast<size_t>(std::ceil(position));
  const double fraction = position - static_cast<double>(lower);
  return sorted_samples[lower] + fraction * (sorted_samples[upper] - sorted_samples[lower]);
}

void AcademicTelemetryLogger::write_json_number(std::ostream & stream, const double value)
{
  if (std::isfinite(value)) {
    stream << value;
  } else {
    stream << "null";
  }
}

}  // namespace gpu_vision_inference