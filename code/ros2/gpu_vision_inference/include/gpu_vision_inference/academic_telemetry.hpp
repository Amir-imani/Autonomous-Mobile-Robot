#ifndef GPU_VISION_INFERENCE__ACADEMIC_TELEMETRY_HPP_
#define GPU_VISION_INFERENCE__ACADEMIC_TELEMETRY_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/logger.hpp"

namespace gpu_vision_inference
{

struct FrameMetrics
{
  std::string run_id;
  uint64_t frame_sequence{0};
  std::string sequence_source;
  int64_t acquisition_time_ns{0};
  int64_t receipt_time_ns{0};
  size_t frame_size_bytes{0};
  double transport_latency_ms{std::numeric_limits<double>::quiet_NaN()};
  double inter_arrival_ms{std::numeric_limits<double>::quiet_NaN()};
  double jitter_ms{std::numeric_limits<double>::quiet_NaN()};
  double queue_delay_ms{std::numeric_limits<double>::quiet_NaN()};
  double frame_age_at_inference_ms{std::numeric_limits<double>::quiet_NaN()};
  double decode_ms{std::numeric_limits<double>::quiet_NaN()};
  double detector_preprocess_ms{std::numeric_limits<double>::quiet_NaN()};
  double inference_ms{std::numeric_limits<double>::quiet_NaN()};
  double detector_postprocess_ms{std::numeric_limits<double>::quiet_NaN()};
  double annotation_encode_ms{std::numeric_limits<double>::quiet_NaN()};
  double end_to_end_ms{std::numeric_limits<double>::quiet_NaN()};
  uint64_t dds_gap_delta{0};
  uint64_t dds_gap_total{0};
  uint64_t overwrite_total{0};
  uint64_t stale_total{0};
  uint64_t invalid_total{0};
  uint64_t processing_failure_total{0};
  uint64_t pipeline_drop_total{0};
  uint32_t detection_count{0};
  std::string decode_backend;
  std::string inference_backend;
  std::string status;
};

struct DistributionStatistics { size_t sample_count{0}; double mean_ms{0.0}; double p95_ms{0.0}; double p99_ms{0.0}; };
struct TelemetryStatistics { uint64_t records_enqueued{0}; uint64_t records_lost{0}; uint64_t statistics_samples_lost{0}; DistributionStatistics transport_latency; DistributionStatistics decode_duration; DistributionStatistics inference_duration; DistributionStatistics end_to_end_latency; };
struct PipelineCounters { std::string decode_backend; std::string inference_backend; double expected_frame_rate_hz{0.0}; uint64_t received_frames{0}; uint64_t processed_frames{0}; uint64_t dds_dropped_frames{0}; uint64_t overwritten_frames{0}; uint64_t stale_frames{0}; uint64_t invalid_frames{0}; uint64_t processing_failures{0}; double average_fps{0.0}; };

class AcademicTelemetryLogger final
{
public:
  struct Configuration
  {
    bool enabled{false};
    std::string output_path;
    std::string format{"csv"};
    std::string run_id{"default"};
    size_t queue_capacity{8192};
    size_t flush_interval_records{30};
    size_t statistics_capacity{1000000};
    bool append{false};
  };

  AcademicTelemetryLogger(Configuration configuration, rclcpp::Logger logger);
  ~AcademicTelemetryLogger();
  AcademicTelemetryLogger(const AcademicTelemetryLogger &) = delete;
  AcademicTelemetryLogger & operator=(const AcademicTelemetryLogger &) = delete;
  void enqueue(FrameMetrics metrics) noexcept;
  void stop();
  TelemetryStatistics statistics() const;
  void write_summary_json(const PipelineCounters & counters);
  bool healthy() const noexcept;

private:
  void writer_loop() noexcept;
  void write_csv_header();
  void write_csv_record(const FrameMetrics & metrics);
  void write_json_record(const FrameMetrics & metrics);
  static std::string escape_json(const std::string & value);
  static DistributionStatistics distribution(const std::vector<double> & samples);
  static double percentile(const std::vector<double> & sorted_samples, double probability);
  static void write_json_number(std::ostream & stream, double value);

  Configuration configuration_;
  rclcpp::Logger logger_;
  std::ofstream output_;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::deque<FrameMetrics> queue_;
  bool accepting_{true};
  bool stop_requested_{false};
  std::thread writer_thread_;
  mutable std::mutex statistics_mutex_;
  uint64_t records_enqueued_{0};
  size_t statistics_frame_count_{0};
  std::vector<double> transport_samples_;
  std::vector<double> decode_samples_;
  std::vector<double> inference_samples_;
  std::vector<double> end_to_end_samples_;
  std::atomic<uint64_t> records_lost_{0};
  std::atomic<uint64_t> statistics_samples_lost_{0};
  std::atomic<bool> writer_failed_{false};
};

}  // namespace gpu_vision_inference

#endif  // GPU_VISION_INFERENCE__ACADEMIC_TELEMETRY_HPP_
