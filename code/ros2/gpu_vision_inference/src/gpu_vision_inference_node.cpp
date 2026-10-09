#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef GPU_VISION_HAS_NVJPEG
#include <cuda_runtime_api.h>
#include <nvjpeg.h>
#endif

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "distributed_vision_msgs/msg/detection2_d.hpp"
#include "distributed_vision_msgs/msg/detection2_d_array.hpp"
#include "gpu_vision_inference/academic_telemetry.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace gpu_vision_inference
{

using namespace std::chrono_literals;

struct Detection
{
  uint32_t class_id{0};
  std::string class_name;
  float confidence{0.0F};
  cv::Rect bounding_box;
};

struct DetectorResult
{
  std::vector<Detection> detections;
  double preprocess_ms{0.0};
  double forward_ms{0.0};
  double postprocess_ms{0.0};
};

struct FrameEnvelope
{
  sensor_msgs::msg::CompressedImage::ConstSharedPtr message;
  FrameMetrics metrics;
  std::chrono::steady_clock::time_point receipt_steady;
};

struct DetectorConfiguration
{
  std::string model_path;
  std::string labels_path;
  int input_width{640};
  int input_height{640};
  float confidence_threshold{0.35F};
  float nms_threshold{0.45F};
  bool output_has_objectness{false};
  bool boxes_are_normalized{false};
  bool require_gpu{true};
  bool allow_cpu_fallback{false};
  bool use_fp16{true};
  int warmup_iterations{2};
  int max_detections{100};
};

struct LetterboxResult
{
  cv::Mat image;
  float scale{1.0F};
  int pad_x{0};
  int pad_y{0};
};

#ifdef GPU_VISION_HAS_NVJPEG
class NvJpegDecoder final
{
public:
  NvJpegDecoder()
  {
    try {
      check_nvjpeg(nvjpegCreateSimple(&handle_), "nvjpegCreateSimple");
      check_nvjpeg(nvjpegJpegStateCreate(handle_, &state_), "nvjpegJpegStateCreate");
      check_cuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
    } catch (...) {
      release();
      throw;
    }
  }

  ~NvJpegDecoder()
  {
    release();
  }

  NvJpegDecoder(const NvJpegDecoder &) = delete;
  NvJpegDecoder & operator=(const NvJpegDecoder &) = delete;

  cv::Mat decode(const std::vector<uint8_t> & encoded)
  {
    if (encoded.empty()) {
      throw std::invalid_argument("nvJPEG input is empty");
    }
    int component_count = 0;
    nvjpegChromaSubsampling_t subsampling{};
    int widths[NVJPEG_MAX_COMPONENT]{};
    int heights[NVJPEG_MAX_COMPONENT]{};
    check_nvjpeg(
      nvjpegGetImageInfo(
        handle_, encoded.data(), encoded.size(), &component_count, &subsampling,
        widths, heights),
      "nvjpegGetImageInfo");
    (void)component_count;
    (void)subsampling;
    const int width = widths[0];
    const int height = heights[0];
    if (width <= 0 || height <= 0) {
      throw std::runtime_error("nvJPEG reported invalid image dimensions");
    }
    const size_t pitch = static_cast<size_t>(width) * 3U;
    if (static_cast<size_t>(height) > std::numeric_limits<size_t>::max() / pitch) {
      throw std::overflow_error("decoded JPEG allocation size overflow");
    }
    const size_t required_bytes = pitch * static_cast<size_t>(height);
    ensure_device_capacity(required_bytes);

    nvjpegImage_t output{};
    output.channel[0] = device_output_;
    output.pitch[0] = pitch;
    check_nvjpeg(
      nvjpegDecode(
        handle_, state_, encoded.data(), encoded.size(), NVJPEG_OUTPUT_BGRI,
        &output, stream_),
      "nvjpegDecode");

    cv::Mat decoded(height, width, CV_8UC3);
    check_cuda(
      cudaMemcpy2DAsync(
        decoded.data, decoded.step, device_output_, pitch, pitch,
        static_cast<size_t>(height), cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpy2DAsync");
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    return decoded;
  }

  void warm_up(const int width, const int height)
  {
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    std::vector<uint8_t> encoded;
    if (!cv::imencode(".jpg", image, encoded, {cv::IMWRITE_JPEG_QUALITY, 75})) {
      throw std::runtime_error("failed to create nvJPEG warm-up image");
    }
    const cv::Mat decoded = decode(encoded);
    if (decoded.cols != width || decoded.rows != height) {
      throw std::runtime_error("nvJPEG warm-up returned unexpected dimensions");
    }
  }

private:
  static void check_nvjpeg(const nvjpegStatus_t status, const char * operation)
  {
    if (status != NVJPEG_STATUS_SUCCESS) {
      throw std::runtime_error(
              std::string(operation) + " failed with nvJPEG status " +
              std::to_string(static_cast<int>(status)));
    }
  }

  static void check_cuda(const cudaError_t status, const char * operation)
  {
    if (status != cudaSuccess) {
      throw std::runtime_error(
              std::string(operation) + " failed: " + cudaGetErrorString(status));
    }
  }

  void ensure_device_capacity(const size_t required_bytes)
  {
    if (required_bytes <= device_capacity_bytes_) {
      return;
    }
    if (device_output_ != nullptr) {
      check_cuda(cudaFree(device_output_), "cudaFree");
      device_output_ = nullptr;
      device_capacity_bytes_ = 0U;
    }
    check_cuda(
      cudaMalloc(reinterpret_cast<void **>(&device_output_), required_bytes),
      "cudaMalloc");
    device_capacity_bytes_ = required_bytes;
  }

  void release() noexcept
  {
    if (stream_ != nullptr) {
      (void)cudaStreamSynchronize(stream_);
    }
    if (device_output_ != nullptr) {
      (void)cudaFree(device_output_);
      device_output_ = nullptr;
    }
    if (stream_ != nullptr) {
      (void)cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
    if (state_ != nullptr) {
      (void)nvjpegJpegStateDestroy(state_);
      state_ = nullptr;
    }
    if (handle_ != nullptr) {
      (void)nvjpegDestroy(handle_);
      handle_ = nullptr;
    }
    device_capacity_bytes_ = 0U;
  }

  nvjpegHandle_t handle_{nullptr};
  nvjpegJpegState_t state_{nullptr};
  cudaStream_t stream_{nullptr};
  unsigned char * device_output_{nullptr};
  size_t device_capacity_bytes_{0U};
};
#endif

class OpenCvYoloDetector
{
public:
  OpenCvYoloDetector(DetectorConfiguration configuration, rclcpp::Logger logger)
  : configuration_(std::move(configuration)), logger_(std::move(logger))
  {
    validate_configuration();
    labels_ = read_labels(configuration_.labels_path);
    try {
      network_ = cv::dnn::readNetFromONNX(configuration_.model_path);
    } catch (const cv::Exception & exception) {
      throw std::runtime_error("failed to load ONNX model: " + std::string(exception.what()));
    }
    if (network_.empty()) {
      throw std::runtime_error("OpenCV returned an empty DNN for the configured model");
    }
    output_names_ = network_.getUnconnectedOutLayersNames();
    if (output_names_.empty()) {
      throw std::runtime_error("ONNX model has no discoverable output layer");
    }

    configure_backend();
    const cv::Mat warmup_image(
      configuration_.input_height, configuration_.input_width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int iteration = 0; iteration < configuration_.warmup_iterations; ++iteration) {
      try {
        (void)detect_internal(warmup_image);
      } catch (const cv::Exception & exception) {
        if (!activate_cpu_fallback(exception.what())) {
          throw std::runtime_error(
                  "DNN warm-up failed on the configured backend: " +
                  std::string(exception.what()));
        }
        (void)detect_internal(warmup_image);
        break;
      }
    }

    RCLCPP_INFO(
      logger_, "Loaded detector model '%s' with backend %s.",
      configuration_.model_path.c_str(), backend_name().c_str());
  }

  DetectorResult detect(const cv::Mat & image)
  {
    try {
      return detect_internal(image);
    } catch (const cv::Exception & exception) {
      if (!activate_cpu_fallback(exception.what())) {
        throw;
      }
      return detect_internal(image);
    }
  }

  std::string backend_name() const
  {
    if (gpu_active_.load()) {
      return configuration_.use_fp16 ? "opencv_dnn_cuda_fp16" : "opencv_dnn_cuda";
    }
    return "opencv_dnn_cpu";
  }

private:
  void validate_configuration() const
  {
    if (configuration_.model_path.empty()) {
      throw std::invalid_argument("model_path is required");
    }
    if (configuration_.input_width <= 0 || configuration_.input_height <= 0) {
      throw std::invalid_argument("detector input dimensions must be greater than zero");
    }
    if (!std::isfinite(configuration_.confidence_threshold) ||
      configuration_.confidence_threshold <= 0.0F ||
      configuration_.confidence_threshold >= 1.0F)
    {
      throw std::invalid_argument("confidence_threshold must be in (0, 1)");
    }
    if (!std::isfinite(configuration_.nms_threshold) ||
      configuration_.nms_threshold <= 0.0F || configuration_.nms_threshold >= 1.0F)
    {
      throw std::invalid_argument("nms_threshold must be in (0, 1)");
    }
    if (configuration_.warmup_iterations < 0 || configuration_.max_detections <= 0) {
      throw std::invalid_argument("warmup_iterations and max_detections are invalid");
    }
  }

  static std::vector<std::string> read_labels(const std::string & path)
  {
    std::vector<std::string> labels;
    if (path.empty()) {
      return labels;
    }
    std::ifstream input(path);
    if (!input.is_open()) {
      throw std::runtime_error("failed to open labels file: " + path);
    }
    std::string line;
    while (std::getline(input, line)) {
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (!line.empty()) {
        labels.push_back(line);
      }
    }
    return labels;
  }

  void configure_backend()
  {
    int cuda_devices = 0;
    try {
      cuda_devices = cv::cuda::getCudaEnabledDeviceCount();
    } catch (const cv::Exception &) {
      cuda_devices = 0;
    }

    if (cuda_devices > 0) {
      network_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      network_.setPreferableTarget(
        configuration_.use_fp16 ? cv::dnn::DNN_TARGET_CUDA_FP16 : cv::dnn::DNN_TARGET_CUDA);
      gpu_active_.store(true);
      return;
    }

    if (configuration_.require_gpu && !configuration_.allow_cpu_fallback) {
      throw std::runtime_error(
              "no CUDA device/OpenCV CUDA build is available and CPU fallback is disabled");
    }
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    gpu_active_.store(false);
    RCLCPP_WARN(logger_, "CUDA DNN is unavailable; detector is using the CPU backend.");
  }

  bool activate_cpu_fallback(const std::string & failure)
  {
    if (!gpu_active_.load() || !configuration_.allow_cpu_fallback) {
      return false;
    }
    bool expected = false;
    if (!cpu_fallback_activated_.compare_exchange_strong(expected, true)) {
      return false;
    }
    RCLCPP_ERROR(
      logger_, "CUDA inference failed (%s); permanently switching this process to CPU.",
      failure.c_str());
    network_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    gpu_active_.store(false);
    return true;
  }

  LetterboxResult letterbox(const cv::Mat & source) const
  {
    const float horizontal_scale =
      static_cast<float>(configuration_.input_width) / static_cast<float>(source.cols);
    const float vertical_scale =
      static_cast<float>(configuration_.input_height) / static_cast<float>(source.rows);
    const float scale = std::min(horizontal_scale, vertical_scale);
    const int resized_width = std::max(1, static_cast<int>(std::lround(source.cols * scale)));
    const int resized_height = std::max(1, static_cast<int>(std::lround(source.rows * scale)));
    const int pad_x = (configuration_.input_width - resized_width) / 2;
    const int pad_y = (configuration_.input_height - resized_height) / 2;

    LetterboxResult result;
    result.image = cv::Mat(
      configuration_.input_height, configuration_.input_width, CV_8UC3,
      cv::Scalar(114, 114, 114));
    cv::Mat resized;
    cv::resize(source, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);
    resized.copyTo(result.image(cv::Rect(pad_x, pad_y, resized_width, resized_height)));
    result.scale = scale;
    result.pad_x = pad_x;
    result.pad_y = pad_y;
    return result;
  }

  DetectorResult detect_internal(const cv::Mat & image)
  {
    if (image.empty() || image.type() != CV_8UC3) {
      throw std::invalid_argument("detector input must be a non-empty CV_8UC3 image");
    }
    const auto preprocess_start = std::chrono::steady_clock::now();
    const LetterboxResult prepared = letterbox(image);
    cv::Mat blob;
    cv::dnn::blobFromImage(
      prepared.image, blob, 1.0 / 255.0,
      cv::Size(configuration_.input_width, configuration_.input_height),
      cv::Scalar(), true, false, CV_32F);
    network_.setInput(blob);
    const double preprocess_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - preprocess_start).count();

    std::vector<cv::Mat> outputs;
    const auto forward_start = std::chrono::steady_clock::now();
    network_.forward(outputs, output_names_);
    const double forward_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - forward_start).count();
    if (outputs.empty()) {
      throw std::runtime_error("detector produced no output tensors");
    }
    const auto postprocess_start = std::chrono::steady_clock::now();
    DetectorResult result;
    result.detections = decode_yolo_output(outputs.front(), prepared, image.size());
    result.preprocess_ms = preprocess_ms;
    result.forward_ms = forward_ms;
    result.postprocess_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - postprocess_start).count();
    return result;
  }

  std::vector<Detection> decode_yolo_output(
    const cv::Mat & raw_output, const LetterboxResult & prepared,
    const cv::Size & source_size) const
  {
    if (raw_output.dims != 3 || raw_output.size[0] != 1) {
      throw std::runtime_error("expected YOLO output with shape [1,N,F] or [1,F,N]");
    }

    cv::Mat continuous_output;
    if (raw_output.type() == CV_32F && raw_output.isContinuous()) {
      continuous_output = raw_output;
    } else {
      raw_output.convertTo(continuous_output, CV_32F);
    }

    const int dimension_one = continuous_output.size[1];
    const int dimension_two = continuous_output.size[2];
    cv::Mat candidates_by_features;
    if (dimension_one < dimension_two) {
      cv::Mat features_by_candidates = continuous_output.reshape(1, dimension_one);
      cv::transpose(features_by_candidates, candidates_by_features);
    } else {
      candidates_by_features = continuous_output.reshape(1, dimension_one);
    }

    const int feature_count = candidates_by_features.cols;
    const int score_offset = configuration_.output_has_objectness ? 5 : 4;
    const int class_count = feature_count - score_offset;
    if (class_count <= 0) {
      throw std::runtime_error("YOLO output does not contain class scores");
    }
    if (!labels_.empty() && static_cast<int>(labels_.size()) != class_count) {
      RCLCPP_WARN_ONCE(
        logger_, "Labels file contains %zu names but model output contains %d classes.",
        labels_.size(), class_count);
    }

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    boxes.reserve(static_cast<size_t>(candidates_by_features.rows));
    confidences.reserve(static_cast<size_t>(candidates_by_features.rows));
    class_ids.reserve(static_cast<size_t>(candidates_by_features.rows));

    for (int row_index = 0; row_index < candidates_by_features.rows; ++row_index) {
      const float * row = candidates_by_features.ptr<float>(row_index);
      const float objectness = configuration_.output_has_objectness ? row[4] : 1.0F;
      cv::Mat scores(1, class_count, CV_32F, const_cast<float *>(row + score_offset));
      cv::Point maximum_location;
      double maximum_score = 0.0;
      cv::minMaxLoc(scores, nullptr, &maximum_score, nullptr, &maximum_location);
      const float confidence = objectness * static_cast<float>(maximum_score);
      if (!std::isfinite(confidence) || confidence < configuration_.confidence_threshold) {
        continue;
      }

      float center_x = row[0];
      float center_y = row[1];
      float box_width = row[2];
      float box_height = row[3];
      if (configuration_.boxes_are_normalized) {
        center_x *= static_cast<float>(configuration_.input_width);
        center_y *= static_cast<float>(configuration_.input_height);
        box_width *= static_cast<float>(configuration_.input_width);
        box_height *= static_cast<float>(configuration_.input_height);
      }
      if (!std::isfinite(center_x) || !std::isfinite(center_y) ||
        !std::isfinite(box_width) || !std::isfinite(box_height) ||
        box_width <= 0.0F || box_height <= 0.0F)
      {
        continue;
      }

      float left = (center_x - box_width * 0.5F - static_cast<float>(prepared.pad_x)) /
        prepared.scale;
      float top = (center_y - box_height * 0.5F - static_cast<float>(prepared.pad_y)) /
        prepared.scale;
      float right = (center_x + box_width * 0.5F - static_cast<float>(prepared.pad_x)) /
        prepared.scale;
      float bottom = (center_y + box_height * 0.5F - static_cast<float>(prepared.pad_y)) /
        prepared.scale;
      left = std::clamp(left, 0.0F, static_cast<float>(source_size.width - 1));
      top = std::clamp(top, 0.0F, static_cast<float>(source_size.height - 1));
      right = std::clamp(right, 0.0F, static_cast<float>(source_size.width));
      bottom = std::clamp(bottom, 0.0F, static_cast<float>(source_size.height));
      const int x_min = static_cast<int>(std::floor(left));
      const int y_min = static_cast<int>(std::floor(top));
      const int x_max = static_cast<int>(std::ceil(right));
      const int y_max = static_cast<int>(std::ceil(bottom));
      if (x_max <= x_min || y_max <= y_min) {
        continue;
      }

      boxes.emplace_back(x_min, y_min, x_max - x_min, y_max - y_min);
      confidences.push_back(confidence);
      class_ids.push_back(maximum_location.x);
    }

    std::unordered_map<int, std::vector<int>> candidates_per_class;
    for (size_t index = 0; index < class_ids.size(); ++index) {
      candidates_per_class[class_ids[index]].push_back(static_cast<int>(index));
    }

    std::vector<int> retained_indices;
    for (const auto & entry : candidates_per_class) {
      std::vector<cv::Rect> class_boxes;
      std::vector<float> class_confidences;
      class_boxes.reserve(entry.second.size());
      class_confidences.reserve(entry.second.size());
      for (const int global_index : entry.second) {
        class_boxes.push_back(boxes[static_cast<size_t>(global_index)]);
        class_confidences.push_back(confidences[static_cast<size_t>(global_index)]);
      }
      std::vector<int> class_retained;
      cv::dnn::NMSBoxes(
        class_boxes, class_confidences, configuration_.confidence_threshold,
        configuration_.nms_threshold, class_retained);
      for (const int local_index : class_retained) {
        retained_indices.push_back(entry.second[static_cast<size_t>(local_index)]);
      }
    }

    std::sort(
      retained_indices.begin(), retained_indices.end(),
      [&confidences](const int left, const int right) {
        return confidences[static_cast<size_t>(left)] >
               confidences[static_cast<size_t>(right)];
      });
    if (retained_indices.size() > static_cast<size_t>(configuration_.max_detections)) {
      retained_indices.resize(static_cast<size_t>(configuration_.max_detections));
    }

    std::vector<Detection> detections;
    detections.reserve(retained_indices.size());
    for (const int index : retained_indices) {
      const int class_id = class_ids[static_cast<size_t>(index)];
      Detection detection;
      detection.class_id = static_cast<uint32_t>(std::max(0, class_id));
      detection.class_name = class_id >= 0 && class_id < static_cast<int>(labels_.size()) ?
        labels_[static_cast<size_t>(class_id)] : "class_" + std::to_string(class_id);
      detection.confidence = confidences[static_cast<size_t>(index)];
      detection.bounding_box = boxes[static_cast<size_t>(index)];
      detections.push_back(std::move(detection));
    }
    return detections;
  }

  DetectorConfiguration configuration_;
  rclcpp::Logger logger_;
  cv::dnn::Net network_;
  std::vector<std::string> output_names_;
  std::vector<std::string> labels_;
  std::atomic<bool> gpu_active_{false};
  std::atomic<bool> cpu_fallback_activated_{false};
};

class GpuVisionInference final : public rclcpp::Node
{
public:
  explicit GpuVisionInference(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gpu_vision_inference", options)
  {
    declare_and_read_parameters();
    validate_parameters();
    detector_ = std::make_unique<OpenCvYoloDetector>(detector_configuration_, get_logger());
    configure_decoder();
    telemetry_logger_ = std::make_unique<AcademicTelemetryLogger>(
      telemetry_configuration_, get_logger());

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    output_qos.best_effort().durability_volatile();
    output_qos.lifespan(rclcpp::Duration::from_seconds(output_lifespan_ms_ / 1000.0));
    annotated_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      annotated_topic_, output_qos);
    detections_publisher_ =
      create_publisher<distributed_vision_msgs::msg::Detection2DArray>(
      detections_topic_, output_qos);

    auto diagnostics_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    diagnostics_qos.reliable().durability_volatile();
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, diagnostics_qos);

    auto input_qos = rclcpp::SensorDataQoS().keep_last(1);
    image_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_, input_qos,
      std::bind(
        &GpuVisionInference::image_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    diagnostics_timer_ = create_wall_timer(
      1s, std::bind(&GpuVisionInference::publish_diagnostics, this));

    // Start the worker only after every potentially-throwing ROS entity is constructed.
    running_.store(true);
    worker_thread_ = std::thread(&GpuVisionInference::worker_loop, this);

    RCLCPP_INFO(
      get_logger(), "Inference accelerator active: %s -> %s and %s (decode=%s, inference=%s)",
      input_topic_.c_str(), detections_topic_.c_str(), annotated_topic_.c_str(),
      decode_backend_name_.c_str(), detector_->backend_name().c_str());
  }

  ~GpuVisionInference() override
  {
    running_.store(false);
    pending_condition_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    telemetry_logger_->stop();
    if (telemetry_configuration_.enabled) {
      report_summary();
    }
  }

private:
  void declare_and_read_parameters()
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/camera/image_raw/compressed");
    annotated_topic_ = declare_parameter<std::string>(
      "annotated_topic", "/vision/gpu/image_annotated/compressed");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/vision/gpu/detections");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics/vision_gpu");

    detector_configuration_.model_path = declare_parameter<std::string>("model_path", "");
    detector_configuration_.labels_path = declare_parameter<std::string>("labels_path", "");
    detector_configuration_.input_width = declare_parameter<int>("model_input_width", 640);
    detector_configuration_.input_height = declare_parameter<int>("model_input_height", 640);
    detector_configuration_.confidence_threshold = static_cast<float>(
      declare_parameter<double>("confidence_threshold", 0.35));
    detector_configuration_.nms_threshold = static_cast<float>(
      declare_parameter<double>("nms_threshold", 0.45));
    detector_configuration_.output_has_objectness = declare_parameter<bool>(
      "output_has_objectness", false);
    detector_configuration_.boxes_are_normalized = declare_parameter<bool>(
      "boxes_are_normalized", false);
    detector_configuration_.require_gpu = declare_parameter<bool>("require_gpu", true);
    detector_configuration_.allow_cpu_fallback = declare_parameter<bool>(
      "allow_cpu_fallback", false);
    detector_configuration_.use_fp16 = declare_parameter<bool>("use_fp16", true);
    detector_configuration_.warmup_iterations = declare_parameter<int>("warmup_iterations", 2);
    detector_configuration_.max_detections = declare_parameter<int>("max_detections", 100);

    annotated_jpeg_quality_ = declare_parameter<int>("annotated_jpeg_quality", 80);
    output_lifespan_ms_ = declare_parameter<double>("output_lifespan_ms", 250.0);
    latency_warning_ms_ = declare_parameter<double>("latency_warning_ms", 150.0);
    stale_frame_drop_ms_ = declare_parameter<double>("stale_frame_drop_ms", 500.0);
    future_stamp_tolerance_ms_ = declare_parameter<double>(
      "future_stamp_tolerance_ms", 50.0);
    no_frame_timeout_ms_ = declare_parameter<double>("no_frame_timeout_ms", 1000.0);
    max_compressed_input_bytes_ = declare_parameter<int64_t>(
      "max_compressed_input_bytes", 4 * 1024 * 1024);
    prefer_gpu_decode_ = declare_parameter<bool>("prefer_gpu_decode", true);
    require_gpu_decode_ = declare_parameter<bool>("require_gpu_decode", false);
    decode_warmup_width_ = declare_parameter<int>("decode_warmup_width", 1280);
    decode_warmup_height_ = declare_parameter<int>("decode_warmup_height", 720);

    expected_frame_rate_hz_ = declare_parameter<double>("expected_frame_rate_hz", 30.0);
    telemetry_configuration_.enabled = declare_parameter<bool>("metrics_enabled", false);
    telemetry_configuration_.output_path = declare_parameter<std::string>(
      "metrics_output_path", "/tmp/vision_benchmark.csv");
    telemetry_configuration_.format = declare_parameter<std::string>("metrics_format", "csv");
    telemetry_configuration_.run_id = declare_parameter<std::string>("metrics_run_id", "default");
    const int64_t queue_capacity = declare_parameter<int64_t>(
      "metrics_queue_capacity", 8192);
    const int64_t flush_interval = declare_parameter<int64_t>(
      "metrics_flush_interval_frames", 30);
    const int64_t statistics_capacity = declare_parameter<int64_t>(
      "metrics_statistics_capacity", 1000000);
    telemetry_configuration_.append = declare_parameter<bool>("metrics_append", false);
    if (queue_capacity > 0) {
      telemetry_configuration_.queue_capacity = static_cast<size_t>(queue_capacity);
    } else {
      telemetry_configuration_.queue_capacity = 0U;
    }
    if (flush_interval > 0) {
      telemetry_configuration_.flush_interval_records = static_cast<size_t>(flush_interval);
    } else {
      telemetry_configuration_.flush_interval_records = 0U;
    }
    if (statistics_capacity > 0) {
      telemetry_configuration_.statistics_capacity = static_cast<size_t>(statistics_capacity);
    } else {
      telemetry_configuration_.statistics_capacity = 0U;
    }
  }

  void validate_parameters() const
  {
    if (input_topic_.empty() || annotated_topic_.empty() || detections_topic_.empty() ||
      diagnostics_topic_.empty())
    {
      throw std::invalid_argument("topic names must not be empty");
    }
    if (annotated_jpeg_quality_ < 1 || annotated_jpeg_quality_ > 100) {
      throw std::invalid_argument("annotated_jpeg_quality must be in [1, 100]");
    }
    const auto positive_finite = [](const double value) {
        return std::isfinite(value) && value > 0.0;
      };
    if (!positive_finite(output_lifespan_ms_) || !positive_finite(latency_warning_ms_) ||
      !positive_finite(stale_frame_drop_ms_) || !positive_finite(future_stamp_tolerance_ms_) ||
      !positive_finite(no_frame_timeout_ms_))
    {
      throw std::invalid_argument("latency and timeout parameters must be finite and positive");
    }
    if (stale_frame_drop_ms_ <= latency_warning_ms_) {
      throw std::invalid_argument("stale_frame_drop_ms must be greater than latency_warning_ms");
    }
    if (max_compressed_input_bytes_ <= 0) {
      throw std::invalid_argument("max_compressed_input_bytes must be greater than zero");
    }
    if (require_gpu_decode_ && !prefer_gpu_decode_) {
      throw std::invalid_argument("require_gpu_decode=true requires prefer_gpu_decode=true");
    }
    if (decode_warmup_width_ <= 0 || decode_warmup_height_ <= 0) {
      throw std::invalid_argument("decode warm-up dimensions must be greater than zero");
    }
    if (!positive_finite(expected_frame_rate_hz_)) {
      throw std::invalid_argument("expected_frame_rate_hz must be finite and positive");
    }
    if (telemetry_configuration_.queue_capacity == 0U ||
      telemetry_configuration_.flush_interval_records == 0U ||
      telemetry_configuration_.statistics_capacity == 0U)
    {
      throw std::invalid_argument("metrics queue and flush parameters must be positive");
    }
  }

  void configure_decoder()
  {
    if (!prefer_gpu_decode_) {
      decode_backend_name_ = "opencv_imgcodecs_cpu";
      return;
    }
#ifdef GPU_VISION_HAS_NVJPEG
    try {
      nvjpeg_decoder_ = std::make_unique<NvJpegDecoder>();
      nvjpeg_decoder_->warm_up(decode_warmup_width_, decode_warmup_height_);
      decode_backend_name_ = "nvjpeg_cuda_to_host";
      RCLCPP_INFO(get_logger(), "JPEG decoder backend: %s", decode_backend_name_.c_str());
      return;
    } catch (const std::exception & exception) {
      if (require_gpu_decode_) {
        throw std::runtime_error(
                "required nvJPEG decoder initialization failed: " +
                std::string(exception.what()));
      }
      RCLCPP_WARN(
        get_logger(), "nvJPEG initialization failed (%s); using CPU JPEG decode.",
        exception.what());
    }
#else
    if (require_gpu_decode_) {
      throw std::runtime_error(
              "require_gpu_decode=true but this binary was built without nvJPEG");
    }
    RCLCPP_WARN(
      get_logger(), "This binary was built without nvJPEG; using CPU JPEG decode.");
#endif
    decode_backend_name_ = "opencv_imgcodecs_cpu";
  }

  cv::Mat decode_image(const std::vector<uint8_t> & encoded)
  {
#ifdef GPU_VISION_HAS_NVJPEG
    if (nvjpeg_decoder_) {
      return nvjpeg_decoder_->decode(encoded);
    }
#endif
    return cv::imdecode(encoded, cv::IMREAD_COLOR);
  }

  void image_callback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr message,
    const rclcpp::MessageInfo & message_info)
  {
    const uint64_t local_sequence = received_frames_.fetch_add(1) + 1U;
    const rclcpp::Time receipt_ros = now();
    const auto receipt_steady = std::chrono::steady_clock::now();

    FrameEnvelope envelope;
    envelope.message = message;
    envelope.receipt_steady = receipt_steady;
    envelope.metrics.receipt_time_ns = receipt_ros.nanoseconds();
    envelope.metrics.frame_size_bytes = message->data.size();
    envelope.metrics.decode_backend = decode_backend_name_;
    envelope.metrics.inference_backend = detector_->backend_name();

    const rmw_message_info_t & rmw_info = message_info.get_rmw_message_info();
    const bool middleware_sequence_supported =
      rmw_info.publication_sequence_number != RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED;
    envelope.metrics.frame_sequence = middleware_sequence_supported ?
      rmw_info.publication_sequence_number : local_sequence;
    envelope.metrics.sequence_source = middleware_sequence_supported ?
      "rmw_publication" : "subscriber_local";

    uint64_t dds_gap_delta = 0U;
    {
      std::lock_guard<std::mutex> lock(ingress_mutex_);
      if (last_arrival_steady_.has_value()) {
        envelope.metrics.inter_arrival_ms = std::chrono::duration<double, std::milli>(
          receipt_steady - *last_arrival_steady_).count();
        const double expected_period_ms = 1000.0 / expected_frame_rate_hz_;
        envelope.metrics.jitter_ms = std::abs(
          envelope.metrics.inter_arrival_ms - expected_period_ms);
      }
      last_arrival_steady_ = receipt_steady;

      if (middleware_sequence_supported) {
        std::array<uint8_t, RMW_GID_STORAGE_SIZE> current_gid{};
        std::memcpy(current_gid.data(), rmw_info.publisher_gid.data, current_gid.size());
        const bool same_publisher = last_publisher_gid_.has_value() &&
          *last_publisher_gid_ == current_gid;
        if (same_publisher && last_publication_sequence_.has_value() &&
          rmw_info.publication_sequence_number > *last_publication_sequence_)
        {
          const uint64_t sequence_delta =
            rmw_info.publication_sequence_number - *last_publication_sequence_;
          if (sequence_delta > 1U) {
            dds_gap_delta = sequence_delta - 1U;
          }
          last_publication_sequence_ = rmw_info.publication_sequence_number;
        } else if (!same_publisher || !last_publication_sequence_.has_value()) {
          last_publication_sequence_ = rmw_info.publication_sequence_number;
        } else {
          // A wrap or publisher-side sequence reset is not a measurable frame gap.
          last_publication_sequence_ = rmw_info.publication_sequence_number;
        }
        last_publisher_gid_ = current_gid;
      }
    }
    if (dds_gap_delta > 0U) {
      dds_dropped_frames_.fetch_add(dds_gap_delta);
    }
    envelope.metrics.dds_gap_delta = dds_gap_delta;
    envelope.metrics.dds_gap_total = dds_dropped_frames_.load();

    {
      std::lock_guard<std::mutex> lock(metrics_mutex_);
      last_received_steady_ = receipt_steady;
    }

    if (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0U) {
      invalid_frames_.fetch_add(1);
      envelope.metrics.status = "invalid_zero_timestamp";
      record_metrics(std::move(envelope.metrics));
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping image with a zero acquisition timestamp.");
      return;
    }

    const rclcpp::Time acquisition_time(message->header.stamp);
    envelope.metrics.acquisition_time_ns = acquisition_time.nanoseconds();
    envelope.metrics.transport_latency_ms = (receipt_ros - acquisition_time).seconds() * 1000.0;
    if (message->data.empty() ||
      message->data.size() > static_cast<size_t>(max_compressed_input_bytes_))
    {
      invalid_frames_.fetch_add(1);
      envelope.metrics.status = "invalid_encoded_size";
      record_metrics(std::move(envelope.metrics));
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping compressed image outside configured byte bounds.");
      return;
    }

    std::optional<FrameEnvelope> overwritten;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      if (pending_frame_.has_value()) {
        overwritten = std::move(pending_frame_);
        const uint64_t overwrite_total = overwritten_frames_.fetch_add(1) + 1U;
        overwritten->metrics.overwrite_total = overwrite_total;
        overwritten->metrics.status = "overwritten_before_processing";
      }
      envelope.metrics.overwrite_total = overwritten_frames_.load();
      pending_frame_ = std::move(envelope);
    }
    if (overwritten.has_value()) {
      record_metrics(std::move(overwritten->metrics));
    }
    pending_condition_.notify_one();
  }

  void worker_loop() noexcept
  {
    while (true) {
      std::optional<FrameEnvelope> envelope;
      {
        std::unique_lock<std::mutex> lock(pending_mutex_);
        pending_condition_.wait(lock, [this]() {
          return !running_.load() || pending_frame_.has_value();
        });
        if (!running_.load() && !pending_frame_.has_value()) {
          break;
        }
        envelope = std::move(pending_frame_);
        pending_frame_.reset();
      }

      try {
        process_frame(*envelope);
      } catch (const cv::Exception & exception) {
        record_inference_error();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "OpenCV inference failure: %s", exception.what());
      } catch (const std::exception & exception) {
        record_inference_error();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Inference worker failure: %s", exception.what());
      } catch (...) {
        record_inference_error();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Inference worker failed with an unknown exception.");
      }
    }
  }

  void record_inference_error()
  {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    last_error_steady_ = std::chrono::steady_clock::now();
  }

  void record_metrics(FrameMetrics metrics)
  {
    metrics.dds_gap_total = dds_dropped_frames_.load();
    metrics.overwrite_total = overwritten_frames_.load();
    metrics.stale_total = stale_frames_.load();
    metrics.invalid_total = invalid_frames_.load();
    metrics.processing_failure_total = inference_errors_.load();
    metrics.pipeline_drop_total = metrics.dds_gap_total + metrics.overwrite_total +
      metrics.stale_total + metrics.invalid_total + metrics.processing_failure_total;
    telemetry_logger_->enqueue(std::move(metrics));