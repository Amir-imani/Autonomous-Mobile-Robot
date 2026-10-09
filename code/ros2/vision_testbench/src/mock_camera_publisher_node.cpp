#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace vision_testbench
{

using namespace std::chrono_literals;

struct GstSampleDeleter
{
  void operator()(GstSample * sample) const noexcept
  {
    if (sample != nullptr) {
      gst_sample_unref(sample);
    }
  }
};

using GstSamplePtr = std::unique_ptr<GstSample, GstSampleDeleter>;

class MockCameraPublisher final : public rclcpp::Node
{
public:
  explicit MockCameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mock_camera_publisher", options)
  {
    declare_and_read_parameters();
    validate_parameters();

    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name_, camera_info_url_);
    validate_calibration();

    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    image_qos.best_effort().durability_volatile();
    image_qos.lifespan(rclcpp::Duration::from_seconds(image_lifespan_ms_ / 1000.0));
    image_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      image_topic_, image_qos);

    auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    camera_info_qos.reliable().transient_local();
    camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, camera_info_qos);

    gst_init(nullptr, nullptr);
    try {
      prepare_pipeline();
      running_.store(true);
      capture_thread_ = std::thread(&MockCameraPublisher::capture_loop, this);
    } catch (...) {
      stop_pipeline();
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "Mock camera ready: source=%s, %dx%d@%d, image=%s, frame=%s",
      source_type_.c_str(), width_, height_, frame_rate_, image_topic_.c_str(),
      optical_frame_id_.c_str());
  }

  ~MockCameraPublisher() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    stop_pipeline();
  }

  bool capture_failed() const noexcept
  {
    return capture_failed_.load();
  }

private:
  void declare_and_read_parameters()
  {
    source_type_ = declare_parameter<std::string>("source_type", "video_file");
    video_path_ = declare_parameter<std::string>("video_path", "");
    webcam_device_ = declare_parameter<std::string>("webcam_device", "/dev/video0");
    test_pattern_ = declare_parameter<std::string>("test_pattern", "smpte");
    camera_name_ = declare_parameter<std::string>("camera_name", "benchmark_camera");
    camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
    optical_frame_id_ = declare_parameter<std::string>(
      "optical_frame_id", "camera_optical_frame");
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/image_raw/compressed");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    frame_rate_ = declare_parameter<int>("frame_rate", 30);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 85);
    image_lifespan_ms_ = declare_parameter<double>("image_lifespan_ms", 150.0);
    timestamp_warning_ms_ = declare_parameter<double>("timestamp_warning_ms", 100.0);
    max_encoded_frame_bytes_ = declare_parameter<int64_t>(
      "max_encoded_frame_bytes", 4 * 1024 * 1024);
    max_frames_ = declare_parameter<int64_t>("max_frames", 0);
    wait_for_subscriber_ = declare_parameter<bool>("wait_for_subscriber", true);
    subscriber_wait_timeout_s_ = declare_parameter<double>(
      "subscriber_wait_timeout_s", 30.0);
    shutdown_grace_ms_ = declare_parameter<double>("shutdown_grace_ms", 250.0);
    require_calibration_ = declare_parameter<bool>("require_calibration", false);
    strict_calibration_dimensions_ = declare_parameter<bool>(
      "strict_calibration_dimensions", true);
  }

  void validate_parameters() const
  {
    if (source_type_ != "video_file" && source_type_ != "webcam" &&
      source_type_ != "test_pattern")
    {
      throw std::invalid_argument(
              "source_type must be 'video_file', 'webcam', or 'test_pattern'");
    }
    if (source_type_ == "video_file" && video_path_.empty()) {
      throw std::invalid_argument("video_path is required for source_type=video_file");
    }
    if (source_type_ == "webcam" && webcam_device_.empty()) {
      throw std::invalid_argument("webcam_device is required for source_type=webcam");
    }
    if (width_ <= 0 || height_ <= 0 || frame_rate_ <= 0) {
      throw std::invalid_argument("width, height, and frame_rate must be greater than zero");
    }
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      throw std::invalid_argument("jpeg_quality must be in [1, 100]");
    }
    const auto positive_finite = [](const double value) {
        return std::isfinite(value) && value > 0.0;
      };
    if (!positive_finite(image_lifespan_ms_) || !positive_finite(timestamp_warning_ms_) ||
      !positive_finite(subscriber_wait_timeout_s_) || !std::isfinite(shutdown_grace_ms_) ||
      shutdown_grace_ms_ < 0.0)
    {
      throw std::invalid_argument(
              "lifespan, timestamp warning, wait timeout, and shutdown grace are invalid");
    }
    if (max_encoded_frame_bytes_ <= 0 || max_frames_ < 0) {
      throw std::invalid_argument("encoded size must be positive and max_frames must be nonnegative");
    }
    if (optical_frame_id_.empty() || image_topic_.empty() || camera_info_topic_.empty()) {
      throw std::invalid_argument("frame IDs and topic names must not be empty");
    }
  }

  void validate_calibration()
  {
    if (!camera_info_manager_->isCalibrated()) {
      if (require_calibration_) {
        throw std::runtime_error("camera calibration is required but camera_info_url is invalid");
      }
      RCLCPP_WARN(
        get_logger(),
        "No calibrated CameraInfo is loaded; benchmark CameraInfo contains dimensions only.");
      return;
    }
    const auto info = camera_info_manager_->getCameraInfo();
    const bool dimensions_match = info.width == static_cast<uint32_t>(width_) &&
      info.height == static_cast<uint32_t>(height_);
    if (!dimensions_match && strict_calibration_dimensions_) {
      throw std::runtime_error("calibration dimensions do not match the benchmark stream");
    }
    if (!dimensions_match) {
      RCLCPP_WARN(
        get_logger(), "Calibration is %ux%u while the benchmark stream is %dx%d.",
        info.width, info.height, width_, height_);
    }
  }

  std::string generated_pipeline() const
  {
    std::ostringstream pipeline;
    if (source_type_ == "video_file") {
      pipeline << "uridecodebin name=mock_source caps=video/x-raw ! ";
    } else if (source_type_ == "webcam") {
      pipeline << "v4l2src name=mock_source do-timestamp=true ! decodebin ! ";
    } else {
      pipeline << "videotestsrc name=mock_source is-live=true do-timestamp=true ! ";
    }
    pipeline << "videoconvert ! videoscale ! videorate ! "
             << "video/x-raw,width=" << width_ << ",height=" << height_
             << ",format=NV12,framerate=" << frame_rate_ << "/1 ! ";
    if (source_type_ == "video_file") {
      // Pace raw frames before JPEG encoding so encoder cost remains visible
      // in acquisition-to-publication latency, as it is on the live Pi path.
      pipeline << "identity sync=true ! ";
    }
    pipeline << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
             << "leaky=downstream ! "
             << "jpegenc quality=" << jpeg_quality_ << " ! "
             << "appsink name=vision_sink emit-signals=false sync=false "
             << "max-buffers=1 drop=true wait-on-eos=false";
    return pipeline.str();
  }

  void prepare_pipeline()
  {
    const std::string pipeline_description = generated_pipeline();
    GError * parse_error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description.c_str(), &parse_error);
    if (pipeline_ == nullptr) {
      const std::string detail = parse_error != nullptr ? parse_error->message : "unknown error";
      if (parse_error != nullptr) {
        g_error_free(parse_error);
      }
      throw std::runtime_error("failed to construct GStreamer pipeline: " + detail);
    }
    if (parse_error != nullptr) {
      const std::string detail = parse_error->message;
      g_error_free(parse_error);
      stop_pipeline();
      throw std::runtime_error("GStreamer pipeline was only partially constructed: " + detail);
    }

    GstElement * sink_element = gst_bin_get_by_name(GST_BIN(pipeline_), "vision_sink");
    if (sink_element == nullptr || !GST_IS_APP_SINK(sink_element)) {
      if (sink_element != nullptr) {
        gst_object_unref(sink_element);
      }
      stop_pipeline();
      throw std::runtime_error("pipeline does not contain appsink 'vision_sink'");
    }
    app_sink_ = GST_APP_SINK(sink_element);

    GstElement * source = gst_bin_get_by_name(GST_BIN(pipeline_), "mock_source");
    if (source == nullptr) {
      stop_pipeline();
      throw std::runtime_error("pipeline does not contain source 'mock_source'");
    }
    if (source_type_ == "video_file") {
      GError * uri_error = nullptr;
      gchar * canonical_path = g_canonicalize_filename(video_path_.c_str(), nullptr);
      gchar * uri = g_filename_to_uri(canonical_path, nullptr, &uri_error);
      g_free(canonical_path);
      if (uri == nullptr) {
        const std::string detail = uri_error != nullptr ? uri_error->message : "unknown error";
        if (uri_error != nullptr) {
          g_error_free(uri_error);
        }
        gst_object_unref(source);
        stop_pipeline();
        throw std::runtime_error("failed to convert video_path to a file URI: " + detail);
      }
      g_object_set(G_OBJECT(source), "uri", uri, nullptr);
      g_free(uri);
    } else if (source_type_ == "webcam") {
      g_object_set(G_OBJECT(source), "device", webcam_device_.c_str(), nullptr);
    } else {
      gst_util_set_object_arg(G_OBJECT(source), "pattern", test_pattern_.c_str());
    }
    gst_object_unref(source);
    RCLCPP_INFO(get_logger(), "GStreamer pipeline: %s", pipeline_description.c_str());
  }

  void start_pipeline()
  {
    const GstStateChangeReturn state_result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("GStreamer refused PLAYING state: " + read_bus_error());
    }
    GstState current_state = GST_STATE_NULL;
    GstState pending_state = GST_STATE_NULL;
    const GstStateChangeReturn wait_result = gst_element_get_state(
      pipeline_, &current_state, &pending_state, 10 * GST_SECOND);
    (void)pending_state;
    if (wait_result == GST_STATE_CHANGE_FAILURE || current_state != GST_STATE_PLAYING) {
      throw std::runtime_error("GStreamer did not reach PLAYING state: " + read_bus_error());
    }
  }

  void stop_pipeline() noexcept
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (app_sink_ != nullptr) {
      gst_object_unref(app_sink_);
      app_sink_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  std::string read_bus_error() const
  {
    if (pipeline_ == nullptr) {
      return "pipeline unavailable";
    }
    GstBus * bus = gst_element_get_bus(pipeline_);
    if (bus == nullptr) {
      return "GStreamer bus unavailable";
    }
    GstMessage * message = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (message == nullptr) {
      return "no GStreamer error detail available";
    }
    const std::string result = describe_bus_message(message);
    gst_message_unref(message);
    return result;
  }

  static std::string describe_bus_message(GstMessage * message)
  {
    if (GST_MESSAGE_TYPE(message) != GST_MESSAGE_ERROR) {
      return "end of stream";
    }
    GError * error = nullptr;
    gchar * debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    std::string result = error != nullptr ? error->message : "unknown GStreamer error";
    if (debug != nullptr) {
      result += " [" + std::string(debug) + "]";
      g_free(debug);
    }
    if (error != nullptr) {
      g_error_free(error);
    }
    return result;
  }

  bool wait_for_inference_subscriber()
  {
    if (!wait_for_subscriber_) {
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(subscriber_wait_timeout_s_);
    while (running_.load() && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (image_publisher_->get_subscription_count() > 0U) {
        RCLCPP_INFO(get_logger(), "Image subscriber discovered; starting timed playback.");
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    return false;
  }

  void capture_loop() noexcept
  {
    try {
      if (!wait_for_inference_subscriber()) {
        throw std::runtime_error("timed out waiting for an image subscriber");
      }
      start_pipeline();
      while (running_.load() && rclcpp::ok()) {
        GstSamplePtr sample(gst_app_sink_try_pull_sample(app_sink_, 200 * GST_MSECOND));
        if (!sample) {
          if (gst_app_sink_is_eos(app_sink_)) {
            finish_benchmark("video end of stream");
            return;
          }
          check_pipeline_bus();
          continue;
        }
        publish_sample(sample.get());
        if (max_frames_ > 0 && published_frames_.load() >= static_cast<uint64_t>(max_frames_)) {
          finish_benchmark("configured frame limit reached");
          return;
        }
      }
    } catch (const std::exception & exception) {
      capture_failed_.store(true);
      RCLCPP_FATAL(get_logger(), "Mock capture stopped: %s", exception.what());
      get_node_base_interface()->get_context()->shutdown("mock camera failure");
    } catch (...) {
      capture_failed_.store(true);
      RCLCPP_FATAL(get_logger(), "Mock capture stopped due to an unknown exception.");
      get_node_base_interface()->get_context()->shutdown("mock camera failure");
    }
  }

  void finish_benchmark(const std::string & reason)
  {
    RCLCPP_INFO(
      get_logger(), "Mock stream complete (%s): published=%llu timestamp_drops=%llu.",
      reason.c_str(), static_cast<unsigned long long>(published_frames_.load()),
      static_cast<unsigned long long>(invalid_timestamp_frames_.load()));
    if (shutdown_grace_ms_ > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(shutdown_grace_ms_));
    }
    get_node_base_interface()->get_context()->shutdown("mock stream complete");
  }

  void check_pipeline_bus()
  {
    GstBus * bus = gst_element_get_bus(pipeline_);
    if (bus == nullptr) {
      throw std::runtime_error("GStreamer bus became unavailable");
    }
    GstMessage * message = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (message == nullptr) {
      return;
    }
    const GstMessageType type = GST_MESSAGE_TYPE(message);
    const std::string detail = describe_bus_message(message);
    gst_message_unref(message);
    if (type == GST_MESSAGE_EOS) {
      finish_benchmark(detail);
      return;
    }
    throw std::runtime_error(detail);
  }

  std::optional<rclcpp::Time> acquisition_stamp(GstSample * sample, GstBuffer * buffer)
  {
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    const GstSegment * segment = gst_sample_get_segment(sample);
    if (!GST_CLOCK_TIME_IS_VALID(pts) || segment == nullptr || segment->format != GST_FORMAT_TIME) {
      return std::nullopt;
    }
    const GstClockTime running_time = gst_segment_to_running_time(
      segment, GST_FORMAT_TIME, pts);
    if (!GST_CLOCK_TIME_IS_VALID(running_time)) {
      return std::nullopt;
    }

    GstClock * pipeline_clock = gst_element_get_clock(pipeline_);
    if (pipeline_clock == nullptr) {
      return std::nullopt;
    }
    const GstClockTime clock_now = gst_clock_get_time(pipeline_clock);
    gst_object_unref(pipeline_clock);
    const GstClockTime base_time = gst_element_get_base_time(pipeline_);
    if (!GST_CLOCK_TIME_IS_VALID(base_time) ||
      running_time > std::numeric_limits<GstClockTime>::max() - base_time)
    {
      return std::nullopt;
    }
    const GstClockTime acquisition_clock_time = base_time + running_time;
    if (acquisition_clock_time > clock_now) {
      return std::nullopt;
    }

    const GstClockTime age_ns_unsigned = clock_now - acquisition_clock_time;
    const double age_ms = static_cast<double>(age_ns_unsigned) / 1.0e6;
    if (age_ms > timestamp_warning_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Mock encode/publish path is %.1f ms behind its acquisition PTS.", age_ms);
    }
    const int64_t age_ns = age_ns_unsigned > static_cast<GstClockTime>(
      std::numeric_limits<int64_t>::max()) ?
      std::numeric_limits<int64_t>::max() : static_cast<int64_t>(age_ns_unsigned);
    const rclcpp::Time ros_now = now();
    if (ros_now.nanoseconds() < age_ns) {
      return std::nullopt;
    }
    return rclcpp::Time(ros_now.nanoseconds() - age_ns, ros_now.get_clock_type());
  }

  void publish_sample(GstSample * sample)
  {
    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstCaps * caps = gst_sample_get_caps(sample);
    if (buffer == nullptr || caps == nullptr || gst_caps_is_empty(caps)) {
      throw std::runtime_error("GStreamer returned a sample without a buffer or caps");
    }
    const GstStructure * structure = gst_caps_get_structure(caps, 0);
    const gchar * media_type = gst_structure_get_name(structure);
    if (media_type == nullptr || std::string(media_type) != "image/jpeg") {
      throw std::runtime_error("vision_sink must output image/jpeg");
    }
    int sample_width = 0;
    int sample_height = 0;
    if (!gst_structure_get_int(structure, "width", &sample_width) ||
      !gst_structure_get_int(structure, "height", &sample_height) ||
      sample_width != width_ || sample_height != height_)
    {
      throw std::runtime_error("negotiated dimensions differ from benchmark configuration");
    }

    const auto stamp = acquisition_stamp(sample, buffer);
    if (!stamp.has_value()) {
      invalid_timestamp_frames_.fetch_add(1);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Dropping mock frame without a trustworthy acquisition PTS.");
      return;
    }

    GstMapInfo map_info = GST_MAP_INFO_INIT;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
      throw std::runtime_error("failed to map encoded mock frame");
    }
    try {
      if (map_info.size == 0U ||
        map_info.size > static_cast<size_t>(max_encoded_frame_bytes_))
      {
        throw std::runtime_error("encoded mock frame violates configured size bounds");
      }
      sensor_msgs::msg::CompressedImage image;
      image.header.stamp = stamp->to_msg();
      image.header.frame_id = optical_frame_id_;
      image.format = "rgb8; jpeg compressed rgb8";
      image.data.assign(map_info.data, map_info.data + map_info.size);
      gst_buffer_unmap(buffer, &map_info);
      map_info.data = nullptr;

      sensor_msgs::msg::CameraInfo camera_info = camera_info_manager_->getCameraInfo();
      camera_info.header = image.header;
      if (!camera_info_manager_->isCalibrated()) {
        camera_info.width = static_cast<uint32_t>(width_);
        camera_info.height = static_cast<uint32_t>(height_);
      }
      camera_info_publisher_->publish(camera_info);
      image_publisher_->publish(std::move(image));
      published_frames_.fetch_add(1);
    } catch (...) {
      if (map_info.data != nullptr) {
        gst_buffer_unmap(buffer, &map_info);
      }
      throw;
    }
  }

  std::string source_type_;
  std::string video_path_;
  std::string webcam_device_;
  std::string test_pattern_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string optical_frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  int width_{1280};
  int height_{720};
  int frame_rate_{30};
  int jpeg_quality_{85};
  double image_lifespan_ms_{150.0};
  double timestamp_warning_ms_{100.0};
  int64_t max_encoded_frame_bytes_{0};
  int64_t max_frames_{0};
  bool wait_for_subscriber_{true};
  double subscriber_wait_timeout_s_{30.0};
  double shutdown_grace_ms_{250.0};
  bool require_calibration_{false};
  bool strict_calibration_dimensions_{true};

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  GstElement * pipeline_{nullptr};
  GstAppSink * app_sink_{nullptr};
  std::atomic<bool> running_{false};
  std::thread capture_thread_;
  std::atomic<uint64_t> published_frames_{0};
  std::atomic<uint64_t> invalid_timestamp_frames_{0};
  std::atomic<bool> capture_failed_{false};
};

}  // namespace vision_testbench

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<vision_testbench::MockCameraPublisher>();
    rclcpp::spin(node);
    exit_code = node->capture_failed() ? 1 : 0;
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("mock_camera_publisher"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return exit_code;
}