#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "camera_info_manager/camera_info_manager.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace pi_camera_publisher
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

class PiCameraPublisher final : public rclcpp::Node
{
public:
  explicit PiCameraPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pi_camera_publisher", options)
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

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    auto joint_qos = rclcpp::SensorDataQoS().keep_last(5);
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      tilt_feedback_topic_, joint_qos,
      std::bind(&PiCameraPublisher::joint_state_callback, this, std::placeholders::_1));
    tilt_watchdog_timer_ = create_wall_timer(1s, std::bind(&PiCameraPublisher::tilt_watchdog, this));

    gst_init(nullptr, nullptr);
    try {
      start_pipeline();
      running_.store(true);
      capture_thread_ = std::thread(&PiCameraPublisher::capture_loop, this);
    } catch (...) {
      stop_pipeline();
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "CSI camera authority active: %dx%d@%d, image=%s, camera_info=%s, frame=%s",
      width_, height_, frame_rate_, image_topic_.c_str(), camera_info_topic_.c_str(),
      optical_frame_id_.c_str());
    if (publish_tilt_tf_) {
      RCLCPP_WARN(
        get_logger(),
        "Direct tilt TF broadcasting is enabled. Ensure robot_state_publisher is not also "
        "publishing %s -> %s.", tilt_parent_frame_.c_str(), tilt_child_frame_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Tilt feedback is monitored on %s; direct TF broadcast is disabled because "
        "robot_state_publisher is expected to own the joint transform.",
        tilt_feedback_topic_.c_str());
    }
  }

  ~PiCameraPublisher() override
  {
    running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    stop_pipeline();
  }

private:
  void declare_and_read_parameters()
  {
    camera_name_ = declare_parameter<std::string>("camera_name", "front_camera");
    camera_device_name_ = declare_parameter<std::string>("camera_device_name", "");
    camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
    optical_frame_id_ = declare_parameter<std::string>(
      "optical_frame_id", "camera_optical_frame");
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/image_raw/compressed");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    pipeline_override_ = declare_parameter<std::string>("gstreamer_pipeline", "");
    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 720);
    frame_rate_ = declare_parameter<int>("frame_rate", 30);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 85);
    image_lifespan_ms_ = declare_parameter<double>("image_lifespan_ms", 150.0);
    capture_latency_warning_ms_ = declare_parameter<double>(
      "capture_latency_warning_ms", 100.0);
    max_encoded_frame_bytes_ = declare_parameter<int64_t>(
      "max_encoded_frame_bytes", 4 * 1024 * 1024);
    require_valid_sensor_pts_ = declare_parameter<bool>("require_valid_sensor_pts", true);
    require_calibration_ = declare_parameter<bool>("require_calibration", false);
    strict_calibration_dimensions_ = declare_parameter<bool>(
      "strict_calibration_dimensions", true);

    tilt_feedback_topic_ = declare_parameter<std::string>(
      "tilt_feedback_topic", "/joint_states");
    tilt_joint_name_ = declare_parameter<std::string>(
      "tilt_joint_name", "camera_tilt_joint");
    publish_tilt_tf_ = declare_parameter<bool>("publish_tilt_tf", false);
    tilt_parent_frame_ = declare_parameter<std::string>(
      "tilt_parent_frame", "camera_tilt_mount_link");
    tilt_child_frame_ = declare_parameter<std::string>(
      "tilt_child_frame", "camera_link");
    tilt_min_rad_ = declare_parameter<double>("tilt_min_rad", -0.34906585040);
    tilt_max_rad_ = declare_parameter<double>("tilt_max_rad", 1.22173047640);
    tilt_feedback_timeout_ms_ = declare_parameter<double>(
      "tilt_feedback_timeout_ms", 500.0);
  }

  void validate_parameters() const
  {
    if (width_ <= 0 || height_ <= 0 || frame_rate_ <= 0) {
      throw std::invalid_argument("width, height, and frame_rate must be greater than zero");
    }
    if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      throw std::invalid_argument("jpeg_quality must be in [1, 100]");
    }
    if (!std::isfinite(image_lifespan_ms_) || image_lifespan_ms_ <= 0.0) {
      throw std::invalid_argument("image_lifespan_ms must be finite and greater than zero");
    }
    if (!std::isfinite(capture_latency_warning_ms_) || capture_latency_warning_ms_ <= 0.0) {
      throw std::invalid_argument(
              "capture_latency_warning_ms must be finite and greater than zero");
    }
    if (max_encoded_frame_bytes_ <= 0) {
      throw std::invalid_argument("max_encoded_frame_bytes must be greater than zero");
    }
    if (optical_frame_id_.empty() || image_topic_.empty() || camera_info_topic_.empty()) {
      throw std::invalid_argument("frame IDs and topic names must not be empty");
    }
    if (!std::isfinite(tilt_min_rad_) || !std::isfinite(tilt_max_rad_) ||
      tilt_min_rad_ >= tilt_max_rad_)
    {
      throw std::invalid_argument("tilt limits must be finite and ordered");
    }
    if (!std::isfinite(tilt_feedback_timeout_ms_) || tilt_feedback_timeout_ms_ <= 0.0) {
      throw std::invalid_argument(
              "tilt_feedback_timeout_ms must be finite and greater than zero");
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
        "No calibrated CameraInfo is loaded. Published CameraInfo will contain dimensions only.");
      return;
    }

    const auto info = camera_info_manager_->getCameraInfo();
    const bool dimensions_match =
      info.width == static_cast<uint32_t>(width_) && info.height == static_cast<uint32_t>(height_);
    if (!dimensions_match && strict_calibration_dimensions_) {
      std::ostringstream message;
      message << "calibration dimensions " << info.width << "x" << info.height
              << " do not match configured stream " << width_ << "x" << height_;
      throw std::runtime_error(message.str());
    }
    if (!dimensions_match) {
      RCLCPP_WARN(
        get_logger(), "Calibration is %ux%u while the configured stream is %dx%d.",
        info.width, info.height, width_, height_);
    }
  }

  std::string generated_pipeline() const
  {
    std::ostringstream pipeline;
    pipeline << "libcamerasrc name=camera_source ! "
             << "video/x-raw,width=" << width_ << ",height=" << height_
             << ",format=NV12,framerate=" << frame_rate_ << "/1 ! "
             << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
             << "jpegenc quality=" << jpeg_quality_ << " ! "
             << "appsink name=vision_sink emit-signals=false sync=false "
             << "max-buffers=1 drop=true wait-on-eos=false";
    return pipeline.str();
  }

  void start_pipeline()
  {
    const std::string pipeline_description =
      pipeline_override_.empty() ? generated_pipeline() : pipeline_override_;

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
      throw std::runtime_error(
              "GStreamer pipeline must contain an appsink named 'vision_sink'");
    }
    app_sink_ = GST_APP_SINK(sink_element);  // Holds the reference returned by get_by_name().

    if (!camera_device_name_.empty()) {
      GstElement * source = gst_bin_get_by_name(GST_BIN(pipeline_), "camera_source");
      if (source == nullptr) {
        stop_pipeline();
        throw std::runtime_error(
                "camera_device_name requires a libcamerasrc named 'camera_source'");
      }
      g_object_set(G_OBJECT(source), "camera-name", camera_device_name_.c_str(), nullptr);
      gst_object_unref(source);
    }

    const GstStateChangeReturn state_result = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_result == GST_STATE_CHANGE_FAILURE) {
      const std::string error = read_bus_error();
      stop_pipeline();
      throw std::runtime_error("GStreamer refused PLAYING state: " + error);
    }

    GstState current_state = GST_STATE_NULL;
    GstState pending_state = GST_STATE_NULL;
    const GstStateChangeReturn wait_result = gst_element_get_state(
      pipeline_, &current_state, &pending_state, 5 * GST_SECOND);
    (void)pending_state;
    if (wait_result == GST_STATE_CHANGE_FAILURE || current_state != GST_STATE_PLAYING) {
      const std::string error = read_bus_error();
      stop_pipeline();
      throw std::runtime_error("GStreamer did not reach PLAYING state: " + error);
    }

    RCLCPP_INFO(get_logger(), "GStreamer pipeline: %s", pipeline_description.c_str());
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
      return "pipeline is unavailable";
    }
    GstBus * bus = gst_element_get_bus(pipeline_);
    if (bus == nullptr) {
      return "GStreamer bus is unavailable";
    }
    GstMessage * message = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (message == nullptr) {
      return "no GStreamer error details were available";
    }

    std::string result;
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * error = nullptr;
      gchar * debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      result = error != nullptr ? error->message : "unknown GStreamer error";
      if (debug != nullptr) {
        result += " [" + std::string(debug) + "]";
        g_free(debug);
      }
      if (error != nullptr) {
        g_error_free(error);
      }
    } else {
      result = "unexpected end of stream";
    }
    gst_message_unref(message);
    return result;
  }

  void capture_loop() noexcept
  {
    try {
      while (running_.load() && rclcpp::ok()) {
        GstSamplePtr sample(
          gst_app_sink_try_pull_sample(app_sink_, 200 * GST_MSECOND));
        if (!sample) {
          if (gst_app_sink_is_eos(app_sink_)) {
            throw std::runtime_error("camera pipeline reached end of stream");
          }
          check_pipeline_bus();
          continue;
        }
        publish_sample(sample.get());
      }
    } catch (const std::exception & exception) {
      capture_failed_.store(true);
      RCLCPP_FATAL(get_logger(), "Camera capture stopped: %s", exception.what());
    } catch (...) {
      capture_failed_.store(true);
      RCLCPP_FATAL(get_logger(), "Camera capture stopped due to an unknown exception");
    }
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

    std::string detail = "unexpected end of stream";
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      GError * error = nullptr;
      gchar * debug = nullptr;
      gst_message_parse_error(message, &error, &debug);
      detail = error != nullptr ? error->message : "unknown GStreamer error";
      if (debug != nullptr) {
        detail += " [" + std::string(debug) + "]";
        g_free(debug);
      }
      if (error != nullptr) {
        g_error_free(error);
      }
    }
    gst_message_unref(message);
    throw std::runtime_error(detail);
  }

  std::optional<rclcpp::Time> acquisition_stamp(GstBuffer * buffer)
  {
    const GstClockTime pts = GST_BUFFER_PTS(buffer);
    if (!GST_CLOCK_TIME_IS_VALID(pts)) {
      if (require_valid_sensor_pts_) {
        return std::nullopt;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Frame has no GStreamer PTS; falling back to callback time.");
      return now();
    }

    GstClock * pipeline_clock = gst_element_get_clock(pipeline_);
    if (pipeline_clock == nullptr) {
      return std::nullopt;
    }
    const GstClockTime clock_now = gst_clock_get_time(pipeline_clock);
    gst_object_unref(pipeline_clock);
    const GstClockTime base_time = gst_element_get_base_time(pipeline_);

    if (!GST_CLOCK_TIME_IS_VALID(base_time) ||
      pts > std::numeric_limits<GstClockTime>::max() - base_time)
    {
      return std::nullopt;
    }
    const GstClockTime capture_clock_time = base_time + pts;
    if (capture_clock_time > clock_now) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Camera PTS is in the future relative to the GStreamer clock; frame dropped.");
      return std::nullopt;
    }

    const GstClockTime capture_age_ns = clock_now - capture_clock_time;
    const double capture_age_ms = static_cast<double>(capture_age_ns) / 1.0e6;
    if (capture_age_ms > capture_latency_warning_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Camera pipeline age is %.1f ms before DDS publication.", capture_age_ms);
    }

    const rclcpp::Time ros_now = now();
    const int64_t age_ns = capture_age_ns > static_cast<GstClockTime>(
      std::numeric_limits<int64_t>::max()) ?
      std::numeric_limits<int64_t>::max() : static_cast<int64_t>(capture_age_ns);
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
      !gst_structure_get_int(structure, "height", &sample_height))
    {
      throw std::runtime_error("JPEG caps do not contain width and height");
    }
    if (sample_width != width_ || sample_height != height_) {
      throw std::runtime_error("negotiated camera dimensions differ from configured dimensions");
    }

    const auto stamp = acquisition_stamp(buffer);
    if (!stamp.has_value()) {
      invalid_timestamp_frames_.fetch_add(1);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "A frame was dropped because no trustworthy acquisition timestamp was available.");
      return;
    }

    GstMapInfo map_info = GST_MAP_INFO_INIT;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) {
      throw std::runtime_error("failed to map encoded camera buffer");
    }

    try {
      if (map_info.size == 0U ||
        map_info.size > static_cast<size_t>(max_encoded_frame_bytes_))
      {
        throw std::runtime_error("encoded camera frame violates configured size bounds");
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

      // Publish the small calibration sample first. Both messages carry the identical sensor stamp.
      camera_info_publisher_->publish(camera_info);
      image_publisher_->publish(std::move(image));
      published_frames_.fetch_add(1);
      observe_sensor_sequence(buffer);
    } catch (...) {
      if (map_info.data != nullptr) {
        gst_buffer_unmap(buffer, &map_info);
      }
      throw;
    }
  }

  void observe_sensor_sequence(GstBuffer * buffer)
  {
    const guint64 sequence = GST_BUFFER_OFFSET(buffer);
    if (sequence == GST_BUFFER_OFFSET_NONE) {
      return;
    }
    std::lock_guard<std::mutex> lock(sequence_mutex_);
    if (last_sensor_sequence_.has_value() && sequence > *last_sensor_sequence_ + 1U) {
      const guint64 missed = sequence - *last_sensor_sequence_ - 1U;
      sensor_drop_count_.fetch_add(missed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "libcamera sensor sequence reports dropped frames; cumulative drops=%llu.",
        static_cast<unsigned long long>(sensor_drop_count_.load()));
    }
    last_sensor_sequence_ = sequence;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr message)
  {
    const auto iterator = std::find(message->name.begin(), message->name.end(), tilt_joint_name_);
    if (iterator == message->name.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "JointState does not contain required tilt joint '%s'.", tilt_joint_name_.c_str());
      return;
    }
    const auto index = static_cast<size_t>(std::distance(message->name.begin(), iterator));
    if (index >= message->position.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "JointState position array is shorter than its name array.");
      return;
    }

    const double angle = message->position[index];
    if (!std::isfinite(angle) || angle < tilt_min_rad_ || angle > tilt_max_rad_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected camera tilt angle %.6f rad outside [%.6f, %.6f].",
        angle, tilt_min_rad_, tilt_max_rad_);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(tilt_mutex_);
      last_tilt_feedback_ = std::chrono::steady_clock::now();
      last_tilt_angle_rad_ = angle;
    }

    if (!publish_tilt_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message->header.stamp;
    if (transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0U) {
      transform.header.stamp = now().to_msg();
    }
    transform.header.frame_id = tilt_parent_frame_;
    transform.child_frame_id = tilt_child_frame_;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = std::sin(angle * 0.5);
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = std::cos(angle * 0.5);
    tf_broadcaster_->sendTransform(transform);
  }

  void tilt_watchdog()
  {
    std::optional<std::chrono::steady_clock::time_point> last_feedback;
    {
      std::lock_guard<std::mutex> lock(tilt_mutex_);
      last_feedback = last_tilt_feedback_;
    }
    if (!last_feedback.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "No camera tilt feedback has been observed on %s.", tilt_feedback_topic_.c_str());
      return;
    }
    const double age_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - *last_feedback).count();
    if (age_ms > tilt_feedback_timeout_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Camera tilt feedback is stale (%.1f ms); dynamic extrinsics are not trustworthy.", age_ms);
    }
    if (capture_failed_.load()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Camera capture worker is not running.");
    }
  }

  std::string camera_name_;
  std::string camera_device_name_;
  std::string camera_info_url_;
  std::string optical_frame_id_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string pipeline_override_;
  int width_{0};
  int height_{0};
  int frame_rate_{0};
  int jpeg_quality_{0};
  double image_lifespan_ms_{0.0};
  double capture_latency_warning_ms_{0.0};
  int64_t max_encoded_frame_bytes_{0};
  bool require_valid_sensor_pts_{true};
  bool require_calibration_{false};
  bool strict_calibration_dimensions_{true};

  std::string tilt_feedback_topic_;
  std::string tilt_joint_name_;
  bool publish_tilt_tf_{false};
  std::string tilt_parent_frame_;
  std::string tilt_child_frame_;
  double tilt_min_rad_{0.0};
  double tilt_max_rad_{0.0};
  double tilt_feedback_timeout_ms_{0.0};

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr tilt_watchdog_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  GstElement * pipeline_{nullptr};
  GstAppSink * app_sink_{nullptr};
  std::atomic<bool> running_{false};
  std::atomic<bool> capture_failed_{false};
  std::thread capture_thread_;

  std::mutex sequence_mutex_;
  std::optional<guint64> last_sensor_sequence_;
  std::atomic<uint64_t> published_frames_{0};
  std::atomic<uint64_t> sensor_drop_count_{0};
  std::atomic<uint64_t> invalid_timestamp_frames_{0};

  std::mutex tilt_mutex_;
  std::optional<std::chrono::steady_clock::time_point> last_tilt_feedback_;
  double last_tilt_angle_rad_{0.0};
};

}  // namespace pi_camera_publisher

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<pi_camera_publisher::PiCameraPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pi_camera_publisher"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}