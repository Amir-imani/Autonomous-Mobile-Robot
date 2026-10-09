# The header is copied from the source image, including its acquisition stamp.
std_msgs/Header header
uint32 image_width
uint32 image_height
float32 source_frame_age_ms
float32 inference_time_ms
string backend
Detection2D[] detections
