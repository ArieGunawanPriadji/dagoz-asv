#include "asv_perception_vision/vision_detector_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace dagozilla
{

VisionDetectorNode::VisionDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vision_detector_node", options)
{
  // Default: threshold warna merah (contoh buoy merah). Ubah via ros2 param.
  h_low_ = this->declare_parameter<int>("h_low", 0);
  h_high_ = this->declare_parameter<int>("h_high", 10);
  s_low_ = this->declare_parameter<int>("s_low", 120);
  s_high_ = this->declare_parameter<int>("s_high", 255);
  v_low_ = this->declare_parameter<int>("v_low", 70);
  v_high_ = this->declare_parameter<int>("v_high", 255);
  min_contour_area_ = this->declare_parameter<double>("min_contour_area", 300.0);

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&VisionDetectorNode::imageCallback, this, std::placeholders::_1));

  obstacle_pub_ = this->create_publisher<asv_msgs::msg::ObstacleArray>(
    "obstacles", 10);

  RCLCPP_INFO(this->get_logger(), "vision_detector_node (dagozilla) siap.");
}

void VisionDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  auto obstacles = detect(cv_ptr->image, msg->header);
  obstacle_pub_->publish(obstacles);
}

asv_msgs::msg::ObstacleArray VisionDetectorNode::detect(
  const cv::Mat & frame, const std_msgs::msg::Header & header)
{
  asv_msgs::msg::ObstacleArray result;
  result.header = header;

  cv::Mat hsv, mask;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(
    hsv,
    cv::Scalar(h_low_, s_low_, v_low_),
    cv::Scalar(h_high_, s_high_, v_high_),
    mask);

  // Bersihkan noise
  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & c : contours) {
    double area = cv::contourArea(c);
    if (area < min_contour_area_) {
      continue;
    }
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(c, center, radius);

    // TODO: konversi posisi pixel -> posisi relatif dunia nyata butuh
    // kalibrasi kamera (intrinsics) + asumsi jarak/ukuran objek, atau
    // fusion dengan data depth/lidar kalau ada. Untuk sekarang publish
    // posisi pixel sebagai placeholder di frame kamera.
    geometry_msgs::msg::Point p;
    p.x = center.x;
    p.y = center.y;
    p.z = 0.0;

    result.positions.push_back(p);
    result.radii.push_back(radius);
  }

  return result;
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::VisionDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
