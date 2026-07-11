#ifndef ASV_PERCEPTION_VISION__VISION_DETECTOR_NODE_HPP_
#define ASV_PERCEPTION_VISION__VISION_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <asv_msgs/msg/obstacle_array.hpp>

namespace dagozilla
{

/// Node deteksi obstacle/buoy dari kamera menggunakan color thresholding.
/// Ganti bagian detect() dengan pipeline yang lebih canggih (YOLO/DNN) kalau perlu.
class VisionDetectorNode : public rclcpp::Node
{
public:
  explicit VisionDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  asv_msgs::msg::ObstacleArray detect(const cv::Mat & frame, const std_msgs::msg::Header & header);

  // Parameter warna HSV untuk buoy (contoh: merah). Sesuaikan lewat ROS params.
  int h_low_, h_high_, s_low_, s_high_, v_low_, v_high_;
  double min_contour_area_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<asv_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
};

}  // namespace dagozilla

#endif  // ASV_PERCEPTION_VISION__VISION_DETECTOR_NODE_HPP_
