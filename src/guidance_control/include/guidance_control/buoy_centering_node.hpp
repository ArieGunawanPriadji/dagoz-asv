#ifndef GUIDANCE_CONTROL__BUOY_CENTERING_NODE_HPP_
#define GUIDANCE_CONTROL__BUOY_CENTERING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <msgs/msg/obstacle_array.hpp>
#include <msgs/msg/mission_status.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace dagozilla
{

/// Node guidance_control untuk mencari titik tengah (centering) 2 buoy (Merah & Hijau).
/// Menerapkan aturan ketat: jika kurang dari 2 buoy terdeteksi (0 atau hanya 1), ASV LANGSUNG STOP (v=0, w=0).
/// Menampilkan live visualizer stream dari kamera beserta visualisasi deteksi bola & status kendali.
class BuoyCenteringNode : public rclcpp::Node
{
public:
  BuoyCenteringNode();
  ~BuoyCenteringNode() override;

private:
  void obstacleCallback(const msgs::msg::ObstacleArray::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void publishCmd(double linear_x, double angular_z);
  void renderOverlay(
    cv::Mat & frame,
    const msgs::msg::ObstacleArray::SharedPtr & obs_msg,
    bool is_active,
    double best_red_x, double best_green_x,
    double midpoint_x, double error_x,
    double cmd_linear, double cmd_angular);

  double heading_kp_{1.5};
  double max_linear_speed_{1.0};
  double max_yaw_rate_{1.0};
  double frame_width_{640.0};
  double frame_height_{480.0};
  double speed_reduction_factor_{0.6};
  bool show_gui_{true};

  cv::Mat latest_frame_;
  std_msgs::msg::Header latest_header_;
  bool has_frame_{false};

  rclcpp::Subscription<msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
};

}  // namespace dagozilla

#endif  // GUIDANCE_CONTROL__BUOY_CENTERING_NODE_HPP_
