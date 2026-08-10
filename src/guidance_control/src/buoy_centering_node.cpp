#include "guidance_control/buoy_centering_node.hpp"
#include <algorithm>
#include <cmath>

namespace dagozilla
{

BuoyCenteringNode::BuoyCenteringNode()
: rclcpp::Node("buoy_centering_node")
{
  this->declare_parameter<double>("heading_kp", 1.5);
  this->declare_parameter<double>("max_linear_speed", 0.8);  // m/s
  this->declare_parameter<double>("max_yaw_rate", 1.0);     // rad/s
  this->declare_parameter<double>("frame_width", 640.0);
  this->declare_parameter<double>("speed_reduction_factor", 0.6);
  this->declare_parameter<bool>("show_gui", true);

  heading_kp_ = this->get_parameter("heading_kp").as_double();
  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
  frame_width_ = this->get_parameter("frame_width").as_double();
  speed_reduction_factor_ = this->get_parameter("speed_reduction_factor").as_double();
  show_gui_ = this->get_parameter("show_gui").as_bool();

  obstacle_sub_ = this->create_subscription<msgs::msg::ObstacleArray>(
    "/asv/obstacles", 10,
    std::bind(&BuoyCenteringNode::obstacleCallback, this, std::placeholders::_1));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    std::bind(&BuoyCenteringNode::imageCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/asv/cmd_vel", 10);
  status_pub_ = this->create_publisher<msgs::msg::MissionStatus>("/asv/mission_status", 10);
  debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/asv/guidance_debug", 10);

  if (show_gui_) {
    try {
      cv::namedWindow("ASV Guidance & Buoy Centering Live Stream", cv::WINDOW_AUTOSIZE);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "OpenCV GUI Window not available: %s", e.what());
      show_gui_ = false;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "buoy_centering_node aktif. Kamera visualizer & deteksi bola ditampilkan.");
}

BuoyCenteringNode::~BuoyCenteringNode()
{
  if (show_gui_) {
    cv::destroyAllWindows();
  }
}

void BuoyCenteringNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    latest_frame_ = cv_ptr->image.clone();
    latest_header_ = msg->header;
    has_frame_ = true;

    if (msg->width > 0) {
      frame_width_ = static_cast<double>(msg->width);
    }
    if (msg->height > 0) {
      frame_height_ = static_cast<double>(msg->height);
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void BuoyCenteringNode::obstacleCallback(const msgs::msg::ObstacleArray::SharedPtr msg)
{
  double best_red_x = -1.0;
  double best_red_y = -1.0;
  float max_red_radius = -1.0f;

  double best_green_x = -1.0;
  double best_green_y = -1.0;
  float max_green_radius = -1.0f;

  size_t red_count = 0;
  size_t green_count = 0;

  for (size_t i = 0; i < msg->positions.size(); ++i) {
    uint8_t cls = msg->classes[i];
    double x = msg->positions[i].x;  // Pixel coordinate u
    double y = msg->positions[i].y;  // Pixel coordinate v
    float r = msg->radii[i];

    if (cls == msgs::msg::ObstacleArray::RED_BUOY) {
      red_count++;
      if (r > max_red_radius) {
        max_red_radius = r;
        best_red_x = x;
        best_red_y = y;
      }
    } else if (cls == msgs::msg::ObstacleArray::GREEN_BUOY) {
      green_count++;
      if (r > max_green_radius) {
        max_green_radius = r;
        best_green_x = x;
        best_green_y = y;
      }
    }
  }

  bool is_active = (best_red_x >= 0 && best_green_x >= 0);

  double linear_x = 0.0;
  double angular_z = 0.0;
  double midpoint_x = -1.0;
  double error_x = 0.0;

  if (is_active) {
    midpoint_x = (best_red_x + best_green_x) / 2.0;
    double center_x = frame_width_ / 2.0;
    error_x = (midpoint_x - center_x) / center_x;

    angular_z = -heading_kp_ * error_x;
    angular_z = std::clamp(angular_z, -max_yaw_rate_, max_yaw_rate_);

    double speed_factor = std::max(0.2, 1.0 - std::abs(error_x) * speed_reduction_factor_);
    linear_x = max_linear_speed_ * speed_factor;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "CENTERING | Red_X: %.1f | Green_X: %.1f | Midpoint: %.1f | Error: %.2f | Cmd -> Speed: %.2f m/s, Yaw: %.2f rad/s",
      best_red_x, best_green_x, midpoint_x, error_x, linear_x, angular_z);

    publishCmd(linear_x, angular_z);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "STOP: Kurang dari 2 buoy terdeteksi (Merah: %zu, Hijau: %zu). Motor MATI.",
      red_count, green_count);

    publishCmd(0.0, 0.0);
  }

  // Render camera display overlay with ball detections & guidance status
  renderOverlay(latest_frame_, msg, is_active, best_red_x, best_green_x, midpoint_x, error_x, linear_x, angular_z);
}

void BuoyCenteringNode::renderOverlay(
  cv::Mat & frame,
  const msgs::msg::ObstacleArray::SharedPtr & obs_msg,
  bool is_active,
  double best_red_x, double best_green_x,
  double midpoint_x, double error_x,
  double cmd_linear, double cmd_angular)
{
  cv::Mat display_frame;
  if (has_frame_ && !frame.empty()) {
    display_frame = frame.clone();
  } else {
    display_frame = cv::Mat(static_cast<int>(frame_height_), static_cast<int>(frame_width_), CV_8UC3, cv::Scalar(20, 20, 20));
    cv::putText(display_frame, "NO CAMERA STREAM RECEIVED", cv::Point(150, 240),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  }

  int w = display_frame.cols;
  int h = display_frame.rows;
  int center_x = w / 2;

  // 1. Draw frame vertical centerline
  cv::line(display_frame, cv::Point(center_x, 40), cv::Point(center_x, h - 30), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  // 2. Draw all detected balls/buoys from obstacle array
  for (size_t i = 0; i < obs_msg->positions.size(); ++i) {
    cv::Point2f pt(obs_msg->positions[i].x, obs_msg->positions[i].y);
    float radius = obs_msg->radii[i];
    uint8_t cls = obs_msg->classes[i];

    cv::Scalar circle_color(255, 255, 255);
    std::string tag = "OBJECT";

    if (cls == msgs::msg::ObstacleArray::RED_BUOY) {
      circle_color = cv::Scalar(0, 0, 255);
      tag = "RED BUOY";
    } else if (cls == msgs::msg::ObstacleArray::GREEN_BUOY) {
      circle_color = cv::Scalar(0, 255, 0);
      tag = "GREEN BUOY";
    } else if (cls == msgs::msg::ObstacleArray::BLUE_DOCKING_BUOY) {
      circle_color = cv::Scalar(255, 0, 0);
      tag = "BLUE DOCK";
    }

    cv::circle(display_frame, pt, static_cast<int>(radius), circle_color, 2, cv::LINE_AA);
    cv::circle(display_frame, pt, 4, circle_color, -1);
    cv::putText(display_frame, tag, cv::Point(pt.x - 25, pt.y - radius - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, circle_color, 2);
  }

  // 3. Draw Centering vector lines & Midpoint if active
  if (is_active) {
    // Find y coordinates of red and green buoys
    double red_y = h / 2.0;
    double green_y = h / 2.0;
    for (size_t i = 0; i < obs_msg->positions.size(); ++i) {
      if (obs_msg->classes[i] == msgs::msg::ObstacleArray::RED_BUOY && std::abs(obs_msg->positions[i].x - best_red_x) < 2.0) {
        red_y = obs_msg->positions[i].y;
      }
      if (obs_msg->classes[i] == msgs::msg::ObstacleArray::GREEN_BUOY && std::abs(obs_msg->positions[i].x - best_green_x) < 2.0) {
        green_y = obs_msg->positions[i].y;
      }
    }

    cv::Point red_pt(static_cast<int>(best_red_x), static_cast<int>(red_y));
    cv::Point green_pt(static_cast<int>(best_green_x), static_cast<int>(green_y));
    cv::Point mid_pt(static_cast<int>(midpoint_x), static_cast<int>((red_y + green_y) / 2.0));

    // Line connecting Red and Green Buoys
    cv::line(display_frame, red_pt, green_pt, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

    // Midpoint marker
    cv::drawMarker(display_frame, mid_pt, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);

    // Line from frame center to midpoint
    cv::line(display_frame, cv::Point(center_x, mid_pt.y), mid_pt, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    // Steering Indicator Bar at bottom
    int bar_y = h - 20;
    int error_bar_len = static_cast<int>(error_x * (center_x - 40));
    cv::line(display_frame, cv::Point(center_x, bar_y), cv::Point(center_x + error_bar_len, bar_y), cv::Scalar(0, 255, 255), 4);
    cv::circle(display_frame, cv::Point(center_x + error_bar_len, bar_y), 6, cv::Scalar(0, 255, 255), -1);
  }

  // 4. Render Top OSD Status Banner
  cv::Rect banner_rect(0, 0, w, 35);
  cv::Scalar banner_bg = is_active ? cv::Scalar(0, 160, 0) : cv::Scalar(0, 0, 180);  // Green for active, Red for safety stop
  cv::rectangle(display_frame, banner_rect, banner_bg, -1);

  std::string status_str = is_active ? "[ STATUS: CENTERING ACTIVE ]" : "[ STATUS: SAFETY STOP - BUOY MISSING ]";
  cv::putText(display_frame, status_str, cv::Point(10, 24),
              cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);

  // 5. Render Command HUD Text at bottom left
  std::string cmd_str = cv::format("Cmd Vel -> Linear: %.2f m/s | Yaw: %.2f rad/s", cmd_linear, cmd_angular);
  cv::putText(display_frame, cmd_str, cv::Point(10, h - 35),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  // 6. Publish annotated debug image for ROS topic / GCS
  std_msgs::msg::Header header;
  if (has_frame_) {
    header = latest_header_;
  } else {
    header.stamp = this->get_clock()->now();
  }
  cv_bridge::CvImage debug_msg(header, "bgr8", display_frame);
  debug_img_pub_->publish(*debug_msg.toImageMsg());

  // 7. Show live OpenCV GUI window if show_gui parameter is true
  if (show_gui_) {
    cv::imshow("ASV Guidance & Buoy Centering Live Stream", display_frame);
    cv::waitKey(1);
  }
}

void BuoyCenteringNode::publishCmd(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  cmd_pub_->publish(msg);
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::BuoyCenteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
