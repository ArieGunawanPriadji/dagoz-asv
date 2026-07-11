#include "guidance_control/waypoint_follower_node.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace dagozilla
{

void WaypointFollowerNode::bearingDistance(
  double lat1, double lon1, double lat2, double lon2,
  double & bearing_deg, double & distance_m)
{
  constexpr double R = 6371000.0;
  double phi1 = lat1 * M_PI / 180.0;
  double phi2 = lat2 * M_PI / 180.0;
  double dphi = (lat2 - lat1) * M_PI / 180.0;
  double dlambda = (lon2 - lon1) * M_PI / 180.0;

  double a = std::sin(dphi / 2) * std::sin(dphi / 2) +
    std::cos(phi1) * std::cos(phi2) * std::sin(dlambda / 2) * std::sin(dlambda / 2);
  distance_m = 2 * R * std::asin(std::sqrt(a));

  double y = std::sin(dlambda) * std::cos(phi2);
  double x = std::cos(phi1) * std::sin(phi2) - std::sin(phi1) * std::cos(phi2) * std::cos(dlambda);
  bearing_deg = std::fmod((std::atan2(y, x) * 180.0 / M_PI) + 360.0, 360.0);
}

WaypointFollowerNode::WaypointFollowerNode()
: rclcpp::Node("waypoint_follower_node")
{
  this->declare_parameter<double>("max_linear_speed", 1.0);  // m/s
  this->declare_parameter<double>("heading_kp", 1.5);

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", rclcpp::SensorDataQoS(),
    std::bind(&WaypointFollowerNode::gpsCallback, this, std::placeholders::_1));

  waypoint_sub_ = this->create_subscription<msgs::msg::Waypoint>(
    "/asv/active_waypoint", 10,
    std::bind(&WaypointFollowerNode::waypointCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
  status_pub_ = this->create_publisher<msgs::msg::MissionStatus>(
    "/asv/mission_status", 10);

  timer_ = this->create_wall_timer(200ms, std::bind(&WaypointFollowerNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "waypoint_follower_node siap.");
}

void WaypointFollowerNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  current_gps_ = *msg;
}

void WaypointFollowerNode::waypointCallback(const msgs::msg::Waypoint::SharedPtr msg)
{
  active_waypoint_ = *msg;
}

void WaypointFollowerNode::controlLoop()
{
  if (!active_waypoint_.has_value()) {
    return;
  }
  if (current_gps_.latitude == 0.0) {
    return;  // belum ada fix GPS
  }

  double bearing, distance;
  bearingDistance(
    current_gps_.latitude, current_gps_.longitude,
    active_waypoint_->latitude, active_waypoint_->longitude,
    bearing, distance);

  if (distance <= active_waypoint_->acceptance_radius) {
    RCLCPP_INFO(this->get_logger(), "Waypoint tercapai.");
    active_waypoint_.reset();
    publishCmd(0.0, 0.0);
    return;
  }

  double max_speed = this->get_parameter("max_linear_speed").as_double();
  double speed = std::min(static_cast<double>(active_waypoint_->target_speed), max_speed);

  // NOTE: bagian ini butuh heading kapal aktual (dari IMU/compass) untuk
  // menghitung heading error yang benar. Skeleton ini disederhanakan —
  // isi dengan PID heading / pure pursuit sesuai kebutuhan tim.
  publishCmd(speed, 0.0);
}

void WaypointFollowerNode::publishCmd(double linear_x, double angular_z)
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
  rclcpp::spin(std::make_shared<dagozilla::WaypointFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
