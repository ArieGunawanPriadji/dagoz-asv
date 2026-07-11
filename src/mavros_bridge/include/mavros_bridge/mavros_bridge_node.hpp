#ifndef MAVROS_BRIDGE__MAVROS_BRIDGE_NODE_HPP_
#define MAVROS_BRIDGE__MAVROS_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <msgs/srv/set_mode.hpp>

namespace dagozilla
{

/// Jembatan antara stack ROS2 (mission/guidance) dengan Pixhawk (ArduRover)
/// lewat MAVROS: baca state & GPS, sediakan service ganti flight mode.
class MavrosBridgeNode : public rclcpp::Node
{
public:
  MavrosBridgeNode();

private:
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void onSetMode(
    const std::shared_ptr<msgs::srv::SetMode::Request> request,
    std::shared_ptr<msgs::srv::SetMode::Response> response);

  mavros_msgs::msg::State current_state_;
  sensor_msgs::msg::NavSatFix current_gps_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Service<msgs::srv::SetMode>::SharedPtr set_mode_srv_;
};

}  // namespace dagozilla

#endif  // MAVROS_BRIDGE__MAVROS_BRIDGE_NODE_HPP_
