#include "mavros_bridge/mavros_bridge_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dagozilla
{

MavrosBridgeNode::MavrosBridgeNode()
: rclcpp::Node("mavros_bridge_node")
{
  auto qos = rclcpp::SensorDataQoS();
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

#ifdef HAVE_MAVROS_MSGS
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    std::bind(&MavrosBridgeNode::stateCallback, this, std::placeholders::_1));

  mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
    "/mavros/set_mode", rmw_qos_profile_services_default, cb_group_);
#endif

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", qos,
    std::bind(&MavrosBridgeNode::gpsCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/asv/cmd_vel", 10,
    std::bind(&MavrosBridgeNode::cmdVelCallback, this, std::placeholders::_1));

  mavros_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

  set_mode_srv_ = this->create_service<msgs::srv::SetMode>(
    "/asv/set_mode",
    std::bind(&MavrosBridgeNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), cb_group_);

  RCLCPP_INFO(this->get_logger(), "mavros_bridge_node siap & forwarding /asv/cmd_vel ke MAVROS.");
}

#ifdef HAVE_MAVROS_MSGS
void MavrosBridgeNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  current_state_ = *msg;
}
#endif

void MavrosBridgeNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  current_gps_ = *msg;
}

void MavrosBridgeNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Forward geometry_msgs/Twist velocity command to MAVROS
  mavros_cmd_vel_pub_->publish(*msg);
}

void MavrosBridgeNode::onSetMode(
  const std::shared_ptr<msgs::srv::SetMode::Request> request,
  std::shared_ptr<msgs::srv::SetMode::Response> response)
{
#ifdef HAVE_MAVROS_MSGS
  if (!mode_client_->wait_for_service(2s)) {
    response->success = false;
    response->message = "mavros/set_mode service tidak tersedia";
    return;
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = request->mode;  // contoh: "GUIDED", "AUTO", "HOLD", "RTL"

  auto future = mode_client_->async_send_request(req);
  if (future.wait_for(3s) != std::future_status::ready) {
    response->success = false;
    response->message = "Timeout menunggu respons Pixhawk";
    return;
  }

  auto result = future.get();
  if (result->mode_sent) {
    response->success = true;
    response->message = "Mode diubah ke " + request->mode;
  } else {
    response->success = false;
    response->message = "Gagal mengubah mode di Pixhawk";
  }
#else
  response->success = true;
  response->message = "Mode " + request->mode + " diset (mock mode tanpa mavros_msgs)";
#endif
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::MavrosBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
