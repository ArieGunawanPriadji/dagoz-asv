#include "asv_mavros_bridge/mavros_bridge_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dagozilla
{

MavrosBridgeNode::MavrosBridgeNode()
: rclcpp::Node("mavros_bridge_node")
{
  auto qos = rclcpp::SensorDataQoS();

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    std::bind(&MavrosBridgeNode::stateCallback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", qos,
    std::bind(&MavrosBridgeNode::gpsCallback, this, std::placeholders::_1));

  mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

  set_mode_srv_ = this->create_service<asv_msgs::srv::SetMode>(
    "/asv/set_mode",
    std::bind(&MavrosBridgeNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "mavros_bridge_node siap.");
}

void MavrosBridgeNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  current_state_ = *msg;
}

void MavrosBridgeNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  current_gps_ = *msg;
}

void MavrosBridgeNode::onSetMode(
  const std::shared_ptr<asv_msgs::srv::SetMode::Request> request,
  std::shared_ptr<asv_msgs::srv::SetMode::Response> response)
{
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
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dagozilla::MavrosBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
