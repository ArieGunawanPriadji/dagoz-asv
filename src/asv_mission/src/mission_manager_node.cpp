#include "asv_mission/mission_manager_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dagozilla
{

MissionManagerNode::MissionManagerNode()
: rclcpp::Node("mission_manager_node")
{
  wp_pub_ = this->create_publisher<asv_msgs::msg::Waypoint>("/asv/active_waypoint", 10);
  status_pub_ = this->create_publisher<asv_msgs::msg::MissionStatus>("/asv/mission_status", 10);

  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/start_mission",
    std::bind(&MissionManagerNode::onStart, this, std::placeholders::_1, std::placeholders::_2));

  abort_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/abort_mission",
    std::bind(&MissionManagerNode::onAbort, this, std::placeholders::_1, std::placeholders::_2));

  status_timer_ = this->create_wall_timer(1s, std::bind(&MissionManagerNode::publishStatus, this));

  RCLCPP_INFO(
    this->get_logger(),
    "mission_manager_node siap. Isi waypoints lewat loadWaypoints() / mission file / GCS.");
}

void MissionManagerNode::loadWaypoints(const std::vector<asv_msgs::msg::Waypoint> & waypoints)
{
  waypoints_ = waypoints;
  current_index_ = 0;
}

void MissionManagerNode::onStart(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (waypoints_.empty()) {
    response->success = false;
    response->message = "Belum ada waypoint yang di-load.";
    return;
  }
  state_ = asv_msgs::msg::MissionStatus::RUNNING;
  publishActiveWaypoint();
  response->success = true;
  response->message = "Misi dimulai.";
}

void MissionManagerNode::onAbort(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  state_ = asv_msgs::msg::MissionStatus::ABORTED;
  response->success = true;
  response->message = "Misi dibatalkan.";
}

void MissionManagerNode::publishActiveWaypoint()
{
  if (current_index_ < waypoints_.size()) {
    wp_pub_->publish(waypoints_[current_index_]);
  } else {
    state_ = asv_msgs::msg::MissionStatus::COMPLETED;
  }
}

void MissionManagerNode::publishStatus()
{
  asv_msgs::msg::MissionStatus msg;
  msg.state = state_;
  msg.current_waypoint_index = static_cast<uint32_t>(current_index_);
  msg.total_waypoints = static_cast<uint32_t>(waypoints_.size());
  status_pub_->publish(msg);
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dagozilla::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
