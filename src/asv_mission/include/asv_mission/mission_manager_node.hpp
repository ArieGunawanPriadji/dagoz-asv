#ifndef ASV_MISSION__MISSION_MANAGER_NODE_HPP_
#define ASV_MISSION__MISSION_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <asv_msgs/msg/waypoint.hpp>
#include <asv_msgs/msg/mission_status.hpp>
#include <vector>

namespace dagozilla
{

/// State machine level tinggi untuk misi ASV (task "Navigation Channel",
/// "Obstacle Avoidance", "Docking", dst sesuai rulebook KKI).
/// Alur: IDLE -> (start_mission) -> RUNNING -> COMPLETED / ABORTED
class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode();

  void loadWaypoints(const std::vector<asv_msgs::msg::Waypoint> & waypoints);

private:
  void onStart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onAbort(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publishActiveWaypoint();
  void publishStatus();

  std::vector<asv_msgs::msg::Waypoint> waypoints_;
  std::size_t current_index_{0};
  uint8_t state_{asv_msgs::msg::MissionStatus::IDLE};

  rclcpp::Publisher<asv_msgs::msg::Waypoint>::SharedPtr wp_pub_;
  rclcpp::Publisher<asv_msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace dagozilla

#endif  // ASV_MISSION__MISSION_MANAGER_NODE_HPP_
