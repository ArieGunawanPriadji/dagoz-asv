#ifndef ASV_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_
#define ASV_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <asv_msgs/msg/obstacle_array.hpp>

namespace dagozilla
{

/// Stub deteksi obstacle generik (mis. dari lidar). Isi dengan clustering
/// atau algoritma sensor lain di luar vision (lihat asv_perception_vision
/// untuk pipeline kamera).
class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

private:
  void detectLoop();
  rclcpp::Publisher<asv_msgs::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dagozilla

#endif  // ASV_PERCEPTION__OBSTACLE_DETECTOR_NODE_HPP_
