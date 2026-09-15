#ifndef AUTOWARE_CORE_MESSAGE_BRIDGE__MESSAGE_BRIDGE_NODE_HPP_
#define AUTOWARE_CORE_MESSAGE_BRIDGE__MESSAGE_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

// TIER IV messages
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_velocity_limit.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>

// AWF messages
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>

namespace autoware_core_message_bridge
{

class MessageBridgeNode : public rclcpp::Node
{
public:
  explicit MessageBridgeNode(const rclcpp::NodeOptions & options);

private:
  // PathWithLaneId bridge (AWF -> TIER IV)
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr sub_awf_path_;
  rclcpp::Publisher<tier4_planning_msgs::msg::PathWithLaneId>::SharedPtr pub_t4_path_;
  void on_path(const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg);

  // Engage bridge (TIER IV srv -> AWF srv)
  rclcpp::Service<tier4_external_api_msgs::srv::Engage>::SharedPtr srv_engage_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr cli_operation_mode_;
  void on_engage(
    const std::shared_ptr<tier4_external_api_msgs::srv::Engage::Request> request,
    std::shared_ptr<tier4_external_api_msgs::srv::Engage::Response> response);

  // Velocity Limit bridge (TIER IV srv -> AWF topic)
  rclcpp::Service<tier4_external_api_msgs::srv::SetVelocityLimit>::SharedPtr srv_vel_limit_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr pub_vel_limit_;
  void on_set_velocity_limit(
    const std::shared_ptr<tier4_external_api_msgs::srv::SetVelocityLimit::Request> request,
    std::shared_ptr<tier4_external_api_msgs::srv::SetVelocityLimit::Response> response);
};

}  // namespace autoware_core_message_bridge

#endif  // AUTOWARE_CORE_MESSAGE_BRIDGE__MESSAGE_BRIDGE_NODE_HPP_
