#include "mobile_manipulator_tasks/bt_nodes/compute_base_goal_from_object.hpp"
#include <behaviortree_cpp/bt_factory.h>

using namespace std::chrono_literals;

namespace mobile_manipulator_tasks
{

ComputeBaseGoalFromObject::ComputeBaseGoalFromObject(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config)
{
  // Node only used for getting time / logging
  node_ = rclcpp::Node::make_shared("compute_base_goal_from_object");
}

BT::PortsList ComputeBaseGoalFromObject::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Pose>("object_pose"),     // Input: object pose (from GetNextObject)
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goal") // Output: PoseStamped for Nav2 NavigateToPose
  };
}

BT::NodeStatus ComputeBaseGoalFromObject::tick()
{
  // 1) Read object_pose from blackboard
  geometry_msgs::msg::Pose object_pose;
  if (!getInput("object_pose", object_pose))
  {
    RCLCPP_ERROR(node_->get_logger(), "ComputeBaseGoalFromObject: missing required input [object_pose]");
    return BT::NodeStatus::FAILURE;
  }

  // 2) Build PoseStamped goal
  geometry_msgs::msg::PoseStamped goal;

  goal.header.frame_id = "map";

  // Use current time if available
  if (rclcpp::ok())
  {
    goal.header.stamp = node_->now();
  }

  // Position: same x,y as object, z = 0 (ground)
  goal.pose.position.x = (object_pose.position.x) - 0.6;
  goal.pose.position.y = object_pose.position.y;
  goal.pose.position.z = 0.0;

  // Orientation: identity quaternion (yaw = 0)
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = 0.0;
  goal.pose.orientation.w = 1.0;

  // 3) Write to blackboard
  setOutput("nav_goal", goal);

  RCLCPP_DEBUG(node_->get_logger(), "ComputeBaseGoalFromObject: nav_goal = (%.3f, %.3f, %.3f)", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mobile_manipulator_tasks

// Register as BT plugin
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::ComputeBaseGoalFromObject>("ComputeBaseGoalFromObject");
}
