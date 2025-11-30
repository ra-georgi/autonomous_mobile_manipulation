#include "mobile_manipulator_tasks/bt_nodes/navigate_to_pose_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>

using namespace std::chrono_literals;

namespace mobile_manipulator_tasks
{

NavigateToPoseBT::NavigateToPoseBT(const std::string& name,const BT::NodeConfiguration& config): BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("navigate_to_pose_bt_node");
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList NavigateToPoseBT::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")  // Input: PoseStamped nav goal (from ComputeBaseGoalFromObject)
  };
}

BT::NodeStatus NavigateToPoseBT::tick()
{
  if (!rclcpp::ok()) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!getInput("goal", goal_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseBT: missing required input [goal]");
    return BT::NodeStatus::FAILURE;
  }

  // 1) Wait for Nav2 action server
  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseBT: navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  // 2) Build goal
  NavigateToPose::Goal goal;
  goal.pose = goal_pose;
  goal.behavior_tree = "";  // use Nav2's default internal BT

  RCLCPP_INFO(node_->get_logger(), "NavigateToPoseBT: sending goal to (%.2f, %.2f, %.2f)", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);

  // 3) Send goal (async)
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions{};
  send_goal_options.result_callback = nullptr;  // we will block on result

  auto goal_handle_future = client_->async_send_goal(goal, send_goal_options);

  // 4) Wait for goal to be accepted
  auto gh_status = rclcpp::spin_until_future_complete(node_, goal_handle_future, 5s);

  if (gh_status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseBT: failed to send goal");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseBT: goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  // 5) Wait for result (this blocks while robot navigates)
  auto result_future = client_->async_get_result(goal_handle);

  RCLCPP_INFO(node_->get_logger(), "NavigateToPoseBT: waiting for navigation result...");

  auto result_status = rclcpp::spin_until_future_complete(node_, result_future, 180s);  // up to 3 min

  if (result_status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseBT: get_result timed out or failed");
    return BT::NodeStatus::FAILURE;
  }

  auto wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "NavigateToPoseBT: navigation SUCCEEDED");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node_->get_logger(), "NavigateToPoseBT: navigation ABORTED");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "NavigateToPoseBT: navigation CANCELED");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseBT: unknown result code");
      return BT::NodeStatus::FAILURE;
  }
}

}  

// Register as BT plugin
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::NavigateToPoseBT>(
      "NavigateToPoseBT");
}
