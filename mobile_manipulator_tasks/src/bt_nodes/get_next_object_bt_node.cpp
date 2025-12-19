#include "mobile_manipulator_tasks/bt_nodes/get_next_object_bt_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
using namespace std::chrono_literals;


namespace mobile_manipulator_tasks
{

GetNextObjectBTNode::GetNextObjectBTNode(const std::string& name, const BT::NodeConfiguration& config): SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("get_next_object_bt_node");
  client_ = node_->create_client<mobile_manipulator_tasks::srv::GetNextObject>("get_next_object");
  while (!client_->wait_for_service(std::chrono::seconds(1))) 
  {
    RCLCPP_WARN(node_->get_logger(), "Waiting for /get_next_object service...");
  }
}

BT::PortsList GetNextObjectBTNode::providedPorts()
{
  return {
      BT::OutputPort<std::string>("target_name"),
      BT::OutputPort<geometry_msgs::msg::Pose>("target_pose"),
      BT::OutputPort<std::string>("target_color"),
  };
}


BT::NodeStatus GetNextObjectBTNode::tick()
{
  if (!rclcpp::ok()) {
    return BT::NodeStatus::FAILURE;
  }

  // 1) Make sure the service is available
  if (!client_->wait_for_service(500ms)) {
    RCLCPP_WARN(node_->get_logger(), "Service /get_next_object not available");
    return BT::NodeStatus::FAILURE;
  }

  // 2) Prepare request
  auto request =
      std::make_shared<mobile_manipulator_tasks::srv::GetNextObject::Request>();

  // 3) Send request asynchronously
  auto future = client_->async_send_request(request);

  // 4) Spin this node until the future completes (or timeout)
  auto result = rclcpp::spin_until_future_complete(
      node_, future, 2s);

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Service call to /get_next_object failed or timed out");
    return BT::NodeStatus::FAILURE;
  }

  auto response = future.get();

  // 5) No new objects case
  if (!response->success) {
    // No unserved spawned objects available
    return BT::NodeStatus::FAILURE;
  }

  // 6) Write values to blackboard
  setOutput("target_name",  response->object.name);
  setOutput("target_pose",  response->object.pose);
  setOutput("target_color", response->object.color);

  return BT::NodeStatus::SUCCESS;
}










}  // namespace mobile_manipulator_tasks

// Register with BT factory
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::GetNextObjectBTNode>(
      "GetNextObject");
}
