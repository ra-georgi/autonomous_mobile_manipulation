#include "mobile_manipulator_tasks/bt_nodes/get_next_object_bt_node.hpp"
#include <behaviortree_cpp/bt_factory.h>

namespace mobile_manipulator_tasks
{

GetNextObjectBTNode::GetNextObjectBTNode(const std::string& name, const BT::NodeConfiguration& config): SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("get_next_object_bt_node");

  client_ = node_->create_client<mobile_manipulator_tasks::srv::GetNextObject>("get_next_object");

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for /get_next_object service...");
  }
}

BT::PortsList GetNextObjectBTNode::providedPorts()
{
  return {
      BT::OutputPort<std::string>("target_name"),
      BT::OutputPort<geometry_msgs::msg::Pose>("target_pose")
  };
}

BT::NodeStatus GetNextObjectBTNode::tick()
{
  auto req = std::make_shared<mobile_manipulator_tasks::srv::GetNextObject::Request>();

  auto future = client_->async_send_request(req);

  // synchronous wait
  auto status = future.wait_for(std::chrono::milliseconds(500));

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Service call timeout");
    return BT::NodeStatus::FAILURE;
  }

  auto resp = future.get();

  if (!resp->success) {
    // no new objects available
    return BT::NodeStatus::FAILURE;
  }

  // write to BT blackboard
  setOutput("target_name", resp->object.name);
  setOutput("target_pose", resp->object.pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mobile_manipulator_tasks

// Register with BT factory
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::GetNextObjectBTNode>(
      "GetNextObject");
}
