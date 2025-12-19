#include "mobile_manipulator_tasks/bt_nodes/place_object_bt_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>

using namespace std::chrono_literals;

namespace mobile_manipulator_tasks
{

PlaceObjectNode::PlaceObjectNode(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config)
{
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  node_ = rclcpp::Node::make_shared("place_object_bt_node", opts);

  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  spin_thread_ = std::thread([this]() { exec_->spin(); });

  arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, kArmGroup);
  arm_->setEndEffectorLink(kEeLink);
  arm_->setPlanningTime(kPlanTime);
  arm_->setMaxVelocityScalingFactor(kVelScale);
  arm_->setMaxAccelerationScalingFactor(kAccScale);

  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node_, kGripperAction);

  RCLCPP_INFO(node_->get_logger(), "PlaceObjectNode initialized");
}

PlaceObjectNode::~PlaceObjectNode()
{
  if (exec_) exec_->cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}

BT::PortsList PlaceObjectNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_color")  // "red" or "green"
  };
}

bool PlaceObjectNode::openGripper(double width_m, double effort)
{
  if (!gripper_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available at %s", kGripperAction);
    return false;
  }

  control_msgs::action::GripperCommand::Goal goal;
  goal.command.position = width_m;
  goal.command.max_effort = effort;

  auto goal_handle_future = gripper_client_->async_send_goal(goal);
  if (goal_handle_future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper goal handle");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper goal rejected");
    return false;
  }

  auto result_future = gripper_client_->async_get_result(goal_handle);
  if (result_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper result");
    return false;
  }

  auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(node_->get_logger(), "Gripper open did not succeed");
    return false;
  }

  return true;
}

BT::NodeStatus PlaceObjectNode::tick()
{
  std::string color;
  if (!getInput("object_color", color)) {
    RCLCPP_ERROR(node_->get_logger(), "PlaceObjectNode: missing input [object_color]");
    return BT::NodeStatus::FAILURE;
  }

  const std::map<std::string, double> place_red = {
    {"panda_joint1",  0.2},
    {"panda_joint2", -0.9},
    {"panda_joint3",  0.0},
    {"panda_joint4", -2.0},
    {"panda_joint5",  0.0},
    {"panda_joint6",  1.4},
    {"panda_joint7",  0.8},
  };

  const std::map<std::string, double> place_green = {
    {"panda_joint1", -0.2},
    {"panda_joint2", -0.9},
    {"panda_joint3",  0.0},
    {"panda_joint4", -2.0},
    {"panda_joint5",  0.0},
    {"panda_joint6",  1.4},
    {"panda_joint7",  0.8},
  };

  const std::map<std::string, double>& target = (color == "red") ? place_red : place_green;

  RCLCPP_INFO(node_->get_logger(), "Placing object in %s box using joint target", color.c_str());

  // Move to place pose
  arm_->setJointValueTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed for place pose (%s)", color.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed for place pose (%s)", color.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Open gripper to drop
  if (!openGripper(kOpenWidth, kOpenEffort)) {
    RCLCPP_WARN(node_->get_logger(), "Gripper open failed; continuing (object may still drop)");
  }

  RCLCPP_INFO(node_->get_logger(), "PlaceObjectNode: SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mobile_manipulator_tasks

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::PlaceObjectNode>("PlaceObject");
}
