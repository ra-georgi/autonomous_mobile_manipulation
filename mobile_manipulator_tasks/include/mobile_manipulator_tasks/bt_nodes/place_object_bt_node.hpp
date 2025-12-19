#pragma once

#include <map>
#include <string>
#include <thread>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

namespace mobile_manipulator_tasks
{

class PlaceObjectNode : public BT::SyncActionNode
{
public:
  PlaceObjectNode(const std::string& name, const BT::NodeConfiguration& config);
  ~PlaceObjectNode();

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  bool openGripper(double width_m, double effort);

  static constexpr const char* kArmGroup = "panda_arm";
  static constexpr const char* kEeLink   = "panda_hand";
  static constexpr const char* kGripperAction = "/panda_hand_controller/gripper_cmd";

  static constexpr double kPlanTime = 5.0;
  static constexpr double kVelScale = 0.3;
  static constexpr double kAccScale = 0.3;

  static constexpr double kOpenWidth  = 0.03;  // Panda fingers open width (meters). Adjust if needed.
  static constexpr double kOpenEffort = 10.0;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread spin_thread_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;
};

}  
