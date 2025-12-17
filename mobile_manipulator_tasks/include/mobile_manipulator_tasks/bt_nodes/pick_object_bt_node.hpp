#pragma once

#include <string>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <control_msgs/action/gripper_command.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>

namespace mobile_manipulator_tasks
{

class PickObjectNode : public BT::SyncActionNode
{
public:
  PickObjectNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  ~PickObjectNode();

private:
  bool closeGripper(double width_m, double effort);

  static constexpr const char* kArmGroup = "panda_arm";
  static constexpr const char* kEeLink   = "panda_hand";

  // These must match your robot frames in TF/URDF
  static constexpr const char* kBaseLink = "base_link";
  static constexpr const char* kArmBase  = "panda_link0";  
//   static constexpr const char* kArmBase  = "base_link";

  // Gripper action name (common for GripperCommandController)
  static constexpr const char* kGripperAction = "/panda_hand_controller/gripper_cmd";

  // Motion parameters
  static constexpr double kPreZ   = 0.15;  // meters above object
  static constexpr double kGraspZ = 0.02;  // meters above object when closing
  static constexpr double kLiftZ  = 0.20;  // meters above object after lift

  static constexpr double kVelScale = 0.3;
  static constexpr double kAccScale = 0.3;
  static constexpr double kPlanTime = 5.0;

  // Gripper widths 
  static constexpr double kCloseWidth = 0.0;   // fully closed
  static constexpr double kCloseEffort = 20.0; // N-ish (depends on controller)

  // ROS/MoveIt handles
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_client_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread spin_thread_;  
};

} 
