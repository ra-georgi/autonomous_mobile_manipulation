#include "mobile_manipulator_tasks/bt_nodes/pick_object_bt_node.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <map>

using namespace std::chrono_literals;

namespace mobile_manipulator_tasks
{

PickObjectNode::PickObjectNode(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config)
{
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});    
  node_ = rclcpp::Node::make_shared("pick_object_bt_node", opts);

  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  spin_thread_ = std::thread([this]() {
    exec_->spin();
  });   


  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // MoveIt Arm interface
  arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, kArmGroup);
  arm_->setEndEffectorLink(kEeLink);
  arm_->setPlanningTime(kPlanTime);
  arm_->setMaxVelocityScalingFactor(kVelScale);
  arm_->setMaxAccelerationScalingFactor(kAccScale);

  // Gripper action client
  gripper_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node_, kGripperAction);

  RCLCPP_INFO(node_->get_logger(), "PickObjectNode initialized. arm_group=%s ee_link=%s base_link=%s arm_base=%s gripper_action=%s",
              kArmGroup, kEeLink, kBaseLink, kArmBase, kGripperAction);
}

BT::PortsList PickObjectNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Pose>("object_pose"),
    BT::InputPort<geometry_msgs::msg::Pose>("base_pose")
  };
}

bool PickObjectNode::closeGripper(double width_m, double effort)
{
  using namespace std::chrono_literals;

  if (!gripper_client_) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper client is null");
    return false;
  }

  if (!gripper_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available at %s", kGripperAction);
    return false;
  }

  control_msgs::action::GripperCommand::Goal goal;
  goal.command.position = width_m;
  goal.command.max_effort = effort;

  // Send goal
  auto goal_handle_future = gripper_client_->async_send_goal(goal);

  if (goal_handle_future.wait_for(2s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper goal handle");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Gripper goal was rejected");
    return false;
  }

  // Wait for result
  auto result_future = gripper_client_->async_get_result(goal_handle);

  if (result_future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for gripper result");
    return false;
  }

  auto wrapped = result_future.get();

  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(node_->get_logger(),
                "Gripper action did not succeed (code=%d)",
                static_cast<int>(wrapped.code));
    return false;
  }

  return true;
}


BT::NodeStatus PickObjectNode::tick()
{
  if (!rclcpp::ok()) {
    return BT::NodeStatus::FAILURE;
  }

  // arm_->setNamedTarget("home");  
  // moveit::planning_interface::MoveGroupInterface::Plan home_plan;
  // if (arm_->plan(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) 
  // {
  //   RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed for HOME (named target)");
  //   return BT::NodeStatus::FAILURE;
  // }
  // if (arm_->execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
  //   RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed for HOME (named target)");
  //   return BT::NodeStatus::FAILURE;
  // }

std::map<std::string, double> home_joints;
home_joints["panda_joint1"] = 0.0;
home_joints["panda_joint2"] = 0.0;
home_joints["panda_joint3"] = 0.0;
home_joints["panda_joint4"] = -1.571;
home_joints["panda_joint5"] = 0.0;
home_joints["panda_joint6"] = 1.571;
home_joints["panda_joint7"] = 0.785398;

arm_->setJointValueTarget(home_joints);

moveit::planning_interface::MoveGroupInterface::Plan home_plan;
if (arm_->plan(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
  RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed for HOME (joint target)");
  return BT::NodeStatus::FAILURE;
}
if (arm_->execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
  RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed for HOME (joint target)");
  return BT::NodeStatus::FAILURE;
}



  geometry_msgs::msg::Pose object_pose_w;
  geometry_msgs::msg::Pose base_pose_w;

  if (!getInput("object_pose", object_pose_w)) {
    RCLCPP_ERROR(node_->get_logger(), "PickObjectNode: missing input [object_pose]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("base_pose", base_pose_w)) {
    RCLCPP_ERROR(node_->get_logger(), "PickObjectNode: missing input [base_pose]");
    return BT::NodeStatus::FAILURE;
  }

  // Get fixed mount transform base_link -> panda_link0 from TF 
  geometry_msgs::msg::TransformStamped T_base_arm;
  try 
  {
    T_base_arm = tf_buffer_->lookupTransform(kBaseLink, kArmBase, tf2::TimePointZero);   // TimePointZero = latest available
    RCLCPP_INFO(
      node_->get_logger(),
      "T_base_arm:\n"
      "  translation: [x=%.3f, y=%.3f, z=%.3f]\n"
      "  rotation (quat): [x=%.3f, y=%.3f, z=%.3f, w=%.3f]",
      T_base_arm.transform.translation.x,
      T_base_arm.transform.translation.y,
      T_base_arm.transform.translation.z,
      T_base_arm.transform.rotation.x,
      T_base_arm.transform.rotation.y,
      T_base_arm.transform.rotation.z,
      T_base_arm.transform.rotation.w
    );
  } 
  catch (const tf2::TransformException& ex) 
  {
    RCLCPP_ERROR(node_->get_logger(),"TF lookup failed (%s -> %s): %s", kBaseLink, kArmBase, ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Build world->base_link transform from ground-truth base_pose (blackboard)
  tf2::Transform T_w_base;
  {
    tf2::Quaternion q(base_pose_w.orientation.x,
                      base_pose_w.orientation.y,
                      base_pose_w.orientation.z,
                      base_pose_w.orientation.w);
    tf2::Vector3 t(base_pose_w.position.x,
                   base_pose_w.position.y,
                   base_pose_w.position.z);
    T_w_base = tf2::Transform(q, t);
  }

  // Convert base->arm TF msg into tf2
  tf2::Transform T_base_arm_tf;
  tf2::fromMsg(T_base_arm.transform, T_base_arm_tf);

  //Compute world->arm_base and then arm_base->object
  tf2::Transform T_w_arm = T_w_base * T_base_arm_tf;

  tf2::Transform T_w_obj;
  {
    tf2::Quaternion q(object_pose_w.orientation.x,
                      object_pose_w.orientation.y,
                      object_pose_w.orientation.z,
                      object_pose_w.orientation.w);
    tf2::Vector3 t(object_pose_w.position.x,
                   object_pose_w.position.y,
                   object_pose_w.position.z);
    T_w_obj = tf2::Transform(q, t);
  }

  tf2::Transform T_arm_obj = T_w_arm.inverse() * T_w_obj;
  tf2::Vector3 p_obj = T_arm_obj.getOrigin();

  RCLCPP_INFO(node_->get_logger(),"Object in arm frame (%s): x=%.3f y=%.3f z=%.3f", kArmBase, p_obj.x(), p_obj.y(), p_obj.z());

  // Use current EE orientation (robust) and only move position
  geometry_msgs::msg::PoseStamped current = arm_->getCurrentPose(kEeLink);
//   geometry_msgs::msg::Quaternion ee_q = current.pose.orientation;
  geometry_msgs::msg::Quaternion ee_q;
  ee_q.x = 0.0;
  ee_q.y = 1.0;
  ee_q.z = 0.0;
  ee_q.w = 0.0;

  const std::string planning_frame = arm_->getPlanningFrame();
  if (planning_frame != std::string(kArmBase)) {
    RCLCPP_WARN(node_->get_logger(), "MoveIt planning frame is '%s' but node assumes '%s'. "
                "This may still work if they are equivalent, but verify.",
                planning_frame.c_str(), kArmBase);
  }

  auto make_target = [&](double z_offset) {
    geometry_msgs::msg::PoseStamped tgt;
    tgt.header.frame_id = planning_frame;
    tgt.header.stamp = node_->now();
    tgt.pose.position.x = p_obj.x();
    tgt.pose.position.y = p_obj.y();
    tgt.pose.position.z = p_obj.z() + z_offset;
    // tgt.pose.position.x = 0.5;
    // tgt.pose.position.y = 0.0;
    // tgt.pose.position.z = 0.6;

    tgt.pose.orientation = ee_q;
    return tgt;
  };

  geometry_msgs::msg::PoseStamped pre   = make_target(kPreZ);
  geometry_msgs::msg::PoseStamped grasp = make_target(kGraspZ);
  geometry_msgs::msg::PoseStamped lift  = make_target(kLiftZ);

  // Helper: plan+execute to a pose
  auto go = [&](const geometry_msgs::msg::PoseStamped& tgt, const char* label) -> bool {
    arm_->setPoseTarget(tgt, kEeLink);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!planned) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed for %s", label);
      return false;
    }

    auto exec = arm_->execute(plan);
    if (exec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed for %s", label);
      return false;
    }
    return true;
  };

  // 7) Execute sequence
  if (!go(pre, "pre-grasp")) {
    return BT::NodeStatus::FAILURE;
  }

  if (!go(grasp, "grasp")) {
    return BT::NodeStatus::FAILURE;
  }

  // Close gripper 
  if (!closeGripper(kCloseWidth, kCloseEffort)) {
    RCLCPP_WARN(node_->get_logger(), "Gripper close reported failure; continuing anyway (physics-only may still work)");
  }

  if (!go(lift, "lift")) {
    return BT::NodeStatus::FAILURE;
  }


  arm_->setJointValueTarget(home_joints);
  if (arm_->plan(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt planning failed for HOME (joint target)");
    return BT::NodeStatus::FAILURE;
  }
  if (arm_->execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt execution failed for HOME (joint target)");
    return BT::NodeStatus::FAILURE;
  }


  RCLCPP_INFO(node_->get_logger(), "PickObjectNode: SUCCESS");
  return BT::NodeStatus::SUCCESS;
}


PickObjectNode::~PickObjectNode()
{
  if (exec_) {
    exec_->cancel();
  }

  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}



}  

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::PickObjectNode>("PickObjectNode");
}
