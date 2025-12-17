#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include <condition_variable>
#include <mutex>
#include <string>

// Gazebo Transport + Msgs
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

namespace mobile_manipulator_tasks
{

class GetPoseGroundTruthBT : public BT::SyncActionNode
{
public:
  GetPoseGroundTruthBT(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void onPoseInfo(const gz::msgs::Pose_V& msg);
  static geometry_msgs::msg::Pose toRosPose(const gz::msgs::Pose& p);

  static constexpr const char* kWorldName = "my_world";
  static constexpr const char* kModelName = "sonny";
  static constexpr int kTimeoutMs = 2000;

  rclcpp::Node::SharedPtr node_;
  gz::transport::Node gz_node_;

  // Synchronization between callback and tick()
  std::mutex mtx_;
  std::condition_variable cv_;

  geometry_msgs::msg::Pose latest_pose_{};
  uint64_t latest_seq_{0};        
  bool subscribed_{false};
};

} 
