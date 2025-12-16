#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

// Gazebo Transport + Msgs
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

namespace mobile_manipulator_tasks
{

class GetGazeboModelPoseBT : public BT::SyncActionNode
{
public:
  GetGazeboModelPoseBT(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void ensureSubscribed(const std::string& world_name);
  void onPoseInfo(const gz::msgs::Pose_V& msg);

  rclcpp::Node::SharedPtr node_;

  gz::transport::Node gz_node_;
  std::atomic<bool> subscribed_{false};
  std::string subscribed_world_;
  std::string pose_topic_;

  std::mutex cache_mutex_;
  std::unordered_map<std::string, geometry_msgs::msg::Pose> pose_cache_;
};

}  