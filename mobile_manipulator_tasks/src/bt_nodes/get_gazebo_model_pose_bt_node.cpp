#include "mobile_manipulator_tasks/bt_nodes/get_gazebo_model_pose_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

namespace mobile_manipulator_tasks
{

static geometry_msgs::msg::Pose toRosPose(const gz::msgs::Pose& p)
{
  geometry_msgs::msg::Pose out;
  out.position.x = p.position().x();
  out.position.y = p.position().y();
  out.position.z = p.position().z();
  out.orientation.x = p.orientation().x();
  out.orientation.y = p.orientation().y();
  out.orientation.z = p.orientation().z();
  out.orientation.w = p.orientation().w();
  return out;
}

GetGazeboModelPoseBT::GetGazeboModelPoseBT(
    const std::string& name,
    const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("get_gazebo_model_pose_bt_node");
}

BT::PortsList GetGazeboModelPoseBT::providedPorts()
{
  return {
    // Which model/entity name to look up (e.g. "cyl_red_1")
    BT::InputPort<std::string>("model_name"),

    // World name to subscribe to (default "my_world" if not provided)
    BT::InputPort<std::string>("world_name"),

    // Output pose in world coordinates
    BT::OutputPort<geometry_msgs::msg::Pose>("model_pose")
  };
}

void GetGazeboModelPoseBT::ensureSubscribed(const std::string& world_name)
{
  if (subscribed_) {
    return;
  }

  subscribed_world_ = world_name;
  pose_topic_ = "/world/" + subscribed_world_ + "/pose/info";

  bool ok = gz_node_.Subscribe(
      pose_topic_,
      &GetGazeboModelPoseBT::onPoseInfo,
      this);

  if (!ok) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to subscribe to Gazebo pose topic: %s",
                 pose_topic_.c_str());
    subscribed_ = false;
    return;
  }

  subscribed_ = true;
  RCLCPP_INFO(node_->get_logger(),
              "Subscribed to Gazebo pose topic: %s",
              pose_topic_.c_str());
}

void GetGazeboModelPoseBT::onPoseInfo(const gz::msgs::Pose_V& msg)
{
  std::lock_guard<std::mutex> lock(cache_mutex_);

  // Update cache with latest pose for each entity name
  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto& pose = msg.pose(i);

    // Entity name (Gazebo uses this to identify models/links)
    const std::string& name = pose.name();
    if (name.empty()) {
      continue;
    }

    pose_cache_[name] = toRosPose(pose);
  }
}

BT::NodeStatus GetGazeboModelPoseBT::tick()
{
  // Subscribe lazily on first tick so XML can provide world_name
  std::string world_name = "my_world";
  (void)getInput<std::string>("world_name", world_name);
  ensureSubscribed(world_name);

  if (!subscribed_) {
    return BT::NodeStatus::FAILURE;
  }

  std::string model_name;
  if (!getInput<std::string>("model_name", model_name)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "GetGazeboModelPoseBT: missing required input [model_name]");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Pose pose_out;
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = pose_cache_.find(model_name);
    if (it == pose_cache_.end()) {
      // Not seen yet (model may not exist/spawned)
      return BT::NodeStatus::FAILURE;
    }
    pose_out = it->second;
  }

  setOutput("model_pose", pose_out);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mobile_manipulator_tasks

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::GetGazeboModelPoseBT>(
      "GetGazeboModelPose");
}
