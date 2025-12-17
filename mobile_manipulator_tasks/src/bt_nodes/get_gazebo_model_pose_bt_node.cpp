#include "mobile_manipulator_tasks/bt_nodes/get_gazebo_model_pose_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>

namespace mobile_manipulator_tasks
{

geometry_msgs::msg::Pose GetPoseGroundTruthBT::toRosPose(const gz::msgs::Pose& p)
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

GetPoseGroundTruthBT::GetPoseGroundTruthBT(const std::string& name, const BT::NodeConfiguration& config): BT::SyncActionNode(name, config)
{
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  opts.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  node_ = rclcpp::Node::make_shared("get_sonny_pose_ground_truth_bt_node", opts);
  const std::string topic = std::string("/world/") + kWorldName + "/pose/info";

  subscribed_ = gz_node_.Subscribe(topic, &GetPoseGroundTruthBT::onPoseInfo, this);

  if (subscribed_) 
  {
    RCLCPP_INFO(node_->get_logger(), "Subscribed to Gazebo ground-truth pose topic: %s (model=%s)", topic.c_str(), kModelName);
  } 
  else 
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to subscribe to Gazebo pose topic: %s", topic.c_str());
  }
}

BT::PortsList GetPoseGroundTruthBT::providedPorts()
{
  return {BT::OutputPort<geometry_msgs::msg::Pose>("model_pose")};
}

void GetPoseGroundTruthBT::onPoseInfo(const gz::msgs::Pose_V& msg)
{
  // Find the pose entry 
  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto& pose = msg.pose(i);

    // IMPORTANT: the name field must match exactly what Gazebo publishes.
    if (pose.name() == kModelName) {
      std::lock_guard<std::mutex> lock(mtx_);
      latest_pose_ = toRosPose(pose);
      latest_seq_++;
      cv_.notify_all();
      return;
    }
  }
}

BT::NodeStatus GetPoseGroundTruthBT::tick()
{
  if (!subscribed_) 
  {
    return BT::NodeStatus::FAILURE;
  }

  // Want a *fresh* sample that arrives after this tick starts.
  uint64_t start_seq;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    start_seq = latest_seq_;
  }

  std::unique_lock<std::mutex> lock(mtx_);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kTimeoutMs);

  const bool got_fresh = cv_.wait_until(lock, deadline, [&]() {
    return latest_seq_ > start_seq;
  });

  if (!got_fresh) {
    RCLCPP_WARN(node_->get_logger(), "Timeout waiting for fresh Gazebo ground-truth pose for '%s' (world=%s). "
                "Make sure pose topic is publishing and model name matches.", kModelName, kWorldName);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("model_pose", latest_pose_);
  RCLCPP_INFO(node_->get_logger(), "GZ ground-truth pose | "
    "pos: [%.3f, %.3f, %.3f] | "
    "quat: [%.3f, %.3f, %.3f, %.3f]",
    latest_pose_.position.x,
    latest_pose_.position.y,
    latest_pose_.position.z,
    latest_pose_.orientation.x,
    latest_pose_.orientation.y,
    latest_pose_.orientation.z,
    latest_pose_.orientation.w
  );
  
  return BT::NodeStatus::SUCCESS;
}

}  

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_tasks::GetPoseGroundTruthBT>("GetPoseGroundTruth");
}
