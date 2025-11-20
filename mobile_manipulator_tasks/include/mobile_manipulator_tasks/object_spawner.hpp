#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace mobile_manipulator_tasks
{

struct SpawnEvent
{
  double spawn_time;                 // seconds, in simulation time
  std::string color;                 // "red" or "green"
  std::string name;                  // entity name in Gazebo
  geometry_msgs::msg::Pose pose;     // pose in world frame
  bool spawned = false;
};

class ObjectSpawnerNode : public rclcpp::Node
{
public:
  explicit ObjectSpawnerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timerCallback();
  bool spawnEvent(const SpawnEvent & event);

  std::string world_name_;

  std::string red_model_path_;
  std::string green_model_path_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<SpawnEvent> schedule_;
};

}  


