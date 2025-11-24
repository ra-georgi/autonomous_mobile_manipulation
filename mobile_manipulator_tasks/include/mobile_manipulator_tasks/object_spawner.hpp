#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// #include <mobile_manipulator_tasks/msg/spawned_object.hpp>
// #include <mobile_manipulator_tasks/srv/get_next_object.hpp>

#include "mobile_manipulator_tasks/msg/spawned_object.hpp"
#include "mobile_manipulator_tasks/srv/get_next_object.hpp"



namespace mobile_manipulator_tasks
{

struct SpawnEvent
{
  double spawn_time;                 // seconds, in simulation time
  std::string color;                 // "red" or "green"
  std::string name;                  // entity name in Gazebo
  geometry_msgs::msg::Pose pose;     // pose in world frame
  bool spawned = false;              // has it been spawned in Gazebo?
  bool served  = false;              // has it already been returned by the service?
};


class ObjectSpawnerNode : public rclcpp::Node
{
public:
  explicit ObjectSpawnerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timerCallback();
  bool spawnEvent(const SpawnEvent & event);
  void handleGetNextObject(
    const mobile_manipulator_tasks::srv::GetNextObject::Request::SharedPtr request,
    mobile_manipulator_tasks::srv::GetNextObject::Response::SharedPtr response);


  std::string world_name_;

  std::string red_model_path_;
  std::string green_model_path_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<SpawnEvent> schedule_;

  rclcpp::Service<mobile_manipulator_tasks::srv::GetNextObject>::SharedPtr get_next_object_service_; 

};

}  




