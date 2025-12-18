#include "mobile_manipulator_tasks/object_spawner.hpp"
#include <chrono>
#include <cstdlib>  // for std::system
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional> 

using namespace std::chrono_literals;

namespace mobile_manipulator_tasks
{

ObjectSpawnerNode::ObjectSpawnerNode(const rclcpp::NodeOptions & options): rclcpp::Node("object_spawner", options)
{
  // Parameter: world name (Gazebo world)
  world_name_ = this->declare_parameter<std::string>("world_name", "my_world");

  // Resolve model paths 
  const std::string pkg_share = ament_index_cpp::get_package_share_directory("mobile_manipulator_tasks");

  red_model_path_   = pkg_share + "/models/red_cylinder.sdf";
  green_model_path_ = pkg_share + "/models/green_cylinder.sdf";

  RCLCPP_INFO(get_logger(), "Red model path:   %s", red_model_path_.c_str());
  RCLCPP_INFO(get_logger(), "Green model path: %s", green_model_path_.c_str());


  // Service: provide the next spawned object (one per call)
  get_next_object_service_ =
    this->create_service<mobile_manipulator_tasks::srv::GetNextObject>("get_next_object",
      std::bind(
        &ObjectSpawnerNode::handleGetNextObject,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Service 'get_next_object' ready.");


  // Build a simple spawn schedule (all times in *simulation* seconds)
  {
    SpawnEvent e1;
    e1.spawn_time = 10.0;             // at sim time t=5s
    e1.color      = "green";
    e1.name       = "cyl_green_1";
    e1.pose.position.x = 0.8;
    e1.pose.position.y = 1.0;
    e1.pose.position.z = 0.56;       
    e1.pose.orientation.x = 0.0;
    e1.pose.orientation.y = 0.0;
    e1.pose.orientation.z = 0.0;
    e1.pose.orientation.w = 1.0;
    schedule_.push_back(e1);
  }

  // {
  //   SpawnEvent e2;
  //   e2.spawn_time = 40.0;           
  //   e2.color      = "green";
  //   e2.name       = "cyl_green_2";
  //   e2.pose.position.x = 1.0;
  //   e2.pose.position.y = -1.0;
  //   e2.pose.position.z = 0.56;
  //   e2.pose.orientation.x = 0.0;
  //   e2.pose.orientation.y = 0.0;
  //   e2.pose.orientation.z = 0.0;
  //   e2.pose.orientation.w = 1.0;
  //   schedule_.push_back(e2);
  // }

  // {
  //   SpawnEvent e3;
  //   e3.spawn_time = 50.0;           
  //   e3.color      = "red";
  //   e3.name       = "cyl_red_1";
  //   e3.pose.position.x = -1.5;
  //   e3.pose.position.y = 1.5;
  //   e3.pose.position.z = 0.56;
  //   e3.pose.orientation.x = 0.0;
  //   e3.pose.orientation.y = 0.0;
  //   e3.pose.orientation.z = 0.0;
  //   e3.pose.orientation.w = 1.0;
  //   schedule_.push_back(e3);
  // }

  // {
  //   SpawnEvent e4;
  //   e4.spawn_time = 60.0;           
  //   e4.color      = "green";
  //   e4.name       = "cyl_green_3";
  //   e4.pose.position.x = -1.5;
  //   e4.pose.position.y = -1.5;
  //   e4.pose.position.z = 0.56;
  //   e4.pose.orientation.x = 0.0;
  //   e4.pose.orientation.y = 0.0;
  //   e4.pose.orientation.z = 0.0;
  //   e4.pose.orientation.w = 1.0;
  //   schedule_.push_back(e4);
  // }


  // Timer: check schedule at 10 Hz
  timer_ = this->create_wall_timer(100ms, std::bind(&ObjectSpawnerNode::timerCallback, this));
}

void ObjectSpawnerNode::timerCallback()
{
  // Use ROS time (simulation time when /clock is used and use_sim_time is true)
  const double t = this->get_clock()->now().seconds();  // double seconds

  for (auto & event : schedule_) {
    if (!event.spawned && t >= event.spawn_time) {
      RCLCPP_INFO(get_logger(), "Time %.2f: spawning %s cylinder '%s'", t, event.color.c_str(), event.name.c_str());

      const bool success = spawnEvent(event);
      if (success) {
        event.spawned = true;
      } else {
        RCLCPP_WARN(get_logger(),
                    "Failed to spawn '%s' (will try again next timer tick).",
                    event.name.c_str());
      }
    }
  }
}

bool ObjectSpawnerNode::spawnEvent(const SpawnEvent & event)
{
  // Choose model path based on color
  std::string model_path;
  if (event.color == "red") {
    model_path = red_model_path_;
  } else if (event.color == "green") {
    model_path = green_model_path_;
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown color '%s' for event '%s'",
                 event.color.c_str(), event.name.c_str());
    return false;
  }

  const auto & p = event.pose.position;

  // Build the command. We only use position 
  std::stringstream cmd;
  cmd << "ros2 run ros_gz_sim create"
      << " -world " << world_name_
      << " -file "  << model_path
      << " -name "  << event.name
      << " -x " << p.x
      << " -y " << p.y
      << " -z " << p.z;

  const std::string cmd_str = cmd.str();

  RCLCPP_INFO(get_logger(), "Running command: %s", cmd_str.c_str());

  int ret = std::system(cmd_str.c_str());

  if (ret != 0) {
    RCLCPP_ERROR(get_logger(), "Command failed with return code %d", ret);
    return false;
  }

  return true;
}


void ObjectSpawnerNode::handleGetNextObject(
  const mobile_manipulator_tasks::srv::GetNextObject::Request::SharedPtr ,
  mobile_manipulator_tasks::srv::GetNextObject::Response::SharedPtr response)
{
  // Default: no object available
  response->success = false;

  // Iterate schedule in order, find first spawned & not yet served
  for (auto & event : schedule_) {
    if (event.spawned && !event.served) {
      // Fill response
      mobile_manipulator_tasks::msg::SpawnedObject obj_msg;
      obj_msg.name  = event.name;
      obj_msg.color = event.color;
      obj_msg.pose  = event.pose;

      response->object  = obj_msg;
      response->success = true;

      // Mark as served so we don't return it again
      event.served = true;

      RCLCPP_INFO(get_logger(),
                  "GetNextObject: returning '%s' (%s)",
                  event.name.c_str(), event.color.c_str());
      return;
    }
  }

  // If we reach here, nothing was found
  RCLCPP_DEBUG(get_logger(),
               "GetNextObject: no new spawned objects available yet.");
}



} 
