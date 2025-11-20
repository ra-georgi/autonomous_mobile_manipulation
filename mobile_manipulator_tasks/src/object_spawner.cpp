#include "mobile_manipulator_tasks/object_spawner.hpp"
#include <chrono>
#include <cstdlib>  // for std::system
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
// using ros_gz_interfaces::srv::SpawnEntity;

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

  // // Create SpawnEntity client to Gazebo
  // const std::string service_name = "/world/" + world_name_ + "/create";
  // spawn_client_ = this->create_client<SpawnEntity>(service_name);

  // RCLCPP_INFO(get_logger(), "Waiting for SpawnEntity service [%s] ...", service_name.c_str());

  // // Wait until the service is available
  // while (!spawn_client_->wait_for_service(1s) && rclcpp::ok()) 
  // {
  //   RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
  //                        "Still waiting for service [%s] ...",
  //                        service_name.c_str());
  // }

  // if (!rclcpp::ok()) {
  //   RCLCPP_ERROR(get_logger(), "ROS shutdown while waiting for SpawnEntity service.");
  //   return;
  // }

  // RCLCPP_INFO(get_logger(), "SpawnEntity service is available.");

  // Build a simple spawn schedule (all times in *simulation* seconds)
  {
    SpawnEvent e1;
    e1.spawn_time = 5.0;             // at sim time t=5s
    e1.color      = "red";
    e1.name       = "cyl_red_1";
    e1.pose.position.x = 1.0;
    e1.pose.position.y = 0.5;
    e1.pose.position.z = 0.06;       
    e1.pose.orientation.x = 0.0;
    e1.pose.orientation.y = 0.0;
    e1.pose.orientation.z = 0.0;
    e1.pose.orientation.w = 1.0;
    schedule_.push_back(e1);
  }

  {
    SpawnEvent e2;
    e2.spawn_time = 10.0;           
    e2.color      = "green";
    e2.name       = "cyl_green_1";
    e2.pose.position.x = 1.3;
    e2.pose.position.y = 0.5;
    e2.pose.position.z = 0.06;
    e2.pose.orientation.x = 0.0;
    e2.pose.orientation.y = 0.0;
    e2.pose.orientation.z = 0.0;
    e2.pose.orientation.w = 1.0;
    schedule_.push_back(e2);
  }

  {
    SpawnEvent e3;
    e3.spawn_time = 15.0;           
    e3.color      = "red";
    e3.name       = "cyl_red_2";
    e3.pose.position.x = 1.6;
    e3.pose.position.y = 0.5;
    e3.pose.position.z = 0.06;
    e3.pose.orientation.x = 0.0;
    e3.pose.orientation.y = 0.0;
    e3.pose.orientation.z = 0.0;
    e3.pose.orientation.w = 1.0;
    schedule_.push_back(e3);
  }

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

  // Build the command. We only use position (no roll/pitch/yaw for now).
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

} 
