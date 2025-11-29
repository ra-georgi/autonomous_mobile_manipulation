#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 1) Create BT factory
  BT::BehaviorTreeFactory factory;

  // 2) Register plugin (.so) from install space
  const std::string pkg_prefix = ament_index_cpp::get_package_prefix("mobile_manipulator_tasks");

  // const std::string plugin_path = pkg_prefix + "/lib/mobile_manipulator_tasks/libget_next_object_bt_node.so";
  // std::cout << "Loading BT plugin: " << plugin_path << std::endl;
  // factory.registerFromPlugin(plugin_path);

  // GetNextObject plugin
  const std::string get_next_plugin = pkg_prefix + "/lib/mobile_manipulator_tasks/libget_next_object_bt_node.so";
  std::cout << "Loading BT plugin: " << get_next_plugin << std::endl;
  factory.registerFromPlugin(get_next_plugin);

  // ComputeBaseGoalFromObject plugin
  const std::string compute_goal_plugin = pkg_prefix + "/lib/mobile_manipulator_tasks/libcompute_base_goal_from_object_bt_node.so";
  std::cout << "Loading BT plugin: " << compute_goal_plugin << std::endl;
  factory.registerFromPlugin(compute_goal_plugin);

  // 3) Load XML tree from share dir
  const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("mobile_manipulator_tasks");
  // const std::string xml_path = pkg_share + "/bt_xml/test_get_next_object.xml";
  const std::string xml_path = pkg_share + "/bt_xml/test_go_to_object.xml";


  std::cout << "Loading BT XML: " << xml_path << std::endl;
  auto tree = factory.createTreeFromFile(xml_path);

  // 4) Tick at 1 Hz
  rclcpp::Rate rate(1.0);

  while (rclcpp::ok())
  {
    std::cout << "\n--- TICK ---" << std::endl;

    BT::NodeStatus status = tree.tickOnce();
    std::cout << "Status: " << BT::toStr(status) << std::endl;

    if (status == BT::NodeStatus::SUCCESS)
    {
      std::string name;
      geometry_msgs::msg::Pose pose;

      geometry_msgs::msg::PoseStamped nav_goal;
      bool ok_nav = tree.rootBlackboard()->get("nav_goal", nav_goal);


      bool ok_name = tree.rootBlackboard()->get("obj_name", name);
      bool ok_pose = tree.rootBlackboard()->get("obj_pose", pose);

      if (ok_name && ok_pose)
      {
        std::cout << "Got object: " << name
                  << " at (" << pose.position.x
                  << ", " << pose.position.y
                  << ", " << pose.position.z << ")\n";
      }
      else
      {
        std::cout << "SUCCESS but blackboard keys missing\n";
      }

      if (ok_nav)
      {
        std::cout << "nav_goal = ("
                  << nav_goal.pose.position.x << ", "
                  << nav_goal.pose.position.y << ", "
                  << nav_goal.pose.position.z << "), yaw = 0"
                  << std::endl;
      }
      else
      {
        std::cout << "SUCCESS but nav blackboard keys missing\n";
      }

    }

    rate.sleep();
  }


  rclcpp::shutdown();
  return 0;
}
