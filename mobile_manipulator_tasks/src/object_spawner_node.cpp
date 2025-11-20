#include "mobile_manipulator_tasks/object_spawner.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mobile_manipulator_tasks::ObjectSpawnerNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
