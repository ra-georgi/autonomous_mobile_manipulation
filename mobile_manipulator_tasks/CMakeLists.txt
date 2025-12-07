cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)



set(MSG_FILES
  "msg/SpawnedObject.msg"
)

set(SRV_FILES
  "srv/GetNextObject.srv"
)


include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES geometry_msgs
)



add_library(mobile_manipulator_tasks_core
  src/object_spawner.cpp
)

ament_target_dependencies(mobile_manipulator_tasks_core
  rclcpp
  geometry_msgs
  rosidl_default_runtime
  ament_index_cpp
)



add_executable(object_spawner_node
  src/object_spawner_node.cpp
)

target_link_libraries(object_spawner_node
  mobile_manipulator_tasks_core
)


ament_target_dependencies(object_spawner_node
  rclcpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mobile_manipulator_tasks_core "${cpp_typesupport_target}")


add_library(get_next_object_bt_node SHARED
  src/bt_nodes/get_next_object_bt_node.cpp
)

ament_target_dependencies(get_next_object_bt_node
  behaviortree_cpp
  rclcpp
  geometry_msgs
  rosidl_default_runtime
)

# Add generated interface include directories     #is this required?
target_include_directories(get_next_object_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# Link ROSIDL generated typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(get_next_object_bt_node "${cpp_typesupport_target}")

target_compile_definitions(get_next_object_bt_node PRIVATE BT_PLUGIN_EXPORT)


add_library(compute_base_goal_from_object_bt_node SHARED
  src/bt_nodes/compute_base_goal_from_object.cpp
)

ament_target_dependencies(compute_base_goal_from_object_bt_node
  behaviortree_cpp
  rclcpp
  geometry_msgs
)

# Export as BT plugin
target_compile_definitions(compute_base_goal_from_object_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS
  compute_base_goal_from_object_bt_node
  DESTINATION lib/${PROJECT_NAME}
)



add_library(navigate_to_pose_bt_node SHARED
  src/bt_nodes/navigate_to_pose_bt_node.cpp
)

ament_target_dependencies(navigate_to_pose_bt_node
  behaviortree_cpp
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

# Export as BT plugin
target_compile_definitions(navigate_to_pose_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS
  navigate_to_pose_bt_node
  DESTINATION lib/${PROJECT_NAME}
)




add_executable(bt_test_runner src/bt_test_runner.cpp)
ament_target_dependencies(bt_test_runner
  behaviortree_cpp
  rclcpp
  geometry_msgs
  ament_index_cpp
)


install(TARGETS bt_test_runner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS
  get_next_object_bt_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install the library to lib/
install(TARGETS
  mobile_manipulator_tasks_core
  DESTINATION lib
)

# Install the executable to lib/<pkg> so ros2 run can find it
install(TARGETS
  object_spawner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()












