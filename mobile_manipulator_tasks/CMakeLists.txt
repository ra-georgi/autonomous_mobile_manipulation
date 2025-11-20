cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(ros_gz_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_library(mobile_manipulator_tasks_core
  src/object_spawner.cpp
)

ament_target_dependencies(mobile_manipulator_tasks_core
  rclcpp
  geometry_msgs
  # ros_gz_interfaces
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

install(TARGETS
  mobile_manipulator_tasks_core
  object_spawner_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()










