import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_share = FindPackageShare("sonny_description")

    # Path to your combined xacro file
    xacro_file = PathJoinSubstitution([
        pkg_share, "urdf", "sonny_base.urdf.xacro"
    ])

    # Process the xacro into robot_description
    # robot_description = Command(["xacro ", xacro_file])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )


    # Robot State Publisher (publishes TF from URDF)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    # Joint State Publisher GUI (sliders to move joints in RViz)
    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # Launch RViz and load a config if you have one
    rviz_config = PathJoinSubstitution([
        pkg_share, "rviz", "view_robot.rviz"   # optional
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[],
        output="screen"
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui,
        rviz_node
    ])
