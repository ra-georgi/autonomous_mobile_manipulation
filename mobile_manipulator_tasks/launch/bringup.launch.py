from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------------------------------------------------------
    # Arguments
    # ---------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_map = DeclareLaunchArgument('map', default_value='/home/reuben/moveit_ws/src/sonny_description/maps/my_world_map.yaml',description='Full path to map.yaml')
    declare_params = DeclareLaunchArgument('params_file', default_value='/home/reuben/moveit_ws/src/sonny_description/config/nav2_params.yaml',description='Full path to Nav2 params file')

    # ---------------------------------------------------------
    # Object Spawner 
    # ---------------------------------------------------------
    spawner_node = Node(
        package='mobile_manipulator_tasks',
        executable='object_spawner_node',
        name='object_spawner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------------------------------------------------
    # Nav2 bringup
    # ---------------------------------------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': nav2_params
        }.items()
    )

    # ---------------------------------------------------------
    # Nav2 RViz
    # ---------------------------------------------------------
    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ---------------------------------------------------------
    # BT Task Runner
    # ---------------------------------------------------------
    bt_task_runner = Node(
        package='mobile_manipulator_tasks',
        executable='bt_test_runner',
        name='bt_task_runner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------------------------------------------------
    # Launch Description
    # ---------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,

        spawner_node,
        nav2_bringup,
        nav2_rviz,
        # bt_task_runner,
    ])
