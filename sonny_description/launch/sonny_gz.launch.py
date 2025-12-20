import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_share = get_package_share_directory('sonny_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    moveit_config_share = get_package_share_directory('moveit_resources_panda_moveit_config')

    
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, "launch", "gz_spawn_model.launch.py")
    # default_model_path = os.path.join(pkg_share, 'src', 'description', 'sonny.sdf')
    default_model_path = os.path.join(pkg_share, 'urdf', 'sonny_main.urdf.xacro')
    
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'moveit.rviz')

    world_path = os.path.join(pkg_share, 'world', 'indoor_world.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')

    robot_controllers = os.path.join(moveit_config_share, 'config', 'ros2_controllers.yaml')

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    # URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'sonny_main.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            # 'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'sonny',
            'z': '0.55',
        }.items(),
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "--param-file", robot_controllers],
        output="screen",
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_hand_controller", "--param-file", robot_controllers],
        output="screen",
        condition=None,  
    )


    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config"
        )
        .robot_description(file_path=urdf_file
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .robot_description_kinematics(file_path="config/kinematics.yaml")  # Add this line
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
            # pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )    

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),    {"use_sim_time": True},],
        # parameters=[moveit_config.to_dict(),{'use_sim_time': use_sim_time,}],
    )    

    # servo_params = PathJoinSubstitution([pkg_share, "config", "servo.yaml"])   

    # servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node",
    #     output="screen",
    #     parameters=[
    #         moveit_config.to_dict(),
    #         servo_params,
    #         {"use_sim_time": True},
    #     ],
    # )




    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        panda_hand_controller_spawner,        
        run_move_group_node,
        # servo_node,
    ])