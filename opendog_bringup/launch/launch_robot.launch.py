import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():


    # Launch teleop, MPU6050 driver, odometry, agent and robot hardware interface & controllers

    # description_prefix = get_package_share_directory("opendog_description")
    # teleop_prefix = get_package_share_directory("werdna_teleop")
    # agent_prefix = get_package_share_directory("werdna_agent")

    # teleop_launch_file = os.path.join(teleop_prefix, "launch", "werdna_teleop.launch.py")

    # ros2_control_launch_file = os.path.join(description_prefix, "launch", "ros2_control.launch.py")


    # teleop = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(teleop_launch_file),
    # )

    # ros2_control = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(ros2_control_launch_file)
    # )
    
    # agent_node = Node(
    #     package=agent_prefix,
    #     executable="werdna_agent_node",
    # )
    
    # Launch ROS2 Control
    description_prefix = get_package_share_directory("opendog_description")
    xacro_file = os.path.join(description_prefix,'urdf','opendog.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

#####################
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("opendog_bringup"),
            "config",
            "opendog_joint_controller.yaml",
        ]
    )

    #if got problem with robot detection, try change output to both for all nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
#########################

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gazebo_joint_controller", "--controller-manager", "/controller_manager"],
    )


#############################
    camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{
                'image_size': [640,480],
                'camera_frame_id': 'camera_link_optical'
                }]
    )

    lidar_package_name = 'sllidar_ros2'
    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(lidar_package_name),'launch','view_sllidar_c1_launch.py'
                )]),
    )
################################

    return LaunchDescription([
        node_robot_state_publisher,
        control_node,
        joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_controller_spawner,
        # teleop,
        # ros2_control,
        # agent_node,
        # camera_node,
        lidar_launch,
    ])