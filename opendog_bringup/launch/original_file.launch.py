import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
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

    # teleop = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(teleop_launch_file),
    # )
    
    # agent_node = Node(
    #     package=agent_prefix,
    #     executable="werdna_agent_node",
    # )
    
    # # Launch ROS2 Control
    # description_prefix = get_package_share_directory("opendog_description")
    # xacro_file = os.path.join(description_prefix,'urdf','opendog.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)

    # params = {'robot_description': robot_description_config.toxml()}

     # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("opendog_description"), "urdf", "opendog.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("opendog_bringup"),
            "config",
            "opendog_joint_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    #Controllers

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # joint_trajectory_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    # )

    forward_command_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gazebo_joint_controller", "--controller-manager", "/controller_manager"],
    )

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'joint_state_broadcaster'],
    #     output='both' )
  
    # load_forward_command_controller = ExecuteProcess(
    #         cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
    #             'gazebo_joint_controller'],
    #         output='both'
    #     )

    load_joint_control_node = ExecuteProcess(
        cmd=['ros2', 'run', 'opendog_bringup', 'opendog_joint_ctrl_node'],
        output='both'
    )

#############################

    #Peripherals
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

    lidar_odom = Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[{
                    'laser_scan_topic' : '/scan',
                    'odom_topic' : '/odom_rf2o',
                    'publish_tf' : True,
                    'base_frame_id' : 'base_link',
                    'odom_frame_id' : 'odom',
                    'init_pose_from_topic' : '',
                    'freq' : 20.0}],
            ),
################################

    #Teleop
    # teleop = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource('/home/pi/dog/src/robodog/opendog_launch/launch/opendog.launch.py'),
    #     )

    #Rviz
    rviz_config_path = os.path.join(get_package_share_directory('opendog_bringup'), 'config', 'opendog.rviz')
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )

    return LaunchDescription([
        robot_state_pub_node,
        control_node,
        joint_state_publisher_node,
        
        joint_state_broadcaster_spawner,
        # joint_trajectory_controller,
        forward_command_controller,

        # load_joint_state_controller,
        # load_forward_command_controller,
        # load_joint_control_node,
        # agent_node,

        # camera_node,
        # lidar_launch,
        # lidar_odom,
        # rviz
    ])