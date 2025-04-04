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
            )

    slam_node = Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[{
                    'slam_params_file' : '/home/pi/dog/src/lidar/mapper_params_online_async.yaml',
                    'use_sim_time' : False}],
            )


    foxglove_bridge_node = ExecuteProcess(
            cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml'],
            output='screen'
        )

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
        lidar_launch,
        lidar_odom,
        slam_node,
        foxglove_bridge_node,
        rviz
    ])