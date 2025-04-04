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
    rviz_config_path = os.path.join(get_package_share_directory('opendog_bringup'), 'config', 'opendog.rviz')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'pcl_ros', 'pcd_to_pointcloud',
                '--ros-args', '-p', 'file_name:=/home/pi/dog/src/1743676760.40377378.pcd'
            ],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
