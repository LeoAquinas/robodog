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

    foxglove_bridge_node = ExecuteProcess(
            cmd=['ros2', 'launch', 'foxglove_bridge', 'foxglove_bridge_launch.xml'],
            output='screen'
        )

    orbslam = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'orbslam3_pose', 'mono',
                '/home/pi/dog/src/slam/ORB_SLAM3/Vocabulary/ORBvoc.txt',
                '/home/pi/dog/src/slam/ORB_SLAM3/Examples/Monocular/TUM1.yaml'
            ],
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
        camera_node,
        foxglove_bridge_node,
        rviz,
        orbslam

    ])