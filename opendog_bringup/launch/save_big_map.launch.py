import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'pcl_ros', 'pointcloud_to_pcd',
                '--ros-args', '--remap', 'input:=/all_mappoints'
            ],
            output='screen'
        )
    ])
