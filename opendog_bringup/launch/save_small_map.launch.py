import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli', 
                '-f', '/home/pi/dog/src'
            ],
            output='screen'
        )
    ])
