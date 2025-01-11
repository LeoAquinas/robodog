# robodog

Github repo to store shared changes among team members of Bottom Gear

# Controller
As the simulation was coded to make use of gamepad to control the robot, please download this to move the robot
https://remotegamepad.com/

When you want to read controller, run:
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

If you want to test if controller being read, run:
  ros2 topic echo joy
Move the controller and there should be data being read
