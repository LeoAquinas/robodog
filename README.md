# robodog

Github repo to store shared changes among team members of Bottom Gear

# Dog simulation
Reference repo from here to run robodog sim:
https://github.com/NDHANA94/hyperdog_ros2
When you try to run the simulation, there would be some errors due to mistakes in the code. the issues have been fixed in this repo.
You can just substitute the repo from above link with this one, but you still need to follow the setup instructions


# Controller
As the simulation was coded to make use of gamepad to control the robot, please download this to move the robot
https://remotegamepad.com/

***** ENSURE THAT YOUR VIRTUAL MACHINE INTERNET CONNECTION IS SET TO BRIDGED *****

When you want to read controller, run:
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

If you want to test if controller being read, run:
  ros2 topic echo joy
Move the controller and there should be data being read
