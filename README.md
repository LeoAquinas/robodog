# Robodog

Github repo to document project among team members of Bottom Gear

# Dog simulation





## SLAM



## Controller
Motor controller used for the robot dog is Moteus r4.11 FOC controller.
Documentation can be found at https://github.com/mjbots/moteus

### Hardware Setup for Controller
https://github.com/mjbots/moteus
Controller would come in seperate parts. Need to assemble it in order to use

### Software Setup for Controller
Before using controller with motor, they needs to be callibrated: https://github.com/mjbots/moteus

### Testing and Run
After callibration, the motor can then be run: https://github.com/mjbots/moteus

# Controller
As the simulation was coded to make use of gamepad to control the robot, please download this to move the robot
https://remotegamepad.com/

***** ENSURE THAT YOUR VIRTUAL MACHINE INTERNET CONNECTION IS SET TO BRIDGED *****

When you want to read controller, run:
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

If you want to test if controller being read, run:
  ros2 topic echo joy
Move the controller and there should be data being read
