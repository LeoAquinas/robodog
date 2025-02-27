# Robodog

Github repo to document project among team members of Bottom Gear

# Dog simulation






## SLAM
Slam would make use of multiple packages:
  Odometry for Mapping:
  EKF filter result from combining multiple pose data using robot localization package

  To get pose data we are going to use:
  1. LiDAR pose
  2. ORBSLAM3
  3. IMU

  Mapping:
  1. ORBSLAM3 for monocular camera mapping, Pointcloud data, & Pose
  2. slamtoolbox for 2D LiDAR mapping

  ### [LiDAR Pose](https://github.com/Adlink-ROS/rf2o_laser_odometry)
  git clone git@github.com:Adlink-ROS/rf2o_laser_odometry.git

  ### [robot_localization package](https://github.com/cra-ros-pkg/robot_localization/tree/humble-devel)
  install dependencies:
  sudo apt-get install libgeographic-dev
  sudo apt-get install geographiclib-tools

  git clone git@github.com:ros-geographic-info/geographic_info.git -b ros2

## Before Colcon Build
After installing all required packages, run rosdep to install all required packages
rosdep install --from-paths src --ignore-src -r -y

## Controller
Motor controller used for the robot dog is Moteus r4.11 FOC controller.
Documentation can be found [here](https://github.com/mjbots/moteus)

### Hardware Setup for Controller
Controller would come in seperate parts. [Need to assemble it in order to use](https://github.com/mjbots/moteus/blob/main/docs/getting_started.md#hardware)

### Software Setup for Controller
Before using controller with motor, they needs to be [callibrated](https://github.com/mjbots/moteus/blob/main/docs/getting_started.md#calibration)

### Testing and Run
After callibration, the motor can then be [run](https://github.com/mjbots/moteus/blob/main/docs/getting_started.md#software)

# Controller
As the simulation was coded to make use of gamepad to control the robot, please download this to move the robot
https://remotegamepad.com/

***** ENSURE THAT YOUR VIRTUAL MACHINE INTERNET CONNECTION IS SET TO BRIDGED *****

When you want to read controller, run:
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'

If you want to test if controller being read, run:
  ros2 topic echo joy
Move the controller and there should be data being read
