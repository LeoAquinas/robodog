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

```
$ git clone git@github.com:Adlink-ROS/rf2o_laser_odometry.git
```

### [robot_localization package](https://github.com/cra-ros-pkg/robot_localization/tree/humble-devel)
install dependencies:
```
git clone git@github.com:ros-geographic-info/geographic_info.git -b ros2
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
```
install package:
```
git clone git@github.com:cra-ros-pkg/robot_localization.git -b humble-devel
```
During execution of robot localization efk node, if odom result returns as NaN, just bump up queue size. NaN happens because data loss from small queue

### ORBSLAM3
- Follow [this](https://github.com/ozandmrz/raspberry_pi_visual_slam) to install ORBSLAM3\

Troubleshooting
-  Pangolin error:
1. if during Pangolin cmake build got [epoxy library issue](https://stackoverflow.com/questions/78583957/cmake-doesnt-see-epoxy-lib), install library:
```
    sudo apt-get install libepoxy-dev
```
2. if during cmake build got error somewhere during 7~10% compilation build, try to include suppression in CmakeList
```
   add_compile_options(-Wno-type-limits)
```
- ORBSLAM3 build error
1. When doing final build for ORBSLAM3, run ```./build.sh``` file like normal ORBSLAM build instead of following the guide
2. Before running the above cmd, check [this](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/118#issuecomment-726648539) out so that system does not crash

- colcon build crash
Can try [this](https://github.com/introlab/rtabmap_ros/issues/916#issuecomment-1503035478) cmd when ```colcon build``` if system keeps on crashing
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
```

### Save ORBSLAM Pointcloud Data
```
git clone git@github.com:ros-perception/perception_pcl.git
```
COLCON IGNORE pcl_conversions
in pcl_ros CmakeList comment out filter packages in cmake list
```
### Declare the pcl_ros_filters library
#add_library(pcl_ros_filters SHARED
#  src/pcl_ros/filters/extract_indices.cpp
#  src/pcl_ros/filters/filter.cpp
#  src/pcl_ros/filters/passthrough.cpp
# src/pcl_ros/filters/project_inliers.cpp
#  src/pcl_ros/filters/radius_outlier_removal.cpp
#  src/pcl_ros/filters/statistical_outlier_removal.cpp
#  src/pcl_ros/filters/voxel_grid.cpp
#  src/pcl_ros/filters/crop_box.cpp
#)
#target_link_libraries(pcl_ros_filters pcl_ros_tf ${PCL_LIBRARIES})
#ament_target_dependencies(pcl_ros_filters ${dependencies})
#rclcpp_components_register_node(pcl_ros_filters
#  PLUGIN "pcl_ros::ExtractIndices"
#  EXECUTABLE filter_extract_indices_node
#)
#rclcpp_components_register_node(pcl_ros_filters
#  PLUGIN "pcl_ros::PassThrough"
#  EXECUTABLE filter_passthrough_node
#)
#rclcpp_components_register_node(pcl_ros_filters
#  PLUGIN "pcl_ros::ProjectInliers"
#  EXECUTABLE filter_project_inliers_node
#)
#rclcpp_components_register_node(pcl_ros_filters
#  PLUGIN "pcl_ros::RadiusOutlierRemoval"
#  EXECUTABLE filter_radius_outlier_removal_node
#)
#rclcpp_components_register_node(pcl_ros_filters
#  PLUGIN "pcl_ros::StatisticalOutlierRemoval"
#  EXECUTABLE filter_statistical_outlier_removal_node
#)
#rclcpp_components_register_node(pcl_ros_filters
#  PLUGIN "pcl_ros::CropBox"
#  EXECUTABLE filter_crop_box_node
#)
#rclcpp_components_register_node(pcl_ros_filters
#  PLUGIN "pcl_ros::VoxelGrid"
#  EXECUTABLE filter_voxel_grid_node
#)
#class_loader_hide_library_symbols(pcl_ros_filters)
```

To save_map
```ros2 run pcl_ros pointcloud_to_pcd --ros-args --remap input:={point cloud topic}```

open_map
```ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:="{path to pcd file}"```

## Before Colcon Build
After installing all required packages, run rosdep to install all required packages
```
rosdep install --from-paths src --ignore-src -r -y
```

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
```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

If you want to test if controller being read, run:
```
ros2 topic echo joy
```
Move the controller and there should be data being read
