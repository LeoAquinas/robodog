cmake_minimum_required(VERSION 3.5)
project(opendog_teleop)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(opendog_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(./include/)

add_executable(opendog_teleop_joy_node src/opendog_teleop_joy.cpp)
ament_target_dependencies(opendog_teleop_joy_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  opendog_msgs
  geometry_msgs
  trajectory_msgs
)



install(TARGETS
  opendog_teleop_joy_node  
  DESTINATION lib/${PROJECT_NAME}
) 

install(DIRECTORY
  src
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
