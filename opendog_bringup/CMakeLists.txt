cmake_minimum_required(VERSION 3.8)
project(opendog_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(opendog_joint_ctrl_node src/opendog_joint_controller.cpp)
ament_target_dependencies(opendog_joint_ctrl_node
  rclcpp  
  std_msgs 
)

install(TARGETS
opendog_joint_ctrl_node  
  DESTINATION lib/${PROJECT_NAME}
) 

install(
  DIRECTORY launch config src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
