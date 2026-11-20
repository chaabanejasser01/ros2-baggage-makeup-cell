cmake_minimum_required(VERSION 3.8)
project(bag_cell_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(bag_cell_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

add_executable(cell_node src/cell_node.cpp)
ament_target_dependencies(cell_node rclcpp moveit_ros_planning_interface geometry_msgs
  moveit_msgs shape_msgs bag_cell_interfaces)
target_link_libraries(cell_node gz-transport::core gz-msgs::core)

# Gazebo -> ROS bag poses by name (the ros_gz bridge drops the names)
add_executable(gz_pose_relay src/gz_pose_relay.cpp)
ament_target_dependencies(gz_pose_relay rclcpp tf2_msgs)
target_link_libraries(gz_pose_relay gz-transport::core gz-msgs::core)

install(TARGETS cell_node gz_pose_relay
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
