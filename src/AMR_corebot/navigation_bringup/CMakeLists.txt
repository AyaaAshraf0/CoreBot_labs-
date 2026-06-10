cmake_minimum_required(VERSION 3.8)
project(navigation_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install maps
install(
  DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)
add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp) 
ament_target_dependencies(cmd_vel_bridge
  "rclcpp"
  "geometry_msgs"
)
install(TARGETS
  cmd_vel_bridge
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
