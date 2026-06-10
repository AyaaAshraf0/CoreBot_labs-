cmake_minimum_required(VERSION 3.8)
project(corebot_bringup)

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

install(
  DIRECTORY config  launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp geometry_msgs)
install(TARGETS
  teleop_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
