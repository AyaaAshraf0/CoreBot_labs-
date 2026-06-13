cmake_minimum_required(VERSION 3.8)
project(corebot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(corebot_interfaces REQUIRED)

add_library(corebot_base_hardware SHARED 
          src/corebot_base_hardware.cpp 
          src/wheel.cpp  
          src/motor_encoder.cpp 
)

target_include_directories(corebot_base_hardware PRIVATE include)

ament_target_dependencies(
  corebot_base_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  corebot_interfaces
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface corebot_base_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/corebot_hardware
)

install(
  TARGETS corebot_base_hardware
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

# EXPORTS 
ament_export_libraries(corebot_base_hardware)

add_executable(test_corebot_hw_plugin
  test/test_plugin_loader.cpp
)

ament_target_dependencies(test_corebot_hw_plugin
  pluginlib
  hardware_interface
)

install(TARGETS test_corebot_hw_plugin
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(imu_complementary_filter src/imu_filter.cpp)

ament_target_dependencies(
  imu_complementary_filter
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(get_imu_covariance src/get_imu_covariance.cpp)

ament_target_dependencies(
  get_imu_covariance
  rclcpp
  sensor_msgs
  tf2_ros
  
)

install(TARGETS
  imu_complementary_filter  
  get_imu_covariance
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
