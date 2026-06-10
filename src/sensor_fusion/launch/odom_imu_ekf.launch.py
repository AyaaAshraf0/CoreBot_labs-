import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


fusion_package = FindPackageShare(package="sensor_fusion").find("sensor_fusion")
config_file = os.path.join(fusion_package,"config","ekf_odom_imu.yaml")

def generate_launch_description():
  
    
    ekf_node = Node(
        package= "robot_localization",
        executable= "ekf_node",
        name = "ekf_filter_node",
        parameters=[config_file],
        
    )
    

    ld = LaunchDescription()
    ld.add_action(ekf_node)
    

    return ld