#include necessary modules
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



sim_launch_file = os.path.join(
    get_package_share_directory('corebot_description'),
    'launch',
    'ign_mobile_robot.launch.py'
)

include_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                sim_launch_file)
        )
joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output="screen",
        
    )
diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller",
                   "--controller-manager", "/controller_manager"],
        output="screen",
        
    )
    
def generate_launch_description():
    return LaunchDescription([
        include_launch_file,
        joint_broadcaster,
        diff_controller,
    ])

