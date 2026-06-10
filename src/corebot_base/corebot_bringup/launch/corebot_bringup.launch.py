from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sensor_fusion = LaunchConfiguration('use_sensor_fusion')
    use_lidar = LaunchConfiguration('use_lidar')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('corebot_description'),
                'launch',
                'corebot_controllers.launch.py'
            )
        )
    )

    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sensor_fusion'),
                'launch',
                'odom_imu_ekf.launch.py'
            )
        ),
        condition=IfCondition(use_sensor_fusion)
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ldlidar_ros2'),
                'launch',
                'ld06.launch.py'
            )
        ),
        condition=IfCondition(use_lidar)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sensor_fusion', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        robot_launch,
        sensor_fusion_launch,
        lidar_launch
    ])
