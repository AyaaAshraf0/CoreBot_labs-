from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get full path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('corebot_description'),
        'urdf',
        'mobile_robot.urdf'
    )

    # Read the URDF contents
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([

	Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',


        ),
	
        # Robot State Publisher — publishes TFs from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],

        ),
        
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen'
        )
    ])
