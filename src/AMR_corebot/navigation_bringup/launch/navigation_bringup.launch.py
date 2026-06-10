import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

navigation_dir = get_package_share_directory('navigation_bringup')
nav2_bringup_dir = get_package_share_directory('nav2_bringup')
navigation_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
map_server_amcl_launch_file = os.path.join(navigation_dir, 'launch', 'amcl.launch.py')
navigation_params = os.path.join(navigation_dir, 'config', 'navigation.yaml')

def generate_launch_description():

    map_server_amcl_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(map_server_amcl_launch_file),
                launch_arguments={
                    'use_sim_time': 'False',
                }.items(),
            )
        ]
    )

    navigation_launch = TimerAction(
        period=3.0,
        actions=[
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(navigation_launch_file),
                        launch_arguments={
                            'params_file': navigation_params,
                            'use_sim_time': 'False',
                        }.items(),
                    )
                ]
            )
        ]
    )
    
    cmd_vel_bridge = Node(
        package='navigation_bringup',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(map_server_amcl_launch)
    ld.add_action(navigation_launch)
    ld.add_action(cmd_vel_bridge)
    return ld