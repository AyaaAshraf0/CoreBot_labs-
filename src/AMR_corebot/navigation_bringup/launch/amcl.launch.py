import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node, SetRemap

from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

robot_description_pkg = get_package_share_directory("corebot_description")
navigation_pkg = get_package_share_directory("navigation_bringup")
map_yaml_file = os.path.join(
        navigation_pkg,
        "maps",
        "map.yaml"
    )


amcl_params = os.path.join(
        navigation_pkg,
        "config",
        "amcl.yaml"
    )

lifecycle_nodes = ['map_server','amcl' ]

def generate_launch_description():
    
    map_server = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{"yaml_filename":map_yaml_file,
                             "use_sim_time":False}]

    )
    
    amcl_node = Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[amcl_params, {"use_sim_time": False}],
    )
    
    lifecycle_manager = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])
    
    deactivate_map_server = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'deactivate'],
            output='screen'
        )]
    )
    
    activate_map_server = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
            output='screen'
        )]
    )
    
   
    
    ld= LaunchDescription()
    ld.add_action(map_server)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(deactivate_map_server)
    ld.add_action(activate_map_server)

    return ld