
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.substitutions import Command


def generate_launch_description():

    description = FindPackageShare(package="corebot_description").find("corebot_description")

    corebot_bringup = FindPackageShare(package="corebot_bringup").find("corebot_bringup")   
    
    urdf_model_path = os.path.join(description, "urdf","CoreBot.urdf.xacro")
    
    controllers_config_file = os.path.join(corebot_bringup, "config", "controllers.yaml")

    # with open(urdf_model_path,"r") as infp:
    #     robot_description = infp.read()

    robot_description = ParameterValue(
        Command(['xacro ', urdf_model_path]),
        value_type=str
    )  
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
        output="screen",
        emulate_tty=True,
    )
    
    # Publish joint states if GUI is not enabled
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        emulate_tty=True,
        )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="controller_manager",
        parameters=[
            controllers_config_file,
        ],
        output="screen",
        emulate_tty=True,
        remappings=[('~/robot_description', '/robot_description'),
                    ]
    )
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster", "--controller-manager", "/controller_manager",
                   ],
        output="screen",
        emulate_tty=True,
    )       
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "--controller-manager", "/controller_manager"],
        output="screen",
        emulate_tty=True,
    ) 
    
    imu_node = Node(
        package= "corebot_hardware",
        executable= "imu_complementary_filter",
        name = "imu_complementary_filter_node",
        output="screen",
        emulate_tty=True,
    )     

    ld = LaunchDescription()
    # Add launch actions to start Gazebo, the robot state publisher, and RViz
    # ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(joint_broadcaster_spawner)
    ld.add_action(diff_drive_spawner)
    ld.add_action(imu_node)
    return ld
