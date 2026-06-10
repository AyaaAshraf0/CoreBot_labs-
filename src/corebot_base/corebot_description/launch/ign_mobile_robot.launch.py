from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import  ExecuteProcess
from launch.substitutions import Command


def generate_launch_description():

    description = FindPackageShare(package="corebot_description").find("corebot_description")

    urdf_model_path = os.path.join(description, "urdf","CoreBot.urdf.xacro")

    # with open(urdf_model_path,"r") as infp:
    #     robot_description = infp.read()

    robot_description =  Command(['xacro ', urdf_model_path])  

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
         remappings=[('/clock', '/fast_clock'),] 
    )
    
    # Publish joint states if GUI is not enabled
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
        )
        
        
        # 1. Launch Gazebo Ignition Fortress with the world
    ign = ExecuteProcess(
        cmd=['ign', 'gazebo', "empty.sdf",'-r'],            
        output='screen'
    )

        # 2. Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'my_robot',
             "-topic", "robot_description",
             "-x", "0", "-y", "0", "-z","0.3",],
        parameters=[{"use_sim_time": True}],
        
        remappings=[('/clock', '/fast_clock'),]
    )
        

        # 3. Start the ROS–Ignition bridge
    bridge= Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='ros_ign_bridge',
        output='screen',
        arguments=[
            # Example bridges (add your robot’s topics)
            # '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            # "/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model",
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            # "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            # "/model/my_robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
            
        ],
        parameters=[{"use_sim_time": True}],
        remappings=[
            # ('/model/my_robot/tf', '/tf'),
                      ('/clock', '/fast_clock'),
                      
                    ],
    )
    
    clock_bridge= Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   
        ],

        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[('/clock', '/fast_clock'),] 
    )
    


    ld = LaunchDescription()
    # Add launch actions to start Gazebo, the robot state publisher, and RViz
    # ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(ign)
    ld.add_action(spawn_entity)
    ld.add_action(bridge)
    ld.add_action(clock_bridge)
    return ld
