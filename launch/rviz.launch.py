from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    my_bot = get_package_share_directory("my_bot")
    xacro_file = os.path.join(my_bot, "description", "robot.urdf.xacro")
    robot_description = xacro.process_file(xacro_file)
    
    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        output = "screen",
        parameters = [{
            "use_sim_time": use_sim_time,
            "robot_description": robot_description.toxml()
        }]
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", rviz_config],
        output = "screen"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value = "false",
            description = "Use simulation (Gazebo) clock if true"
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value = os.path.join(my_bot, "config", "view.rviz"),
            description = "Config file for Rviz2"
        ),
        robot_state_publisher_node,
        rviz_node
    ])
