#   Shubh Khandelwal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():

    my_bot = get_package_share_directory("my_bot")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(my_bot, "launch", "rviz.launch.py")]),
        launch_arguments = {"use_sim_time" : "true"}.items()
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_ros, "launch", "gazebo.launch.py")]),
    )

    spawner_node = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = [
            "-topic", "robot_description",
            "-entity", "my_bot"
        ],
        output = "screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_node,
        spawner_node,
    ])