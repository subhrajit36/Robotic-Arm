import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='final_final_design_for_urdf').find('final_final_design_for_urdf')
    urdfModelPath = os.path.join(pkgPath, 'urdf', 'final_final_design_for_urdf.urdf')

    with open(urdfModelPath, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("gazebo_ros"), '/launch', '/gazebo.launch.py']
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-file', urdfModelPath],
        output='screen'
    )

    return launch.LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity
    ])
