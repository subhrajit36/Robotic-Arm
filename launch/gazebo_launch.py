import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.parameter_descriptions import ParameterValue
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='final_final_design_for_urdf').find('final_final_design_for_urdf')
    urdfModelPath = os.path.join(pkgPath, 'urdf', 'final_final_design_for_urdf.urdf')

    # Load the URDF file using Command (or xacro if applicable)
    robot_description = Command(['xacro ', urdfModelPath])

    # Wrap the robot_description in ParameterValue with type=str
    robot_description_param = ParameterValue(robot_description, value_type=str)

    # Parameters for robot_state_publisher
    params = {'robot_description': robot_description_param}

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("gazebo_ros"), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments={'verbose': 'true'}.items()  # Enable verbose logging for debugging
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-file', urdfModelPath],  # Use the absolute path
        output='screen'
    )

    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[params],
    )

    # ROS Control: Joint State Broadcaster
    joint_state_broadcaster_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ROS Control: Joint Trajectory Controller
    joint_trajectory_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return launch.LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity,
        controller_manager_node,
        joint_state_broadcaster_node,
        joint_trajectory_controller_node
    ])