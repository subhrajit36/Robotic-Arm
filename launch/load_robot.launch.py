from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare paths to URDF and configuration files
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('final_final_design_for_urdf'),
            'urdf',
            'final_final_design_for_urdf.urdf'
        ]),
        description='Path to the robot URDF file'
    )

    joint_names_config_arg = DeclareLaunchArgument(
        'joint_names_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('final_final_design_for_urdf'),
            'config',
            'joint_names_final_final_design_for_urdf.yaml'
        ]),
        description='Path to the joint names configuration file'
    )

    joint_trajectory_controller_config_arg = DeclareLaunchArgument(
        'joint_trajectory_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('final_final_design_for_urdf'),
            'config',
            'joint_trajectory_controller.yaml'
        ]),
        description='Path to the joint trajectory controller configuration file'
    )

    # Node for Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[LaunchConfiguration('joint_names_config')]
    )

    # Node for Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': LaunchConfiguration('urdf_path')}
        ]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': LaunchConfiguration('urdf_path')},
            LaunchConfiguration('joint_trajectory_config')
        ]
    )

    # Joint State Controller Spawner
    joint_state_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_controller',
        arguments=['joint_state_controller'],
        output='screen'
    )

    # Robot Arm Controller Spawner
    robot_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_robot_arm_controller',
        arguments=['robot_arm_controller'],
        output='screen'
    )

    # Hand End Effector Controller Spawner
    hand_ee_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_hand_ee_controller',
        arguments=['hand_ee_controller'],
        output='screen'
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-file', LaunchConfiguration('urdf_path')],
        output='screen'
    )


    return LaunchDescription([
        urdf_path_arg,
        joint_names_config_arg,
        joint_trajectory_controller_config_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_controller_spawner,
        robot_arm_controller_spawner,
        hand_ee_controller_spawner,
        spawn_robot_node
    ])
