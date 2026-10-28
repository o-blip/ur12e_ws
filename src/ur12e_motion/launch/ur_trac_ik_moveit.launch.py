"""Launch the official UR MoveIt setup with TRAC-IK enabled."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Build the UR MoveIt launch description using the TRAC-IK config."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    publish_semantic = LaunchConfiguration(
        'publish_robot_description_semantic'
    )
    warehouse_path = LaunchConfiguration('warehouse_sqlite_path')

    kinematics_file = os.path.join(
        get_package_share_directory('ur12e_motion'),
        'config',
        'trac_ik_kinematics.yaml',
    )

    # Load the standard UR configuration, replacing only kinematics.yaml.
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ur',
            package_name='ur_moveit_config',
        )
        .robot_description_semantic(
            Path('srdf') / 'ur.srdf.xacro',
            # The official simulator publishes a URDF named 'ur'.
            {'name': 'ur'},
        )
        .robot_description_kinematics(kinematics_file)
        .to_moveit_configs()
    )

    warehouse_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_path,
    }

    wait_for_description = Node(
        package='ur_robot_driver',
        executable='wait_for_robot_description',
        output='screen',
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_config,
            {
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': publish_semantic,
            },
        ],
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'),
        'config',
        'moveit.rviz',
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur12e',
            description='UR robot model.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz.',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='Publish the semantic robot description.',
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='MoveIt warehouse database path.',
        ),
        wait_for_description,
    ])

    launch_description.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_for_description,
                on_exit=[move_group, rviz],
            )
        )
    )
    return launch_description
