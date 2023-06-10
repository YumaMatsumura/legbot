import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            RegisterEventHandler, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Set launch params
    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')
    declare_server_cmd = DeclareLaunchArgument(
        'server',
        default_value='true',
        description='Set to "false" not to run gzserver.')

    display_launch_path = os.path.join(
        get_package_share_directory('legbot_description'), 'launch', 'display.launch.py')
    gzserver_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
    gzclient_path = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'legbot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.5',
                   '-topic', '/robot_description'],
        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_state_broadcaster',
             '--set-state', 'active'],
        shell=True,
        output='screen')

    load_joint_group_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', 'joint_group_position_controller',
             '--set-state', 'active'],
        shell=True,
        output='screen')

    return LaunchDescription([
        declare_gui_cmd,

        declare_server_cmd,

        # ===== display launch (RViz2) ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_path),
            launch_arguments={
                'use_sim_time': 'true',
                'use_ignition': 'false'}.items()),

        # ===== Gazebo ===== #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver_path),
            condition=IfCondition(server)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_path),
            condition=IfCondition(gui)),

        spawn_entity,

        # ===== Ros2 Control ===== #
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster])),

        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_group_position_controller]))
    ])
