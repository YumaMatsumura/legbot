import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Set launch params
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ignition = LaunchConfiguration('use_ignition')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_use_ignition_cmd = DeclareLaunchArgument(
        'use_ignition',
        default_value='true',
        description='Use ignition gazebo if true, use gazebo if false')

    # Get the directory
    description_dir = get_package_share_directory('legbot_description')
    xacro_file = os.path.join(description_dir, 'urdf', 'legbot.urdf.xacro')
    rviz_file = os.path.join(description_dir, 'rviz', 'legbot.rviz')
    
    robot_description = Command(['xacro', ' ', xacro_file, ' use_ignition:=', use_ignition])


    # Create nodes
    load_nodes = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description}]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    output='screen',
        #    arguments=['-d', rviz_file],
        #    parameters=[{'use_sim_time': use_sim_time}])
        ])

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_ignition_cmd,
        load_nodes
        ])
