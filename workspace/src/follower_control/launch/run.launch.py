import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('follower_scene')
    
    # 1. Launch: Gazebo, RViz, spawn 2 robot, bridge.
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'bringup.launch.py')
        )
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view_node',
        arguments=['/camera/image_debug'], 
        output='screen'
    )

    leader_control = Node(
        package='follower_control',
        executable='leader_control_node',
        name='leader_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    follower_control = Node(
        package='follower_control',
        executable='follower_control_node',
        name='follower_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        sim_launch,
        TimerAction(
            period=8.0,
            actions=[leader_control, follower_control],
        ),
        rqt_image_view_node,
    ])
