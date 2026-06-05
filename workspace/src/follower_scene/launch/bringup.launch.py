import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_lab = get_package_share_directory('follower_scene')
    
    follower_xacro_file = os.path.join(pkg_lab, 'urdf', 'follower_robot.xacro') # follower
    leader_xacro_file = os.path.join(pkg_lab, 'urdf', 'leader_robot.xacro')  # leader (wwith aruco marker)

    # for using models
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_lab, 'models')
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('follower_scene'), 'worlds','map.sdf'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world_path], 
        }.items()
    )

    # 4. Node Bridge dùng chung (Xem lưu ý bên dưới về file YAML)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_lab, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # 5. Rviz2
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       parameters=[{'use_sim_time': True}],
       arguments=['-d', os.path.join(pkg_lab, 'config', 'rviz.rviz')],
    )

    # group 1: follower
    follower_group = GroupAction([
        PushRosNamespace('follower'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': Command(['xacro ', follower_xacro_file])},
                {'frame_prefix': 'follower/'}
            ],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        ),
        
        Node(
            package='ros_gz_sim', 
            executable='create', 
            arguments=[ '-name', 'follower_bot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0'], 
            output='screen'
        ),

        # Controller Manager của xe này sẽ nằm ở /follower/controller_manager
        TimerAction(period=4.0, actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/follower/controller_manager"],
                output="screen"
            )
        ]),
        TimerAction(period=6.0, actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/follower/controller_manager"],
                output="screen"
            )
        ])
    ])

    # leader
    leader_group = GroupAction([
        PushRosNamespace('leader'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': Command(['xacro ', leader_xacro_file])},
                {'frame_prefix': 'leader/'}
            ],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        ),
        
        Node(
            package='ros_gz_sim', 
            executable='create', 
            arguments=[ '-name', 'leader_bot', '-topic', 'robot_description', '-x', '2.0', '-y', '0.0', '-z', '0.0'], 
            output='screen'
        ),

        TimerAction(period=4.0, actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/leader/controller_manager"],
                output="screen"
            )
        ]),
        TimerAction(period=6.0, actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/leader/controller_manager"],
                output="screen"
            )
        ])
    ])

    # Gắn trục odom của xe follower vào world (ở tọa độ 0, 0)
    tf_world_to_follower = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'follower/odom'],
        output='screen'
    )

    # Gắn trục odom của xe leader vào world (cách 2 mét)
    tf_world_to_leader = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '2.0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'leader/odom'],
        output='screen'
    )


    return LaunchDescription([
        set_env_vars_resources,
        gz_sim,
        bridge,
        follower_group,
        leader_group,
        tf_world_to_follower, # <-- Node 1
        tf_world_to_leader,   # <-- Node 2       
        rviz,
    ])
