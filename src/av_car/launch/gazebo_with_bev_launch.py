from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch the rosbag node
    av_car = FindPackageShare('av_car')
    car_model = FindPackageShare('car_model')
    bev_py = FindPackageShare('bev_py')

    bev_file_arg = DeclareLaunchArgument('bev_file', default_value='recordings/videos/san_drive_output.mp4',
                                         description="bev video file path")

    bev_file = LaunchConfiguration('bev_file', default='recordings/videos/san_drive_output.mp4')
    rviz_config = LaunchConfiguration('rviz_config', default='ouster_rviz_with_bev.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Launching Gazebo simulation
    world = LaunchConfiguration('world', default='car_world.sdf')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([car_model, 'launch', 'gazebo_model_launch.py'])
        ),
        launch_arguments={'world': world}.items()
    )

    # Launching the BEV Node
    bev_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="bev_py",
                executable="publish_bev",
                arguments=[bev_file],
                output='screen'
            )
        ]
    )

    cmd_vel_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '/cmd_vel',
            'geometry_msgs/msg/Twist',
            'linear: {x: 2.0}'
        ],
        output='screen'
    )

    bounding_box_server = Node(
        package = 'rviz_visualization',
        executable = 'bounding_box_server',
        arguments = ['/base_link'],
    )

    load_model = TimerAction(
        period=2.0,
        actions=[
            Node(
                package = 'rviz_visualization',
                executable = 'bounding_box_client'
            )
        ]
    )

    cmd_vel_kill = TimerAction(
        period=23.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '/cmd_vel',
                    'geometry_msgs/msg/Twist',
                    'linear: {x: -1.0}'
                ],
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        bev_file_arg,
        bev_node,
        cmd_vel_pub,
        bounding_box_server,
        load_model,
        cmd_vel_kill,
    ])
