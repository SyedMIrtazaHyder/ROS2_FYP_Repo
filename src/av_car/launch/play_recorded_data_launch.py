from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch the rosbag node
    av_car = FindPackageShare('av_car')
    bev_py = FindPackageShare('bev_py')
    ouster_ros = FindPackageShare('ouster_ros')

    metadata_arg = DeclareLaunchArgument('metadata', default_value='',
                                         description="metadata file path")

    bev_file_arg = DeclareLaunchArgument('bev_file', default_value='',
                                         description="bev video file path")

    pcap_file = LaunchConfiguration('pcap_file')
    bev_file = LaunchConfiguration('bev_file')
    metadata = LaunchConfiguration('metadata')
    rviz_config = LaunchConfiguration('rviz_config', default='ouster_rviz_with_bev.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Launching Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([av_car, 'launch', 'rviz_launch.py'])
        ),
        launch_arguments={'rviz_config': rviz_config, 'use_sim_time': use_sim_time}.items()
    )

    # Launching the PCAP -> ROSBAG NODE
    pcap_launch = ExecuteProcess(
        cmd = [['ros2 launch ouster_ros replay_pcap.launch.xml viz:=false pcap_file:=', pcap_file, ' metadata:=', metadata]],
        shell=True
    )

    # Launching the BEV Node
    bev_node = Node(
        package="bev_py",
        executable="publish_bev",
        arguments=[bev_file],
        output='screen'
    )

    return LaunchDescription([
        metadata_arg,
        bev_file_arg,
        rviz_launch,
        pcap_launch,
        bev_node,
    ])
