import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch the rosbag node
    car_model = FindPackageShare('car_model')

    record_topic = LaunchConfiguration('record_topic')
    bag_file = LaunchConfiguration('bag_file')
    sensor_hostname = LaunchConfiguration('sensor_hostname')
    pcap_file = LaunchConfiguration('pcap_file')

    record_topic_arg = DeclareLaunchArgument('record_topic', default_value="/lidar/points",
                                        description="Which topic we want to record on")
    bag_file_arg = DeclareLaunchArgument('bag_file', default_value="",
                                        description="Name of the bag file")
    launch_type_arg = DeclareLaunchArgument('launch_type', default_value="gazebo",
                                        description="Whether to launch from gazebo or from sensor")
    sensor_hostname_arg = DeclareLaunchArgument('sensor_hostname', default_value="169.254.166.184",
                                        description="IPv4 address of sensor")
    pcap_file_arg = DeclareLaunchArgument('pcap_file', default_value=f"{datetime.now()}_recording",
                                        description="Name of the bag file")

    # Executing commands to caputure PCAP file, we can then replay the pcap file using the command
    ouster_pcap = ExecuteProcess(
        cmd=[['ouster-cli source ', sensor_hostname, ' save ', pcap_file]],
        condition=LaunchConfigurationEquals('launch_type', 'sensor'),
        shell=True
    )

    # Launching Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([car_model, 'launch', 'gazebo_model_launch.py'])
        ),
        condition=LaunchConfigurationEquals('launch_type', 'gazebo')
    )

    # Recording in rosbag
    rosbag2 = ExecuteProcess(
        cmd=[['ros2 bag record ', record_topic, ' -o ', bag_file]],
        condition=LaunchConfigurationNotEquals('bag_file', ''),
        shell=True
    )

    return LaunchDescription([
        launch_type_arg,
        record_topic_arg,
        sensor_hostname_arg,
        bag_file_arg,
        pcap_file_arg,
        ouster_pcap,
        gazebo_launch,
        rosbag2,
    ])
