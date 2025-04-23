from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    av_car = FindPackageShare('av_car')
    bev_py = FindPackageShare('bev_py')
    ouster_ros = FindPackageShare('ouster_ros')

    metadata_file_arg = DeclareLaunchArgument('metadata', default_value='metadata/ouster_metadata.json')

    rviz_config = LaunchConfiguration('rviz_config', default='ouster_rviz_with_bev.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    metadata = LaunchConfiguration('metadata', default='metadata/ouster_metadata.json')


    params_file_path = PathJoinSubstitution([av_car, 'config', 'ouster_params.yaml'])
    params_file = LaunchConfiguration('params_file', default=params_file_path)
    ouster_rviz_enable = LaunchConfiguration('viz', default='false')


    # Launching the ouster sensors, disabling rviz that is launched when we use ouster_snesor
    ouster_sensor = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ouster_ros, 'launch', 'driver.launch.py']),
                ),
            launch_arguments={'params_file': params_file, 'viz': ouster_rviz_enable}.items()
            )

    rviz2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([av_car, 'launch', 'rviz_launch.py']),
                ),
            launch_arguments={'rviz_config': rviz_config, 'use_sim_time': use_sim_time}.items()
            )

    # Launching the live_bev node
    live_bev = Node(
        package = 'bev_py',
        executable = 'live_bev',
        arguments = [metadata],
    )

    ld.add_action(metadata_file_arg)
    ld.add_action(ouster_sensor)
    ld.add_action(rviz2)
    ld.add_action(live_bev)
    return ld