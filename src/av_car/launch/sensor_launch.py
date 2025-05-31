from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    av_car = FindPackageShare('av_car')
    ouster_ros = FindPackageShare('ouster_ros')

    params_file_path = PathJoinSubstitution([av_car, 'config', 'ouster_params.yaml'])
    params_file = LaunchConfiguration('params_file', default=params_file_path)
    rviz_config = LaunchConfiguration('rviz_config', default='ouster_rviz.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ouster_rviz_enable = LaunchConfiguration('viz', default='false')

    # Launching the ouster sensors, disabling rviz that is launched when we use ouster_snesor
    ouster_sensor = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ouster_ros, 'launch', 'driver.launch.py']),
                ),
            launch_arguments={'params_file': params_file, 'viz': ouster_rviz_enable}.items()
            )

    # Launching rviz with our custom configuration
    rviz2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([av_car, 'launch', 'rviz_launch.py']),
                ),
            launch_arguments={'rviz_config': rviz_config, 'use_sim_time': use_sim_time}.items()
            )

    ld.add_action(ouster_sensor)
    ld.add_action(rviz2)
    return ld

