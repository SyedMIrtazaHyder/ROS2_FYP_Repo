from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    car_model = FindPackageShare('car_model')


    # Launching the rviz file with the model
    urdf = LaunchConfiguration('urdf', default='gz_ackermann_demo.xacro.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config = LaunchConfiguration('rviz_config', default='updated_config.rviz')
    launch_jsb = LaunchConfiguration('launch_jsb', default='true')

    rviz_urdf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([car_model, 'launch', 'rviz_launch.py'])
                ),
            launch_arguments={'rviz_config': rviz_config,'use_sim_time': use_sim_time, 'urdf': urdf}.items()
            )

    # Launching an empty gazebo gui and server
    world = LaunchConfiguration('world', default='empty.sdf')

    ign_gz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([car_model, 'launch', 'launch_gazebo.py'])
                ),
            launch_arguments={'world': world}.items()
            )

    # Launching ros_gz_sim to get information for robot_description topic
    spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/robot_description'],
            output='screen')

    # Launching ros_gz_bridge

    ld.add_action(rviz_urdf_launch)
    ld.add_action(ign_gz_launch)
    ld.add_action(spawn_robot)
    return ld
