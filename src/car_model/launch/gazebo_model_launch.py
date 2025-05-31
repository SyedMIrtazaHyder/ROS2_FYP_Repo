from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    car_model = FindPackageShare('car_model')
    bev_py = FindPackageShare('bev_py')


    # Launching the rviz file with the model
    urdf = LaunchConfiguration('urdf', default='AV_car.xacro.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config = LaunchConfiguration('rviz_config', default='lidar_config.rviz')
    launch_jsb = LaunchConfiguration('launch_jsb', default='true')
    record = LaunchConfiguration('record', default='false')
    bag_file = LaunchConfiguration('bag_file')

    bag_file_arg = DeclareLaunchArgument('bag_file', default_value="",
                                        description="Name of the bag file")

    record_arg = DeclareLaunchArgument('record', default_value="",
                                        description="To record the bev image to later send to Xavier for processing")

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

    # Launching ros_ign_bridge
    gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="bridge_node",
            #arguments=['/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'],
            #arguments=['--ros-args', '-p', 'config_file:=/home/user/AV/src/car_model/config/ros_gz_bridge.yaml'],
            arguments=['--ros-args', '-p', ['config_file:=', PathJoinSubstitution([car_model, 'config', 'ros_gz_bridge.yaml'])]],
            output='screen'
            )

    # Generating bev image
    bev_image = Node(
            package="bev_py",
            executable="gazebo_bev",
            name="bev_image"
            )

    bev_record = Node(
            package="bev_py",
            executable="record_avi",
            name="bev_recorder",
            condition=LaunchConfigurationEquals('record', 'true')
            )

    rosbag2 = ExecuteProcess(
        cmd=[['ros2 bag record -ao ', bag_file]],
        condition=LaunchConfigurationNotEquals('bag_file', ''),
        shell=True
    )

    ld.add_action(bag_file_arg)
    ld.add_action(record_arg)
    ld.add_action(rviz_urdf_launch)
    ld.add_action(ign_gz_launch)
    ld.add_action(spawn_robot)
    ld.add_action(gz_bridge)
    ld.add_action(bev_image)
    ld.add_action(bev_record)
    ld.add_action(rosbag2)
    return ld
