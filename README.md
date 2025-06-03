# ROS2 repo for running simulations and visualizing LiDAR data
This repository only contains the ros packages used for simulation and visualization purposes for the [
LiDAR based Object Detection and Tracking](https://github.com/SyedMIrtazaHyder/LiDAR_based_Object_Detection_and_Tracking) project. Kindly review the main repository to see the full project.

- Environment: Docker [image](https://hub.docker.com/r/osrf/ros/tags)
- ROS2 Humble
- Gazebo Fortress

**NOTE**: The ros2 pipeline developed here does not work on Xavier due to docker version issues and ROS2 Humble image on Jetson ([dustynv](https://github.com/dusty-nv/jetson-containers/tree/L4T-R35.4.1)) giving issues with ultralytics installation on Jetpack 5 (Ubuntu 2020.04).

**NOTE** Docker images with network-mode host work if the link-local setting of ethernet is properly configured. If you are using DHCP server (i.e. dhcp-kea or systemd-networkd) prefer using macvlan instead of host.
## Commands
### Running Simulations
``` bash
# The sdf files are all stored in src/car_model/worlds, to add new simulation environment add it in src/car_model/worlds directory. The launch file is hardcoded to retrieve the sdf file from this path always i.e. world:=test_world.sdf will launch src/car_model/worlds/test_world.sdf file
# record tells to save the BEV video as avi file. This file can be later sent to Jetson for computing the bounding boxes.
# bag_file saves the recorded data in a bag file, if left empty it would not generate a bag file
ros2 launch car_model gazebo_model_launch world:=<path to sdf file> \
    record:=<true or false> \
    bag_file:=<name of bag file>
```

To control the car in the simulation environments use
``` bash
# Using ros2
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# using gazebo
ign topic -t "cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.1}

```
### Visualizing Live Sensor Data
``` bash
# Rviz2 with all range, signal, NIR, and reflectivity images and 3D point cloud
ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=<sensor host name>

# Loading LiDAR properties via config file, see src/av_car/config/ouster_params.yaml
ros2 launch ouster_ros driver.launch.py \
    params_file:=<path to params yaml file>

# Wrapper for above function, loads Rviz2 with the src/av_car/config/ouster_params.yaml config file
ros2 launch av_car sensor_launch

# Live Sensor visualization with BEV image
ros2 launch av_car sensor_with_bev_launch.py \
    sensor_metadata:=metadata/ouster_metadata.json \
    sensor:=true

# Recorded sensor data visualization with BEV image
ros2 launch av_car sensor_with_bev_launch.py sensor:=false \
    sensor_metadata:=metadata/ouster_metadata.json \
    pcap_file:=recordings/pcap/test_pcap.pcap \
    pcap_metadata:=recordings/pcap/test_pcap_0.json
```
### Storing Data
``` bash
# ROSBAG
ros2 bag record -ao <filename>

ros2 bag record <topic_name> -o <filename>

# Recording Live sensor data in rosbag
ros2 launch ouster_ros record.launch.xml \
    sensor_hostname:=<sensor host name> \
    bag_file:=<optional bag file name> \
    metadata:=<json file name>

# Recording in both bag and pcap file simultaneously
ros2 launch av_car recording_launch \
    record_topic:=</lidar/points or /ouster/points> \ bag_file:=<bag_filename> \
    launch_type:=<sensor or gazebo> \ sensor_hostname:=<sensor_ip> \
    pcap_file:=<pcap_file_name>

# PCAP
ouster-cli source <sensor_hostname> save <filename>.pcap
```

### Replaying Data
``` bash
# BAG
ros2 bag replay <bag_file_dir>

# PCAP
ros2 launch ouster_ros replay_pcap.launch.xml \
    pcap_file:=<path to ouster pcap file> \
    metadata:=<json file name>
```
