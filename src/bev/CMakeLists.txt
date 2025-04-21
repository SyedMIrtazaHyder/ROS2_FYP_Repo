cmake_minimum_required(VERSION 3.8)
project(bev)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(read_bag src/read_bag.cpp)
target_link_libraries(read_bag PUBLIC rclcpp::rclcpp rosbag2_transport::rosbag2_transport ${sensor_msgs_TARGETS})

install(TARGETS
	read_bag
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
