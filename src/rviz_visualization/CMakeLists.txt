cmake_minimum_required(VERSION 3.8)
project(rviz_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(bounding_box_server src/bounding_box_server.cpp)
target_link_libraries(bounding_box_server PUBLIC rclcpp::rclcpp ${visualization_msgs_TARGETS} ${geometry_msgs_TARGETS} ${interfaces_TARGETS})

add_executable(bounding_box_client src/bounding_box_client.cpp)
target_link_libraries(bounding_box_client PUBLIC rclcpp::rclcpp ${interfaces_TARGETS})

install(TARGETS
		bounding_box_server
		bounding_box_client
		DESTINATION lib/${PROJECT_NAME}
)

ament_package()
