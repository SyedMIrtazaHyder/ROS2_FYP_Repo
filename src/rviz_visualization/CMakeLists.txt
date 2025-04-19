cmake_minimum_required(VERSION 3.8)
project(rviz_visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(bounding_box src/bounding_box.cpp)
target_link_libraries(bounding_box PUBLIC rclcpp::rclcpp ${visualization_msgs_TARGETS})

install(TARGETS
		bounding_box
		DESTINATION lib/${PROJECT_NAME}
	)
ament_package()
