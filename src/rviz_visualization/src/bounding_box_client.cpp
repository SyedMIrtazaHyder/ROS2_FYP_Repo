#include <chrono>
#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/box.hpp"

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("drawer");
	auto client = node->create_client<interfaces::srv::Box>("visualize_box");

	while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
		RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
		return 1;
    }
		RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
	}

	auto request = std::make_shared<interfaces::srv::Box::Request>();
	request->coords = {1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0};
	auto result_future = client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(node->get_logger(), "service call failed :(");
		client->remove_pending_request(result_future);
		return 1;
	}
	auto result = result_future.get();
	RCLCPP_INFO(node->get_logger(), "%s", result->status.c_str());
	rclcpp::shutdown();
	return 0;

}