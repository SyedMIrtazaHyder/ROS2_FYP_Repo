#include <chrono>
#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/box.hpp"

int main(int argc, char * argv[]){
	using namespace std::this_thread; 
    using namespace std::chrono; 

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
	// putting the caputed args in the request array, using first 8 as input is first 8 x,y coordinates
	// temporary setting the class and detected objects manually
	request->objects = 4;
	request->class_id = {0, 1, 2, 0};
	request->coords = {1.0, 1.0, 
	1.0, 3.5,
	2.0, 3.5,
	2.0, 1.0, // first box
	-1.0, -1.0,
	-1.0, -1.5,
	-1.5, -1.5,
	-1.5, -1.0, // second box
	3.0, 3.0,
	3.0, 4.0,
	4.0, 4.0,
	4.0, 3.0, // third box
	-3.0, -3.0,
	-4.5, -3.0,
	-4.5, -4.0,
	-3.0, -4.0}; // fourth box

	auto result_future = client->async_send_request(request);
	if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(node->get_logger(), "service call failed :(");
		client->remove_pending_request(result_future);
		return 1;
		auto result = result_future.get();
		RCLCPP_INFO(node->get_logger(), "%s", result->status.c_str());
	}


	// for reading no of args given as arguments when calling the client node
	//for (int i = 0; i < argc; i++)
	//	request->coords.push_back(atoll(argv[i + 1])); // +1 as argv starts from 1

	/*for (int i=0; i < 9; i++){
		auto result_future = client->async_send_request(request);
		request->objects = 1;
		request->class_id = {2};
		request->coords = {};
		for (int j = 0; j < 8; j++)
			request->coords.push_back(coords[i*8 + j]);
		if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(node->get_logger(), "service call failed :(");
			client->remove_pending_request(result_future);
			return 1;
		}
		auto result = result_future.get();
		//RCLCPP_INFO(node->get_logger(), "%d: %s", timer[i], result->status.c_str());

		sleep_for(milliseconds(timer[i]));
	}
	*/

	/*float coords[] = {
		0.54,  9.30,  4.54,  9.30,  4.54,  7.30,  0.54,  7.30,
		100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,
		100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,
		0.54, -3.90, 4.54, -3.90, 4.54, -5.90, 0.54, -5.90,
		100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,
		1.54,  10.30,  5.54,  10.30,  5.54,  8.30,  1.54,  8.30,
		100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,  100.0,
		2.54,  11.30,  6.54,  11.30,  6.54,  9.30,  2.54,  9.30
	};

	int timer[] = {600, 800, 7500, 1500, 2000, 2000, 3000, 2000};

	for (int i=0; i < 9; i++){
		auto result_future = client->async_send_request(request);
		request->objects = 1;
		request->class_id = {2};
		request->coords = {};
		for (int j = 0; j < 8; j++)
			request->coords.push_back(coords[i*8 + j]);
		if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
		{
			RCLCPP_ERROR(node->get_logger(), "service call failed :(");
			client->remove_pending_request(result_future);
			return 1;
		}
		auto result = result_future.get();
		//RCLCPP_INFO(node->get_logger(), "%d: %s", timer[i], result->status.c_str());

		sleep_for(milliseconds(timer[i]));
	}
	*/

	rclcpp::shutdown();
	return 0;

}
