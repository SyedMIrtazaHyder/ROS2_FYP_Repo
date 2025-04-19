#include <chrono>
#include <string>
#include <memory>
#include <functional>


#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "interfaces/srv/box.hpp"

// reference: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines

using namespace std::chrono_literals;


class BoundingBox : public rclcpp::Node
{
public:
	BoundingBox() : Node("bounding_box")
	{
		publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("box", 5);
		service_ = this->create_service<interfaces::srv::Box>("visualize_box",
				std::bind(&BoundingBox::box_generator, this, std::placeholders::_1, std::placeholders::_2));
				// where _1 shows the request arg and _2 shows the response args
	}
	
	void box_generator(const std::shared_ptr<interfaces::srv::Box::Request> request,
						std::shared_ptr<interfaces::srv::Box::Response> response){
		auto marker = visualization_msgs::msg::Marker();

		// marker attributes
		marker.header.frame_id = "/map";
		marker.header.stamp = rclcpp::Clock().now();

		marker.ns = "basic_shape";
		marker.id = 0;

		marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
		marker.action = visualization_msgs::msg::Marker::ADD;

		marker.pose.orientation.w = 1.0;

		marker.scale.x = 0.01f;

		marker.color.r = std::rand()%10 / 10.0;
		marker.color.g = std::rand()%10 / 10.0;
		marker.color.b = std::rand()%10 / 10.0;
		marker.color.a = 1.0;   // Don't forget to set the alpha!

		for (uint32_t i=0; i < 8; i+=2){
			geometry_msgs::msg::Point p;
			p.x = request->coords[i];
			p.y = request->coords[i+1];
			p.z = 1.0;
			RCLCPP_INFO(this->get_logger(), "%d %f %f", i, p.x, p.y);

			marker.points.push_back(p);
		}

		marker.points.push_back(marker.points[0]);

		marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

		this->publisher_->publish(marker);
		response->status = "Box Generated";
	}
private:
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
	rclcpp::Service<interfaces::srv::Box>::SharedPtr service_;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BoundingBox>());
	rclcpp::shutdown();
	return 0;
}


