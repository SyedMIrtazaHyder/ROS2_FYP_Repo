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
		auto top_bb = visualization_msgs::msg::Marker();
		auto vertical_edges = visualization_msgs::msg::Marker();

		// top_bb attributes
		top_bb.header.frame_id = vertical_edges.header.frame_id = "/map";
		top_bb.header.stamp = vertical_edges.header.stamp = rclcpp::Clock().now();

		top_bb.ns = "top";
		vertical_edges.ns = "side";
		top_bb.id =	vertical_edges.id = 0;

		top_bb.type = visualization_msgs::msg::Marker::LINE_STRIP;
		vertical_edges.type = visualization_msgs::msg::Marker::LINE_LIST;

		top_bb.action = top_bb.action = visualization_msgs::msg::Marker::ADD;

		top_bb.pose.orientation.w = vertical_edges.pose.orientation.w = 1.0;

		top_bb.scale.x = vertical_edges.scale.x = 0.01f;

		top_bb.color.r = 1.0;
		vertical_edges.color.g = 1.0;
		//top_bb.color.g = std::rand()%10 / 10.0;
		//top_bb.color.b = std::rand()%10 / 10.0;
		top_bb.color.a = vertical_edges.color.a = 1.0;   // Don't forget to set the alpha!

		for (uint32_t i=0; i < 8; i+=2){
			geometry_msgs::msg::Point p;
			p.x = request->coords[i];
			p.y = request->coords[i+1];
			p.z = 1.0;
			RCLCPP_INFO(this->get_logger(), "%d %f %f", i, p.x, p.y);

			top_bb.points.push_back(p);
			vertical_edges.points.push_back(p);
			p.z = 0.0;
			vertical_edges.points.push_back(p);
		}

		top_bb.points.push_back(top_bb.points[0]);

		top_bb.lifetime = rclcpp::Duration::from_nanoseconds(0);
		vertical_edges.lifetime = rclcpp::Duration::from_nanoseconds(0);

		this->publisher_->publish(top_bb);
		this->publisher_->publish(vertical_edges);
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