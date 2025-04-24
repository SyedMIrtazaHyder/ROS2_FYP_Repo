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
	BoundingBox(const std::string& frame) : Node("bounding_box")
	{
		publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("box", 5);
		service_ = this->create_service<interfaces::srv::Box>("visualize_box",
				std::bind(&BoundingBox::box_generator, this, std::placeholders::_1, std::placeholders::_2));
				// where _1 shows the request arg and _2 shows the response args
		this->frame = frame;
	}
	
	void box_generator(const std::shared_ptr<interfaces::srv::Box::Request> request,
						std::shared_ptr<interfaces::srv::Box::Response> response){
		// The ros message is structed in the following way:
		// objects: contains the total number of objects detected in one frame
		// class: an array containing each objects detection class (0 -> Pedestrian, 1-> Bike, 2 -> Car)
		// coords: a reshaped array of 8*objects which contains the four xy coordinates of the bounding box i.e. (x1, y1, x2, y2, x3, y3, x4, y4) for each object
		int8_t objects = request->objects;
		for (int8_t detection = 0; detection < objects; detection++)
		{
			auto top_bb = visualization_msgs::msg::Marker();
			auto vertical_edges = visualization_msgs::msg::Marker();

			// top_bb attributes
			top_bb.header.frame_id = vertical_edges.header.frame_id = this->frame;
			top_bb.header.stamp = vertical_edges.header.stamp = rclcpp::Clock().now();

			top_bb.ns = "top";
			vertical_edges.ns = "side";
			top_bb.id =	vertical_edges.id = detection;

			top_bb.type = visualization_msgs::msg::Marker::LINE_STRIP;
			vertical_edges.type = visualization_msgs::msg::Marker::LINE_LIST;

			top_bb.action = vertical_edges.action = visualization_msgs::msg::Marker::ADD;

			top_bb.pose.orientation.w = vertical_edges.pose.orientation.w = 1.0;

			top_bb.scale.x = vertical_edges.scale.x = 0.01f;

			// Bounding box colored based on its class
			// Pedestrian(0) = Red, Bike(1) = Green and Car(2) = Blue
			switch (request->class_id[detection])
			{
				case 0:
					top_bb.color.r = 1.0;
					vertical_edges.color.r = 1.0;
					break;

				case 1:
					top_bb.color.g = 1.0;
					vertical_edges.color.g = 1.0;
					break;

				case 2:
					top_bb.color.b = 1.0;
					vertical_edges.color.b = 1.0;
					break;

				default:
					top_bb.color.r = 0.0;
					vertical_edges.color.r = 0.0;
					break;

			}
			top_bb.color.a = vertical_edges.color.a = 1.0;   // Don't forget to set the alpha!

			for (uint8_t i=8*detection; i < 8*(detection + 1); i+=2){
				geometry_msgs::msg::Point p;
				p.x = request->coords[i];
				p.y = request->coords[i+1];
				p.z = 1.0;
				RCLCPP_INFO(this->get_logger(), "%d %d %f %f", detection, i, p.x, p.y);

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
	}
private:
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
	rclcpp::Service<interfaces::srv::Box>::SharedPtr service_;
	std::string frame;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BoundingBox>(argv[1]));
	rclcpp::shutdown();
	return 0;
}
