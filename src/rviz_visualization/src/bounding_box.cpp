#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

// reference: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines

using namespace std::chrono_literals;

class BoundingBox : public rclcpp::Node
{
public:
	BoundingBox() : Node("bounding_box")
	{
		publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("box", 5);

		auto timer_callback = 
			[this]() -> void {
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

				for (uint32_t i=0; i < 4; i++){
					geometry_msgs::msg::Point p;
					p.x = std::rand()%5 / 10.0;
					p.y = std::rand()%5 / 10.0;
					p.z = 1.0;

					marker.points.push_back(p);
				}

				marker.points.push_back(marker.points[0]);

				marker.lifetime = rclcpp::Duration::from_nanoseconds(0);

				this->publisher_->publish(marker);
			};
		timer_ = this->create_wall_timer(5s, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BoundingBox>());
	rclcpp::shutdown();
	return 0;
}


