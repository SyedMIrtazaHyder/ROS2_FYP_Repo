#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using namespace std::chrono_literals;

class PlaybackNode : public rclcpp::Node
{
  public:
    PlaybackNode(const std::string & bag_filename, const std::string & topic)
    : Node("playback_node")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);

      timer_ = this->create_wall_timer(100ms,
          [this](){return this->timer_callback();}
      );

      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = bag_filename;
      reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
      reader_->open(storage_options);
	  this->topic = topic;
    }

  private:
    void timer_callback()
    {
      while (reader_->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_->read_next();

        if (msg->topic_name != this->topic) {
          continue;
        }

        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        sensor_msgs::msg::PointCloud2::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        serialization_.deserialize_message(&serialized_msg, ros_msg.get());

        publisher_->publish(*ros_msg);
		if (count == 0){
			auto print = [&](auto &n) { RCLCPP_INFO(this->get_logger(), "Name: %s\nOffset: %u\nDatatype: %u\nCount: %u",
																						n.name.c_str(), n.offset, n.datatype, n.count); };
			std::cout << '(';
			std::for_each(ros_msg->fields.begin(), ros_msg->fields.end(), print); // cause data is unbounded array which translate to vector type
			std::cout << ")\n";
			count++;
		}

        break;
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization_;
    std::unique_ptr<rosbag2_cpp::Reader> reader_;
	uint8_t count = 0;
	std::string topic;
};

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <bag>" << " <topic>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaybackNode>(argv[1], argv[2]));
  rclcpp::shutdown();

  return 0;
}
