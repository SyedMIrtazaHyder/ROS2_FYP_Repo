import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class BEVVideoRecorder(Node):
    def __init__(self):
        super().__init__('bev_video_recorder')
        self.subscription = self.create_subscription(
            Image,
            '/bev_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_size = None
        self.fps = 10 
        self.get_logger().info("Subscribed to /bev_image")

        # Ensure output directory exists
        os.makedirs("videos", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = f"videos/bev_recording_{timestamp}.avi"

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        if self.video_writer is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.video_writer = cv2.VideoWriter(
                self.output_path,
                cv2.VideoWriter_fourcc(*'MJPG'),
                self.fps,
                self.frame_size)
            self.get_logger().info(f"Started video writer: {self.output_path}")

        self.video_writer.write(frame)

    def destroy_node(self):
        if self.video_writer:
            self.video_writer.release()
            self.get_logger().info("Video file saved.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = BEVVideoRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down node (keyboard interrupt)")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
