import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PublishBEV(Node):
    def __init__(self, video_path):
        super().__init__('publish_bev')
        self.video = cv2.VideoCapture(video_path)

        self.pub_bev = self.create_publisher(Image, 'bev_image', 10)
        timer_period = 0.1 # for 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()

    def timer_callback(self):
        res, frame = self.video.read()
        if res == True:
            self.pub_bev.publish(self.br.cv2_to_imgmsg(frame))

def main():
  
  rclpy.init()
  publish_bev = PublishBEV(sys.argv[1])

  rclpy.spin(publish_bev)
  
  publish_bev.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()