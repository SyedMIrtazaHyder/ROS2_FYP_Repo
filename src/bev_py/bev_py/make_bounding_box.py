import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from interfaces.srv import Box
from ultralytics import YOLO
import numpy as np
import cv2
import math


class BoundingBoxClient(Node):

    def _init_(self):
        super()._init_('bounding_box_client')

        self.cli = self.create_client(Box, 'visualize_box')
	
        self.sub = self.create_subscription(Image, self.make_box, 'bev_image', 10)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = Box.Request()

        self.model = YOLO('epoch582_11s.engine')  

        self.br = CvBridge()

    def xywhr_to_corners(x, y, w, h, angle_deg):
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        # Define corners relative to center (before rotation)
        dx = w / 2
        dy = h / 2
        corners = [
            (-dx, -dy),
            (dx, -dy),
            (dx, dy),
            (-dx, dy)
        ]

        # Rotate and shift corners
        rotated = []
        for cx, cy in corners:
            rx = x + cx * cos_a - cy * sin_a
            ry = y + cx * sin_a + cy * cos_a
            rotated.extend([rx, ry])
        return rotated

    def process_detections(self, results):
        detected_classes = {}
        detected_coords = {}

        for idx, r in enumerate(results[0].boxes):

            x, y, w, h, angle = r.xywhr[0].cpu().numpy()  # Extract OBB format
            cls_id = int(r.cls)  # Class ID

            corners = self.xywhr_to_corners(x, y, w, h, angle)

            detected_classes[idx] = results[0].names[cls_id]

            # Store coordinates in x1,y1,x2,y2,x3,y3,x4,y4 format
            detected_coords[idx] = [
                (int(corners[0]), int(corners[1])),
                (int(corners[2]), int(corners[3])),
                (int(corners[4]), int(corners[5])),
                (int(corners[6]), int(corners[7]))
            ]

        return detected_classes, detected_coords

    def make_box(self, bev):
        # Convert ROS image to OpenCV format
        frame = self.br.imgmsg_to_cv2(bev)

        # Run detection
        results = self.model(frame)

        # Process results into required format
        classes_dict, coords_dict = self.process_detections(results)

        future = self.send_request(len(classes_dict), list(classes_dict.values(), list(coords_dict.values())))
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

    def send_request(self, objects, class_id, coords):
        self.req.objects = objects
        self.req.class_id = class_id
        self.req.coords = coords

        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = BoundingBoxClient()

    #future = BoundingBoxClient.send_request(
    #    int(sys.argv[1]), int(sys.argv[2]))

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '_main_':
    main()
