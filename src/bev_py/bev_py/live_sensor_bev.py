import sys
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2

from ouster.sdk import client

import config.kitti_config as cnf

import json


class LiveBEV(Node):
    def __init__(self, metadata):
        super().__init__('sensor_bev_v1')
        self.boundary = {
            "minX": -25,
            "maxX": 25,
            "minY": -25,
            "maxY": 25,
            "minZ": -1.73,
            "maxZ": 2.27
        }

        self.DISCRETIZATION = (self.boundary["maxX"] - self.boundary["minX"]) / 608
    
        self.metadata, self.validator = client.parse_and_validate_metadata(metadata)
        self.xyz_LUT = client.XYZLut(self.metadata)

        self.bev_publisher = self.create_publisher(Image, 'bev_image', 10)

        self.br = CvBridge()

        self.range = Subscriber(self, Image, "/ouster/range_image")
        self.signal = Subscriber(self, Image, "/ouster/signal_image")
        self.points = Subscriber(self, PointCloud2, "/ouster/points")

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.range, self.signal, self.points],
                                                     queue_size, max_delay)
        self.time_sync.registerCallback(self.publishBEV)

    def publishBEV(self, range, sig, points):
        #xyz = self.xyz_LUT(scan.field(client.ChanField.RANGE))
        # Try to change this to /ouster/points to remove the LUT issue
        # https://github.com/ouster-lidar/ouster-ros/issues/250
        #range_img = self.br.imgmsg_to_cv2(range)
        sig_img = self.br.imgmsg_to_cv2(sig)
        #xyz = self.xyz_LUT(range_img)
        #self.get_logger().info("LUT xyz_shape: %s" % (str(xyz.shape)))
        #self.get_logger().info("type: %s" % type(xyz))
        xyz = np.array([[*point] for point in point_cloud2.read_points(points, field_names=['x', 'y', 'z'])], dtype=np.float32).reshape(1024, 64, 3).transpose(1,0,2)

        #self.get_logger().info("xyz_value: %s" % (str(xyz.shape)))
        sig_img = cv.normalize(sig_img, None, alpha=0, beta=1, norm_type=cv.NORM_MINMAX)
        #self.get_logger().info("xyz_shape: %s\nsig_img shape: %s" % (str(xyz.shape), str(sig_img.shape)))
        xyzi = np.concatenate((xyz, sig_img[..., np.newaxis]), axis=-1)
        xyzi = self.removePoints(xyzi.reshape(-1, 4), self.boundary)
        bev = self.makeBVFeature(xyzi, self.DISCRETIZATION, self.boundary)
        bev = (bev * 255).astype(np.uint8)
        bev = np.transpose(bev, (1, 2, 0))
        #bev = cv.flip(bev, -1) # Required when using the LUT table for conversion 
        bev = np.vstack((bev[304:], bev[:304])) # for getting a more centered point cloud
        self.bev_publisher.publish(self.br.cv2_to_imgmsg(bev))
    
    def callback(self):
        try:
            self.bev_publisher.publish(self.br.cv2_to_imgmsg(self.bev))
        except:
            self.get_logger().info("No image found yet")

    def removePoints(self, PointCloud, BoundaryCond):
        # Boundary condition
        minX = BoundaryCond['minX']
        maxX = BoundaryCond['maxX']
        minY = BoundaryCond['minY']
        maxY = BoundaryCond['maxY']
        minZ = BoundaryCond['minZ']
        maxZ = BoundaryCond['maxZ']

        # Remove the point out of range x,y,z
        mask = np.where((PointCloud[:, 0] >= minX) & (PointCloud[:, 0] <= maxX) & (PointCloud[:, 1] >= minY) & (
                PointCloud[:, 1] <= maxY) & (PointCloud[:, 2] >= minZ) & (PointCloud[:, 2] <= maxZ))
        PointCloud = PointCloud[mask]

        PointCloud[:, 2] = PointCloud[:, 2] - minZ

        return PointCloud


    def makeBVFeature(self, PointCloud_, Discretization, bc):
        Height = cnf.BEV_HEIGHT + 1
        Width = cnf.BEV_WIDTH + 1

        # Discretize Feature Map
        PointCloud = np.copy(PointCloud_)
        PointCloud[:, 0] = np.int_(np.floor(PointCloud[:, 0] / Discretization))
        PointCloud[:, 1] = np.int_(np.floor(PointCloud[:, 1] / Discretization) + Width / 2)

        # sort-3times
        indices = np.lexsort((-PointCloud[:, 2], PointCloud[:, 1], PointCloud[:, 0]))
        PointCloud = PointCloud[indices]

        # Height Map
        heightMap = np.zeros((Height, Width))

        _, indices = np.unique(PointCloud[:, 0:2], axis=0, return_index=True)
        PointCloud_frac = PointCloud[indices]
        # some important problem is image coordinate is (y,x), not (x,y)
        max_height = float(np.abs(bc['maxZ'] - bc['minZ']))
        heightMap[np.int_(PointCloud_frac[:, 0]), np.int_(PointCloud_frac[:, 1])] = PointCloud_frac[:, 2] / max_height

        # Intensity Map & DensityMap
        intensityMap = np.zeros((Height, Width))
        densityMap = np.zeros((Height, Width))

        _, indices, counts = np.unique(PointCloud[:, 0:2], axis=0, return_index=True, return_counts=True)
        PointCloud_top = PointCloud[indices]

        normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64))

        intensityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = PointCloud_top[:, 3]
        densityMap[np.int_(PointCloud_top[:, 0]), np.int_(PointCloud_top[:, 1])] = normalizedCounts

        RGB_Map = np.zeros((3, Height - 1, Width - 1))
        RGB_Map[2, :, :] = densityMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # r_map
        RGB_Map[1, :, :] = heightMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # g_map
        RGB_Map[0, :, :] = intensityMap[:cnf.BEV_HEIGHT, :cnf.BEV_WIDTH]  # b_map

        return RGB_Map

def main():
    rclpy.init()

    # Some random JSON parsing issue which is resolved via this
    json_path = sys.argv[1]
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)  # Load and parse the JSON file

    live_bev = LiveBEV(json.dumps(data, indent=4))

    rclpy.spin(live_bev)

    live_bev.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
