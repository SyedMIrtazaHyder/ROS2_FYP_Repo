import numpy as np
import cv2 as cv
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from ouster.sdk import pcap, client
from interfaces.srv import GenerateBEV

import config.kitti_config as cnf
import data_process.kitti_bev_utils as bev_utils

# The PCAP file can be generated from the ouster-sdk python library by running ouster-cli source <sensor_name> <filename>.pcap
class BEVServer(Node):
	def __init__(self):
		super().__init__('bev_node')
		self.bev = self.create_service(GenerateBEV, 'generate_bev', self.generate_bev_callback)

	def generate_bev_callback(self, request, response):
		self.get_logger().info("Generating BEV file from source %s" % (request.source))
		self.to_bev(request.source, request.output)
		self.get_logger().info("BEV file generated at %s" % (request.output))
		response.status = "Complete"
		return response

	def to_bev(self, source, output):
		# Writes the video data in avi file
		source = pcap.PcapScanSource(source).single_source(0)
		vid_writer = cv.VideoWriter(f'{output}.avi', cv.VideoWriter_fourcc(*'MJPG'), 10, (cnf.BEV_WIDTH, cnf.BEV_HEIGHT))
		for frame in self.pcap_2_bev(source):
			vid_writer.write(frame)

	def pcap_2_bev(self, source):
		lut = client.XYZLut(source.metadata)
		bevs = []
		for scan in source:
			xyz = lut(scan.field(client.ChanField.RANGE))
			sig = scan.field(client.ChanField.SIGNAL)
			sig = cv.normalize(sig, None, alpha=0, beta=1, norm_type=cv.NORM_MINMAX)
			xyz = np.concatenate((xyz, sig[..., np.newaxis]), axis=-1)
			xyz = bev_utils.removePoints(xyz.reshape(-1, 4), cnf.boundary)
			bev = bev_utils.makeBVFeature(xyz, cnf.DISCRETIZATION, cnf.boundary).transpose(1, 2, 0)
			bev = cv.flip((bev * 255).astype(np.uint8), -1)
			bevs.append(bev)
		return bevs
	
def main():
	rclpy.init()
	bev_node = BEVServer()
	rclpy.spin(bev_node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()