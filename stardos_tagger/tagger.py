import rclpy
from rclpy.node import Node
from stardos_interfaces.msg import SensorData, Attitude, GPSPosition
import sys
import json
import time
import exiv2

class Tagger(Node):

	output_path: str
	config: str
	input: str
	
	def __init__(self):
		super().__init__('tagger')

		self.get_logger().info('initializing tagger')

		if (len(sys.argv) > 1):
			self.get_logger().info(f'loading config...')
			args = json.loads(sys.argv[1])

			self.config = args.get('config')
			self.input = args.get('input')
		else:
			self.get_logger().error(f'failure receiving config')
			sys.exit(126)


		self.get_logger().info(f'subscribing to {self.input}')

		self.subscription = self.createSubscription(
			SensorData,
			self.input,
			10)



if __name__ == '__main__':
	
	rclpy.init()

	tagger = Tagger()

	rclpy.spin(tagger)

	tagger.destroy_node()
	rclpy.shutdown()

