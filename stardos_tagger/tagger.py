import rclpy
from rclpy.node import Node
from stardos_interfaces.msg import SensorData, Attitude, GlobalPosition, SystemTime

from datetime import datetime
from collections import deque
from fractions import Fraction
import sys
import json
import time
import pyexiv2
import math

class Tagger(Node):

	input_topic: str
	output_topic = 'data'
	gps_topic = '/global_position'
	attitude_topic = '/attitude'
	time_topic = '/system_time'

	config: dict

	time_offset: int

	nspace: str
	aircraft_nspace: str

	output_pub: Publisher
	input_sub: Subscriber
	attitude_sub: Subscriber
	gps_sub: Subscriber
	time_sub: Subscriber


	# cached metadata messages in case sensor rate is slower than capture rate
	gps_msg: GPSPosition = None
	attitude_msg: Attitude = None

	# queue of stored messages from the future
	attitude_queue: deque
	gps_queue: deque
	

	def __init__(self):
		super().__init__('tagger')

		self.attitude_queue = deque()
		self.gps_queue = deque()

		self.get_logger().info('initializing tagger')

		#TODO: add proper config parsing w/ starcommand
		if (len(sys.argv) > 1):
			self.get_logger().info(f'loading config...')
			args = json.loads(sys.argv[1])

			self.config = args.get('config')
			self.input_topic = args.get('input')
		else:
			self.get_logger().error(f'failure receiving config')
			sys.exit(126)

		self.nspace = tagger.get_namespace()
        
		self.aircraft_nspace = '/'.join(list(self.nspace.split('/')[0:2]))

		self.get_logger().debug(f'{self.nspace = }')
		self.get_logger().debug(f'{self.aircraft_nspace = }')

		self.get_logger().info(f'setting up publisher on {self.nspace}{self.output_topic}')

		self.output_pub = self.create_publisher(
			SensorData,
			self.nspace + self.output_topic,
			100)

		self.get_logger().info(f'subscribing to {self.input_topic}')

		self.input_sub = self.create_subscription(
			SensorData,
			self.input_topic,
			self.tag_image,
			100)

		self.get_logger().info(f'subscribing to {self.aircraft_nspace}{self.attitude_topic}')

		self.attitude_sub = self.create_subscription(
			Attitude,
			self.aircraft_nspace + self.attitude_topic,
			self.enqueue_attitude,
			100)

		self.get_logger().info(f'subscribing to {self.aircraft_nspace}{self.gps_topic}')

		self.gps_sub = self.create_subscription(
			GPSPosition,
			self.aircraft_nspace + self.gps_topic,
			self.enqueue_gps,
			100)

		self.get_logger().info(f'subscribing to {self.aircraft_nspace}{self.time_topic}')

		self.time_sub = self.create_subscription(
			SystemTime,
			self.aircraft_nspace + self.time_topic,
			self.get_time_offset,
			1)
	

	# get next messasge from queue passed in
	def get_msg(self, timestamp: int, msg, queue: deque):
		delta = float('inf')
		
		# deque objects are falsy when empty
		while queue: 

			next_msg = queue.popleft()
			next_delta = abs((next_msg.time_boot_ms + self.time_offset) - timestamp)

			# if delta gets bigger, then current message is the closest
			# make sure to put next back
			if (next_delta > delta):
				queue.appendleft(next_msg)
				return msg
			
			msg = next_msg
			delta = next_delta

		# if the queue is empty, use the message we have
		return msg


	# subscriber method
	def enqueue_attitude(self, msg: Attitude):
		# self.get_logger().info(f'appending to attitude_queue')
		self.attitude_queue.append(msg)

		
	# get next attitude message from stored queue
	# previous message is stored in self.attitude_msg in case the queue runs dry
	def get_attitude(self, timestamp: int) -> Attitude:
		self.attitude_msg = self.get_msg(timestamp, self.attitude_msg, self.attitude_queue)

		# TODO: loop fell through, handle potential error
		if self.attitude_msg == None: 
			self.get_logger().error('attitude queue fell through')

		return self.attitude_msg


	# subscriber method
	def enqueue_gps(self, msg: GPSPosition):
		self.get_logger().info(f'appending to gps_queue')
		self.gps_queue.append(msg)


	# get next gps message from stored queue
	# previous message is stored in self.gps_msg in case the queue runs dry
	def get_gps(self, timestamp: int) -> GPSPosition:

		self.gps_msg = self.get_msg(timestamp, self.gps_msg, self.gps_queue)

		# TODO: loop fell through, handle potential error
		if self.gps_msg == None: 
			self.get_logger().error('gps queue fell through')

		return self.gps_msg


	def get_time_offset(self, msg: SystemTime):
		self.time_offset = (msg.time_unix_us / 1000) - msg.time_boot_ms
		self.get_logger().info(f'setting time offset {self.time_offset}')
		self.get_logger().info('removing system_time subscriber')
		if not self.destroy_subscription(self.time_sub):
			self.get_logger().error('failure to destroy time offset subscription')


	# these utility functions from stack overflow: 
	# https://stackoverflow.com/questions/10799366/geotagging-jpegs-with-pyexiv2
	def dms_to_decimal(self, degrees, minutes, seconds, sign=' '):
		# Convert degrees, minutes, seconds into decimal degrees.
    #
		# >>> dms_to_decimal(10, 10, 10)
		# 10.169444444444444
		# >>> dms_to_decimal(8, 9, 10, 'S')
		# -8.152777777777779

		return (-1 if sign[0] in 'SWsw' else 1) * (
			float(degrees)        +
			float(minutes) / 60   +
			float(seconds) / 3600)


	def decimal_to_dms(self, decimal):
		# Convert decimal degrees into degrees, minutes, seconds.
		#
		# >>> decimal_to_dms(50.445891)
		# [Fraction(50, 1), Fraction(26, 1), Fraction(113019, 2500)]
		# >>> decimal_to_dms(-125.976893)
		# [Fraction(125, 1), Fraction(58, 1), Fraction(92037, 2500)]
		decimal = decimal / 10000000
		remainder, degrees = math.modf(abs(decimal))
		remainder, minutes = math.modf(remainder * 60)
		return [Fraction(n).limit_denominator(10000) for n in (degrees, minutes, remainder * 60)]


	# main usage of this node: 
	# * tag images with positional metadata we're subscribed to
	# * tag images with camera parameters passed in via the config
	def tag_image(self, msg: SensorData):
		self.get_logger().info(f'tagging image {msg.content[0]}')

		metadata = pyexiv2.ImageMetadata(msg.content[0])
		metadata.read()

		metadata['Exif.Image.NewSubfileType'] = 0
		metadata['Exif.Image.Make'] = 'makename'
		metadata['Exif.Image.Model'] = 'modelname'
		metadata['Exif.Image.Orientation'] = 1
		metadata['Exif.Image.Software'] = 'USU AggieAir STARDOS'
		metadata['Exif.Photo.ExifVersion'] = '2.30'

		# TODO: perhaps set Exif.Photo.SubSecTime based off of time remainder here?
		metadata['Exif.Image.DateTime'] = datetime.fromtimestamp(msg.collected_at / 1000).strftime('%Y:%m:%d %H:%M:%S')

		# TODO: add additional image metadata & config parsing details

		# TODO: finish programmatic ref determination
		gps_msg = self.get_gps(msg.collected_at)
		metadata['Exif.GPSInfo.GPSVersionID'] = '2 2 0 0'
		metadata['Exif.GPSInfo.GPSLatitudeRef'] = 'N'
		metadata['Exif.GPSInfo.GPSLatitude'] = self.decimal_to_dms(gps_msg.lat)
		metadata['Exif.GPSInfo.GPSLongitudeRef'] = 'W'
		metadata['Exif.GPSInfo.GPSLongitude'] = self.decimal_to_dms(gps_msg.lon)
		# TODO: make sure this is encoded correctly
		# relative_alt is AGL 
		metadata['Exif.GPSInfo.GPSAltitudeRef'] = '0'
		metadata['Exif.GPSInfo.GPSAltitude'] = Fraction(gps_msg.relative_alt)
		# figure out how to get this later
		# metadata['Exif.GPSInfo.GPSDOP'] = Fraction(0,4294967295)
		
		attitude_msg = self.get_attitude(msg.collected_at)
		metadata['Xmp.DLS.Roll'] = attitude_msg.roll
		metadata['Xmp.DLS.Pitch'] = attitude_msg.pitch
		metadata['Xmp.DLS.Yaw'] = attitude_msg.yaw

		# TODO: run through config object for camera parameter names

		metadata.write()

		self.output_pub.publish(msg)


def main():
	# pyexiv2 must register XMP namespaces before using them
	# attitude metadata is not standardized, so we'll use micasense's format
	pyexiv2.xmp.register_namespace('http://micasense.com/DLS/1.0/','DLS')
	
	rclpy.init()
	tagger = Tagger()

	rclpy.spin(tagger)

	tagger.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()


