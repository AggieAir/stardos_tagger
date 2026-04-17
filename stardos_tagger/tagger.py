import rclpy
from rclpy.node import Node
from stardos_interfaces.msg import SensorData, Attitude, GlobalPosition, SystemTime, NodeHeartbeat
from pylibstardos.pipeline_node import PipelineNode
from pylibstardos.utils import BitArray

from datetime import datetime
from collections import deque
from fractions import Fraction
from signal import SIGTERM, signal
import sys
import json
import time
import math
import os
import enum
# this is referring to py3exiv2, which is different from pyexiv2 but is imported the same way
# https://python3-exiv2.readthedocs.io/en/latest/
import pyexiv2


class NodeState(enum.IntEnum):
	INITIALIZING = 0
	INIT_WAITING_FOR_TIME_OFFSET = 1
	OPERATING = 10
	STANDBY = 12


class ErrorMask(enum.IntEnum):
	ATTITUDE_QUEUE_EMPTY = 1
	GPS_QUEUE_EMPTY = 2


class Tagger(PipelineNode):

	input_topic: str
	output_topic = 'data'
	gps_topic = '/global_position'
	attitude_topic = '/attitude'
	time_topic = '/system_time'
	data_path = '/opt/stardos/tmp'

	time_offset: int = 0

	nspace: str
	aircraft_nspace: str

	output_pub = None
	input_sub = None
	attitude_sub = None
	gps_sub = None
	time_sub = None

	# cached metadata messages in case sensor rate is slower than capture rate
	gps_msg: GlobalPosition = None
	attitude_msg: Attitude = None

	# queue of stored messages from the future
	attitude_queue: deque
	gps_queue: deque
	

	def __init__(self):
		super().__init__('tagger')

		self.heartbeat_message.state = NodeState.INITIALIZING

		self.attitude_queue = deque(maxlen=1000)
		self.gps_queue = deque(maxlen=1000)

		self.get_logger().info('initializing tagger')

		self.get_logger().info(f'loading config...')

		if self.config is None: 
			self.get_logger().warn(f'no config received, using dummy parameters')
			# ezri says to set a warning bit here, defined in the node config

		self.nspace = self.get_namespace()

		self.aircraft_nspace = '/'.join(list(self.nspace.split('/')[0:2]))

		self.get_logger().debug(f'{self.nspace = }')
		self.get_logger().debug(f'{self.aircraft_nspace = }')


		self.get_logger().info(f'setting up publisher on {self.nspace}/{self.output_topic}')
		self.output_pub = self.create_publisher(
			SensorData,
			self.output_topic,
			100)

		self.attitude_topic = self.aircraft_nspace + self.attitude_topic
		self.get_logger().info(f'subscribing to {self.attitude_topic}')
		self.attitude_sub = self.create_subscription(
			Attitude,
			self.attitude_topic,
			self.enqueue_attitude,
			100)

		self.gps_topic = self.aircraft_nspace + self.gps_topic
		self.get_logger().info(f'subscribing to {self.gps_topic}')
		self.gps_sub = self.create_subscription(
		  GlobalPosition,
			self.gps_topic,
			self.enqueue_gps,
			100)

		# TODO: this may need more work. when we're testing in simulation, our clock will be accurate,
		# but in the field we may not have access to reliable time other than what we get from the gps.
		# my best effort for now is to try to calculate a time offset based what the current time is, but
		# this will probably need to be changed as we proceed with field testing.
		if (self.config['time_offset_required']):
			self.time_topic = self.aircraft_nspace + self.time_topic
			self.get_logger().info(f'subscribing to {self.time_topic}')
			self.time_sub = self.create_subscription(
				SystemTime,
				self.time_topic,
				self.get_time_offset,
				1)

			self.heartbeat_message.state = NodeState.INIT_WAITING_FOR_TIME_OFFSET
		else:
			self.heartbeat_message.state = NodeState.STANDBY

		with self.state_mutex:
			self.heartbeat_message.state = NodeState.STANDBY
			
		# self.heartbeat_timer = self.create_timer(self.heartbeat_cadence, self.heartbeat_callback)
	
	# def heartbeat_callback(self):
	# 	msg = NodeHeartbeat()
	# 	self.heartbeat_pub.publish(msg)
	# 	self.get_logger().debug(f'sending heartbeat message {msg}')

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
		if msg is None:
			self.get_logger().warn(f'received empty attitude message')
		else: 
			self.attitude_queue.append(msg)

		
	# get next attitude message from stored queue
	# previous message is stored in self.attitude_msg in case the queue runs dry
	def get_attitude(self, timestamp: int) -> Attitude:
		self.attitude_msg = self.get_msg(timestamp, self.attitude_msg, self.attitude_queue)

		# TODO: loop fell through
		# 
		if self.attitude_msg == None: 
			self.get_logger().warn('attitude queue fell through, using latest message')
			self.heartbeat_message.errors 

		return self.attitude_msg


	# subscriber method
	def enqueue_gps(self, msg: GlobalPosition):
		# self.get_logger().info(f'appending to gps_queue')
		if msg is None:
			self.get_logger().warn(f'received empty gps message')
		else:
			self.gps_queue.append(msg)


	# get next gps message from stored queue
	# previous message is stored in self.gps_msg in case the queue runs dry
	def get_gps(self, timestamp: int) -> GlobalPosition:

		self.gps_msg = self.get_msg(timestamp, self.gps_msg, self.gps_queue)

		# TODO: loop fell through
		if self.gps_msg == None: 
			self.get_logger().warn('gps queue fell through, using latest message')

		return self.gps_msg


	def get_time_offset(self, msg: SystemTime):
		self.time_offset = (msg.time_unix_us / 1000) - msg.time_boot_ms
		self.get_logger().info(f'setting time offset {self.time_offset}')
		self.get_logger().info(f'time_offset after conversion is {datetime.fromtimestamp(self.time_offset / 1000).strftime("%Y:%m:%d %H:%M:%S")}')
		self.get_logger().info('removing system_time subscriber')
		if not self.destroy_subscription(self.time_sub):
			self.get_logger().error('failure to destroy time offset subscription')

		self.heartbeat_message.state = NodeState.STANDBY


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
	def process(self, msg: SensorData):
		with self.state_mutex:
			self.heartbeat_message.state = NodeState.PRIMARY

		filename = msg.content[0].split('/')[-1]

		#TODO: check if the image actually exists here
		self.get_logger().info(f'tagging image {filename}')

		output_name = f'{self.data_out_path}/{filename}'	

		self.get_logger().info(f'moving image {msg.content[0]} to {output_name}')

		os.rename(msg.content[0], f'{output_name}')
		msg.content[0] = output_name

		metadata = pyexiv2.ImageMetadata(output_name)
		metadata.read()

		metadata['Exif.Image.NewSubfileType'] = 0
		metadata['Exif.Image.Make'] = 'USU AggieAir STARDOS'
		metadata['Exif.Image.Model'] = 'STARDOS Fake Altum PT Pancromatic'
		metadata['Exif.Image.Orientation'] = 1
		metadata['Exif.Image.Software'] = 'USU AggieAir STARDOS'
		metadata['Exif.Photo.ExifVersion'] = '2.30'

		self.get_logger().info(f'time after conversion is {datetime.fromtimestamp(msg.collected_at / 1000).strftime("%Y:%m:%d %H:%M:%S")}')

		# TODO: perhaps set Exif.Photo.SubSecTime based off of time remainder here?
		metadata['Exif.Image.DateTime'] = datetime.fromtimestamp(msg.collected_at / 1000).strftime('%Y:%m:%d %H:%M:%S')

		# TODO: add additional image metadata & config parsing details

		# TODO: finish programmatic ref determination
		gps_msg = self.get_gps(msg.collected_at)
		if gps_msg is not None:
			metadata['Exif.GPSInfo.GPSVersionID'] = '2 2 0 0'
			metadata['Exif.GPSInfo.GPSLatitudeRef'] = 'N'
			metadata['Exif.GPSInfo.GPSLatitude'] = self.decimal_to_dms(gps_msg.lat)
			metadata['Exif.GPSInfo.GPSLongitudeRef'] = 'W'
			metadata['Exif.GPSInfo.GPSLongitude'] = self.decimal_to_dms(gps_msg.lon)

			# TODO: make sure this is encoded correctly
			metadata['Exif.GPSInfo.GPSAltitudeRef'] = '0'

			# relative_alt is AGL
			# alt is MSL
			alt = gps_msg.alt

			if alt < 0:
				self.get_logger().warn(f'value out of range, skipping: {alt = }')
				metadata['Exif.GPSInfo.GPSAltitude'] = Fraction(0)
			else: 
				metadata['Exif.GPSInfo.GPSAltitude'] = Fraction(alt, 1000)

			# figure out how to get this later
			# metadata['Exif.GPSInfo.GPSDOP'] = Fraction(0,4294967295)
		else:
			self.get_logger().error('skipping gps tags')
		
		attitude_msg = self.get_attitude(msg.collected_at)
		if attitude_msg is not None:
			metadata['Xmp.Camera.Roll'] = attitude_msg.roll
			metadata['Xmp.Camera.Pitch'] = attitude_msg.pitch
			metadata['Xmp.Camera.Yaw'] = attitude_msg.yaw
		else:
			self.get_logger().error('skipping attitude tags')

		metadata['Xmp.Camera.FOV'] = '47.5 deg'
		metadata['Xmp.Camera.FocalLength35efl'] = '16.6 mm'

		# TODO: run through config object for camera parameter names

		metadata.write()

		self.output_pub.publish(msg)

		with self.state_mutex:
			self.heartbeat_message.state = NodeState.STANDBY


def main():
	# pyexiv2 must register XMP namespaces before using them
	# attitude metadata is not standardized, so we'll use micasense's format
	pyexiv2.xmp.register_namespace('http://pix4d.com/camera/1.0/','Camera')
	# ^^ this is probably outdated. we're moving to Xmp.Camera.Roll / Pitch / Yaw
	
	rclpy.init()
	tagger = Tagger()

	def sigterm(_signo, _):
		if _signo == SIGTERM:
			tagger.get_logger().info("Exiting...")
			tagger.destroy_node()
			rclpy.shutdown()
			sys.exit(0)

	signal(SIGTERM, sigterm)

	rclpy.spin(tagger)

	tagger.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()


