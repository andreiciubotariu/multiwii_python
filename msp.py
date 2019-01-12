from serial import Serial
from construct import Struct, Const, Int8ul, Int32ul
import struct
import time

MSP_SERIAL_BAUD = 115200
MSP_SERIAL_TIMEOUT = 2

# Message IDs (commands)
MSP_IDENT = 100


MSP_PREAMBLE = b'$M'
MSP_DIR_FROM_BOARD = b'>'
MSP_DIR_TO_BOARD = b'<'

MSP_CONSTRUCTS = {
	MSP_IDENT : Struct('preamble' / Const(MSP_PREAMBLE),
						  'direction' / Const(MSP_DIR_FROM_BOARD),
						  'size' / Int8ul,
						  'message_id' / Const(100, Int8ul),
						  'version' / Int8ul,
						  'multitype' / Int8ul,
						  'msp_version' / Int8ul,
						  'capability' / Int32ul,
						  'crc' / Int8ul)
}

class MSP:

	def __init__(self, port, serial_delay=15):
		self.serial = Serial(port=port, 
							 baudrate=MSP_SERIAL_BAUD,
							 timeout=MSP_SERIAL_TIMEOUT)
		print('Waiting {0} seconds for board to wakeup'.format(serial_delay))
		time.sleep(serial_delay)
		print('Done')

	def send_data(self, message_id, data):
		# TODO send data
		# TODO wait for ack back
		pass

	def send_construct(self, cmd, data):
		self.serial.write(cmd.build(data))

	def request_info(self, message_id):
		# An information request does not contain any 'data', so the CRC is simply the message_id
		request = Struct('preamble' / Const(MSP_PREAMBLE),
			  'direction' / Const(MSP_DIR_TO_BOARD),
			  'size' / Const(0, Int8ul),
			  'message_id' / Const(message_id, Int8ul),
			  'crc' / Const(message_id, Int8ul)) 
		self.send_construct(request, {});

		parser = MSP_CONSTRUCTS[message_id]
		# TODO check the received CRC and raise exception if incorrect
		return parser.parse(self.read(parser.sizeof()))

	def read(self, num_bytes):
		return self.serial.read(num_bytes)
		

serial_port = "/dev/ttyUSB0"

msp = MSP(serial_port)
print(msp.request_info(MSP_IDENT))


