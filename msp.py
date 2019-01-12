from serial import Serial
from construct import Struct, Const, Int8ul, Int32ul
import struct
import time

MSP_SERIAL_TIMEOUT = 2


MSP_PREAMBLE = '$M'
MSP_DIRECTION_FROM_FLIGHT_CONTROLLER = '>'
MSP_DIRECTION_TO_FLIGHT_CONTROLLER = '<'


MSP_CMD_IDENTITY = Struct('preamble' / Const(b'$M'),
						  'direction' / Int8ul,
						  'size' / Int8ul,
						  'message_id' / Const(100, Int8ul),
						  'version' / Int8ul,
						  'multitype' / Int8ul,
						  'msp_version' / Int8ul,
						  'capability' / Int32ul,
						  'crc' / Int8ul)


class MSP:

	def __init__(self, port):
		self.serial = Serial(port=port, 
							 baudrate=115200,
							 timeout=MSP_SERIAL_TIMEOUT)
		time.sleep(15)

	def send_cmd(self, cmd, data):
		to_send = struct.pack('<2ssBBB', b'$M', b'<' ,0, 100, 100)
		print(to_send)
		print(struct.unpack('<2ssBBB', to_send))
		self.serial.write(to_send)

	def read(self, num_bytes):
		return self.serial.read(num_bytes)
		


print(MSP_CMD_IDENTITY.sizeof())

serial_port = "/dev/ttyUSB0"

msp = MSP(serial_port)
msp.send_cmd(0,0)

while True:
	read_bytes = msp.read(MSP_CMD_IDENTITY.sizeof())
	print(MSP_CMD_IDENTITY.parse(read_bytes));


