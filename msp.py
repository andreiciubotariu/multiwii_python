from serial import Serial
from construct import Struct, Const, Int8ul, Int16ul, Int32ul
import struct
import time

MSP_SERIAL_BAUD = 115200
MSP_SERIAL_TIMEOUT = 5

# Message IDs (commands)
MSP_IDENT = 100
MSP_GET_WP = 118
MSP_NAV_STATUS = 121
MSP_SET_WP = 209


MSP_PREAMBLE = b'$M'
MSP_DIR_FROM_BOARD = b'>'
MSP_DIR_TO_BOARD = b'<'
MSP_DATASIZE_INDEX = len(MSP_PREAMBLE + MSP_DIR_TO_BOARD)

MSP_ACK = Struct('preamble' / Const(MSP_PREAMBLE),
                 'direction' / Const(MSP_DIR_FROM_BOARD),
                 'size' / Const(0, Int8ul),
                 'message_id' / Int8ul,
                 'crc' / Int8ul)


MSP_SEND_CONSTRUCTS = {
    MSP_SET_WP : Struct('preamble' / Const(MSP_PREAMBLE),
                        'direction' / Const(MSP_DIR_TO_BOARD),
                        'size' / Const(21, Int8ul),
                        'message_id' / Const(MSP_SET_WP, Int8ul),
                        'wp_no' / Int8ul,
                        'action' / Int8ul,
                        'lat' / Int32ul,
                        'lon' / Int32ul,
                        'altitude' / Int32ul,
                        'param1' / Int16ul,
                        'param2' / Int16ul,
                        'param3' / Int16ul,
                        'flag' / Int8ul),
}

MSP_PARAMETERIZED_REQUESTS = {
    MSP_GET_WP : Struct('preamble' / Const(MSP_PREAMBLE),
                        'direction' / Const(MSP_DIR_TO_BOARD),
                        'size' / Const(1, Int8ul),
                        'message_id' / Const(MSP_GET_WP, Int8ul),
                        'wp_no' / Int8ul),
}

MSP_RECEIVE_CONSTRUCTS = {
    MSP_IDENT : Struct('preamble' / Const(MSP_PREAMBLE),
                       'direction' / Const(MSP_DIR_FROM_BOARD),
                       'size' / Const(7, Int8ul),
                       'message_id' / Const(MSP_IDENT, Int8ul),
                       'version' / Int8ul,
                       'multitype' / Int8ul,
                       'msp_version' / Int8ul,
                       'capability' / Int32ul,
                       'crc' / Int8ul),
    MSP_GET_WP : Struct('preamble' / Const(MSP_PREAMBLE),
                        'direction' / Const(MSP_DIR_FROM_BOARD),
                        'size' / Const(21, Int8ul),
                        'message_id' / Const(MSP_GET_WP, Int8ul),
                        'wp_no' / Int8ul,
                        'action' / Int8ul,
                        'lat' / Int32ul,
                        'lon' / Int32ul,
                        'altitude' / Int32ul,
                        'param1' / Int16ul,
                        'param2' / Int16ul,
                        'param3' / Int16ul,
                        'flag' / Int8ul,
                        'crc' / Int8ul),
    MSP_NAV_STATUS : Struct('preamble' / Const(MSP_PREAMBLE),
                            'direction' / Const(MSP_DIR_FROM_BOARD),
                            'size' / Const(7, Int8ul),
                            'message_id' / Const(MSP_NAV_STATUS, Int8ul),
                            'gps_mode' / Int8ul,
                            'nav_state' / Int8ul,
                            'curr_mission_action' / Int8ul,
                            'curr_mission_number' / Int8ul,
                            'nav_error' / Int8ul,
                            'target_bearing' / Int16ul,
                            'crc' / Int8ul),
}

class MSP:

    def __init__(self, port, serial_delay=15):
        self.serial = Serial(port=port, 
                             baudrate=MSP_SERIAL_BAUD,
                             timeout=MSP_SERIAL_TIMEOUT)
        print('Waiting {0} seconds for board to wake up'.format(serial_delay))
        time.sleep(serial_delay)
        print('Done waiting')

    def calc_crc(self, data):
        crc = 0
        for a_byte in data:
            crc ^= a_byte
        return crc

    def send_data(self, message_id, parameters):
        self.send_construct(MSP_SEND_CONSTRUCTS[message_id], parameters);

    def read_ack(self, message_id):
        ack = self.receive_data(MSP_ACK)
        if ack.message_id != message_id:
            raise ValueError("Received ACK for {0} but expected {1}", ack.message_id, message_id)

    def send_construct(self, cmd, parameters):
        data = cmd.build(parameters)
        crc = self.calc_crc(data[MSP_DATASIZE_INDEX::])
        data += crc.to_bytes(1, byteorder='little')
        self.serial.write(data)

    def receive_data(self, parser):
        received_data = self.read(parser.sizeof())
        crc = self.calc_crc(received_data[MSP_DATASIZE_INDEX:-1])
        try:
            parsed_data = parser.parse(received_data)
        except:
            print("received_data: ", received_data)
            raise

        if (crc != parsed_data.crc):
            raise ValueError("CRC does not match. Expected {0} but got {1}. {2}".format(crc, parsed_data.crc, parsed_data))

        return parsed_data

    def request_info(self, message_id, parameters={}):
        request = MSP_PARAMETERIZED_REQUESTS.get(message_id, 
                                                 Struct('preamble' / Const(MSP_PREAMBLE),
                                                        'direction' / Const(MSP_DIR_TO_BOARD),
                                                        'size' / Const(0, Int8ul),
                                                        'message_id' / Const(message_id, Int8ul)))
        self.send_construct(request, parameters)
        return self.receive_data(MSP_RECEIVE_CONSTRUCTS[message_id])

    def read(self, num_bytes):
        return self.serial.read(num_bytes)

    def close(self):
        self.serial.close()    

serial_port = "/dev/ttyUSB0"

msp = MSP(serial_port)

print(msp.request_info(MSP_IDENT))
print(msp.request_info(MSP_GET_WP, {'wp_no': 0}))
print(msp.request_info(MSP_GET_WP, {'wp_no': 5}))

print('Send waypoint')
msp.send_data(MSP_SET_WP, 
             {
                'wp_no'  : 1,
                'action' : 1,
                'lat' : 4,
                'lon' : 57,
                'altitude' : 9,
                'param1' : 1,
                'param2' : 5,
                'param3' : 4,
                'flag' : 0,
             }) 
msp.read_ack(MSP_SET_WP);
print('Get same waypoint')
print(msp.request_info(MSP_GET_WP, {'wp_no': 1}))
print(msp.request_info(MSP_NAV_STATUS))

msp.close()
