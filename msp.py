from __future__ import print_function

# TODO
# Change lan and lon to Int32. That's how they're stored in the mission_step struct
from builtins import bytes # For python2/3 compatibility

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

MSP_RECVD_HEADER = Struct('preamble' / Const(MSP_PREAMBLE),
                          'direction' / Const(MSP_DIR_FROM_BOARD),
                          'size' / Int8ul,
                          'message_id' / Int8ul)

MSP_ACK = Struct('preamble' / Const(MSP_PREAMBLE),
                 'direction' / Const(MSP_DIR_FROM_BOARD),
                 'size' / Const(0, Int8ul),
                 'message_id' / Int8ul,
                 'crc' / Int8ul)


MSP_SETTINGS_PROVIDERS = {
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

MSP_REQUEST_RESPONSES = {
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

    def __init__(self, port='/dev/ttyUSB0', baudrate=MSP_SERIAL_BAUD, serial_delay=15):
        if port is not None:
            self.serial = Serial(port=port,
                                 baudrate=baudrate,
                                 timeout=MSP_SERIAL_TIMEOUT)
            print('Waiting {0} seconds for board to wake up'.format(serial_delay))
            time.sleep(serial_delay)
            print('Done waiting')

    def calc_crc(self, data):
        data = bytes(data) # for python2/3 compatibility
        crc = 0
        for a_byte in data:
            crc ^= a_byte
        return crc

    def provide(self, message_id, parameters):
        self.send(self.build(self.get_provider(message_id), parameters))

    def read_ack(self, message_id):
        ack = self.receive_data(MSP_ACK)
        if ack.message_id != message_id:
            raise ValueError("Received ACK for {0} but expected {1}".format(ack.message_id, message_id))

    def get_provider(self, message_id):
        return MSP_SETTINGS_PROVIDERS[message_id]

    def get_request(self, message_id):
        return MSP_PARAMETERIZED_REQUESTS.get(message_id,
                                             Struct('preamble' / Const(MSP_PREAMBLE),
                                                    'direction' / Const(MSP_DIR_TO_BOARD),
                                                    'size' / Const(0, Int8ul),
                                                    'message_id' / Const(message_id, Int8ul)))
    def get_response(self, message_id):
        return MSP_REQUEST_RESPONSES[message_id]

    def build(self, cmd, parameters):
        data = cmd.build(parameters)
        crc = self.calc_crc(data[MSP_DATASIZE_INDEX::])
        data += struct.pack('<i', crc) # python2/3 compatible. data += crc.to_bytes(1, byteorder='little') python3 only
        return data

    def parse(self, data, template, crc_data=True):
        try:
            parsed_data = template.parse(data)
        except:
            print("Attempted to parse", data)
            raise

        if crc_data:
            crc = self.calc_crc(data[MSP_DATASIZE_INDEX:-1])
            if (crc != parsed_data.crc):
                raise ValueError("CRC does not match. Expected {0} but got {1}. {2}".format(crc, parsed_data.crc, parsed_data))

        return parsed_data

    def send(self, data):
        self.serial.write(data)

    def receive_data(self, parser):
        received_data = self.read(parser.sizeof())
        return self.parse(received_data, parser)

    def request(self, message_id, parameters={}):
        self.send(self.build(self.get_request(message_id), parameters))
        return self.receive_data(self.get_response(message_id))

    def read(self, num_bytes):
        return self.serial.read(num_bytes)

    def close(self):
        self.serial.close()

if __name__ == '__main__':
    serial_port = "/dev/ttyACM0"

    msp = MSP(serial_port)

    print(msp.request(MSP_IDENT))
    print(msp.request(MSP_GET_WP, {'wp_no': 0}))
    print(msp.request(MSP_GET_WP, {'wp_no': 5}))

    print('Send waypoint')
    msp.provide(MSP_SET_WP,
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
    msp.read_ack(MSP_SET_WP)
    print('Get same waypoint')
    print(msp.request(MSP_GET_WP, {'wp_no': 1}))
    print(msp.request(MSP_NAV_STATUS))

    msp.close()
