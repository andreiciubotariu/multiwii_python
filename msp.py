from __future__ import print_function

from builtins import bytes # For python2/3 compatibility

from serial import Serial
from construct import Struct, Const, Int8ul, Int16ul, Int16sl, Int32ul, Int32sl
import struct
import time
import traceback

MSP_SERIAL_BAUD = 115200
MSP_SERIAL_TIMEOUT = 5

# Message IDs (commands)
# Custom commands. The behaviour of these commands is specific to my MultiWii 2.4 fork
MSP_GPS_REPORT_INTERVAL = 50
MSP_PERIODIC_GPS_REPORT = 51
MSP_SET_RC_OVERRIDES = 52
MSP_GET_RC_OVERRIDES = 53
MSP_UNSET_RC_OVERRIDES = 54

# Standard commands
MSP_IDENT = 100
MSP_SERVO = 103
MSP_MOTOR = 104
MSP_RC = 105
MSP_RAW_GPS = 106
MSP_ALTITUDE = 109
MSP_GET_WP = 118
MSP_SERVO_CONF = 120
MSP_NAV_STATUS = 121
MSP_RESET_CONF = 208
MSP_SET_WP = 209

MSP_PREAMBLE = b'$M'
MSP_DIR_FROM_BOARD = b'>'
MSP_DIR_TO_BOARD = b'<'
MSP_HAS_ERROR = b'!'
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

MSP_ERROR = Struct('preamble' / Const(MSP_PREAMBLE),
                 'direction' / Const(MSP_HAS_ERROR),
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
                        'lat' / Int32sl,
                        'lon' / Int32sl,
                        'altitude' / Int32sl,
                        'param1' / Int16ul,
                        'param2' / Int16ul,
                        'param3' / Int16ul,
                        'flag' / Int8ul),

    MSP_GPS_REPORT_INTERVAL : Struct('preamble' / Const(MSP_PREAMBLE),
                                     'direction' / Const(MSP_DIR_TO_BOARD),
                                     'size' / Const(4, Int8ul),
                                     'message_id' / Const(MSP_GPS_REPORT_INTERVAL, Int8ul),
                                     'gps_report_interval' / Int32ul),

    MSP_SET_RC_OVERRIDES : Struct('preamble' / Const(MSP_PREAMBLE),
                                  'direction' / Const(MSP_DIR_TO_BOARD),
                                  'size' / Const(1, Int8ul),
                                  'message_id' / Const(MSP_SET_RC_OVERRIDES, Int8ul),
                                  'rc_overrides' / Int8ul),

    MSP_UNSET_RC_OVERRIDES : Struct('preamble' / Const(MSP_PREAMBLE),
                                    'direction' / Const(MSP_DIR_TO_BOARD),
                                    'size' / Const(1, Int8ul),
                                    'message_id' / Const(MSP_UNSET_RC_OVERRIDES, Int8ul),
                                    'rc_overrides_to_unset' / Int8ul),

    MSP_RESET_CONF : Struct('premable' / Const(MSP_PREAMBLE),
                            'direction' / Const(MSP_DIR_TO_BOARD),
                            'size' / Const(0, Int8ul),
                            'message_id' / Const(MSP_RESET_CONF, Int8ul)),
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
                        'lat' / Int32sl,
                        'lon' / Int32sl,
                        'altitude' / Int32sl,
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
                            'target_bearing' / Int16sl,
                            'crc' / Int8ul),

    MSP_RAW_GPS : Struct('preamble' / Const(MSP_PREAMBLE),
                         'direction' / Const(MSP_DIR_FROM_BOARD),
                         'size' / Const(16, Int8ul),
                         'message_id' / Const(MSP_RAW_GPS, Int8ul),
                         'has_fix' / Int8ul,
                         'num_satellites' / Int8ul,
                         'lat' / Int32sl,
                         'lon' / Int32sl,
                         'altitude' / Int16ul,
                         'speed' / Int16ul,
                         'ground_course' / Int16ul,
                         'crc' / Int8ul),

    MSP_PERIODIC_GPS_REPORT : Struct('preamble' / Const(MSP_PREAMBLE),
                                     'direction' / Const(MSP_DIR_FROM_BOARD),
                                     'size' / Const(26, Int8ul),
                                     'message_id' / Const(MSP_PERIODIC_GPS_REPORT, Int8ul),
                                     'gps_has_fix' / Int8ul,
                                     'gps_num_satellites' / Int8ul,
                                     'gps_lat' / Int32sl,
                                     'gps_lon' / Int32sl,
                                     'gps_altitude' / Int16ul,
                                     'gps_speed' / Int16ul,
                                     'gps_ground_course' / Int16ul,
                                     'baro_estimated_alt' / Int32sl,
                                     'baro_vario' / Int16sl,
                                     'timestamp' / Int32ul,
                                     'crc' / Int8ul),

    MSP_ALTITUDE : Struct('preamble' / Const(MSP_PREAMBLE),
                          'direction' / Const(MSP_DIR_FROM_BOARD),
                          'size' / Const(6, Int8ul),
                          'message_id' / Const(MSP_ALTITUDE, Int8ul),
                          'estimated_alt' / Int32sl,
                          'vario' / Int16sl,
                          'crc' / Int8ul),

    MSP_GET_RC_OVERRIDES : Struct('preamble' / Const(MSP_PREAMBLE),
                                  'direction' / Const(MSP_DIR_FROM_BOARD),
                                  'size' / Const(1, Int8ul),
                                  'message_id' / Const(MSP_GET_RC_OVERRIDES, Int8ul),
                                  'rc_overrides' / Int8ul,
                                  'crc' / Int8ul),

    MSP_SERVO_CONF : Struct('preamble' / Const(MSP_PREAMBLE),
                            'direction' / Const(MSP_DIR_FROM_BOARD),
                            'size' / Const(56, Int8ul),
                            'message_id' / Const(MSP_SERVO_CONF, Int8ul),
                            'min0' / Int16ul,
                            'max0' / Int16ul,
                            'mid0' / Int16ul,
                            'rate0' / Int8ul,
                            'min1' / Int16ul,
                            'max1' / Int16ul,
                            'mid1' / Int16ul,
                            'rate1' / Int8ul,
                            'min2' / Int16ul,
                            'max2' / Int16ul,
                            'mid2' / Int16ul,
                            'rate2' / Int8ul,
                            'min3' / Int16ul,
                            'max3' / Int16ul,
                            'mid3' / Int16ul,
                            'rate3' / Int8ul,
                            'min4' / Int16ul,
                            'max4' / Int16ul,
                            'mid4' / Int16ul,
                            'rate4' / Int8ul,
                            'min5' / Int16ul,
                            'max5' / Int16ul,
                            'mid5' / Int16ul,
                            'rate5' / Int8ul,
                            'min6' / Int16ul,
                            'max6' / Int16ul,
                            'mid6' / Int16ul,
                            'rate6' / Int8ul,
                            'min7' / Int16ul,
                            'max7' / Int16ul,
                            'mid7' / Int16ul,
                            'rate7' / Int8ul,
                            'crc' / Int8ul),

    MSP_SERVO : Struct('preamble' / Const(MSP_PREAMBLE),
                       'direction' / Const(MSP_DIR_FROM_BOARD),
                       'size' / Const(16, Int8ul),
                       'message_id' / Const(MSP_SERVO, Int8ul),
                       'servo0' / Int16ul,
                       'servo1' / Int16ul,
                       'servo2' / Int16ul,
                       'servo3' / Int16ul,
                       'servo4' / Int16ul,
                       'servo5' / Int16ul,
                       'servo6' / Int16ul,
                       'servo7' / Int16ul,
                       'crc' / Int8ul),
    MSP_MOTOR : Struct('preamble' / Const(MSP_PREAMBLE),
                       'direction' / Const(MSP_DIR_FROM_BOARD),
                       'size' / Const(16, Int8ul),
                       'message_id' / Const(MSP_MOTOR, Int8ul),
                       'servo0' / Int16ul,
                       'servo1' / Int16ul,
                       'servo2' / Int16ul,
                       'servo3' / Int16ul,
                       'servo4' / Int16ul,
                       'servo5' / Int16ul,
                       'servo6' / Int16ul,
                       'servo7' / Int16ul,
                       'crc' / Int8ul),
    MSP_RC : Struct('preamble' / Const(MSP_PREAMBLE),
                       'direction' / Const(MSP_DIR_FROM_BOARD),
                       'size' / Const(16, Int8ul),
                       'message_id' / Const(MSP_RC, Int8ul),
                       'ROLL' / Int16ul,
                       'PITCH' / Int16ul,
                       'YAW' / Int16ul,
                       'THROTTLE' / Int16ul,
                       'AUX1' / Int16ul,
                       'AUX2' / Int16ul,
                       'AUX3' / Int16ul,
                       'AUX4' / Int16ul,
                       'crc' / Int8ul),

}

class MSP:

    def __init__(self, transport, initialization_delay=0):
        self.transport = transport
        if transport is not None and initialization_delay > 0:
            print('Waiting {0} seconds for board to wake up'.format(initialization_delay))
            time.sleep(initialization_delay)
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
        data += struct.pack('<B', crc) # python2/3 compatible. data += crc.to_bytes(1, byteorder='little') python3 only
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
        print('Sending MSP len', len(data))
        self.transport.write(data)

    def receive_data(self, parser):
        received_data = self.read(parser.sizeof())
        return self.parse(received_data, parser)

    def request(self, message_id, parameters={}):
        self.send(self.build(self.get_request(message_id), parameters))
        return self.receive_data(self.get_response(message_id))

    def read(self, num_bytes):
        return self.transport.read(num_bytes)

def stop_gps_updates(msp):
    msp.provide(MSP_GPS_REPORT_INTERVAL, {'gps_report_interval': 0})
    # Clear buffer of last GPS update

    temp = msp.read(1)
    while len(temp) > 0:
        temp = msp.read(1)


if __name__ == '__main__':
    transport = Serial(port='/dev/ttyACM0',
                       baudrate=115200,
                       timeout=5)
    msp = MSP(transport, initialization_delay=15)

    stop_gps_updates(msp)

    # msp.provide(MSP_SET_RC_OVERRIDES, {'rc_overrides' : 0})
    # msp.read_ack(MSP_SET_RC_OVERRIDES)

    # msp.provide(MSP_SET_RC_OVERRIDES, {'rc_overrides' : (1 << 1) | (1 << 5)})
    # msp.read_ack(MSP_SET_RC_OVERRIDES)
    # time.sleep(5)

    # print(msp.request(MSP_NAV_STATUS))
    # print(msp.request(MSP_RAW_GPS))
    # msp.provide(MSP_SET_WP,
    #             {
    #                 'wp_no'  : 255,
    #                 'action' : 1,
    #                 'lat' : 5,
    #                 'lon' : -5,
    #                 'altitude' : 9,
    #                 'param1' : 1,
    #                 'param2' : 5,
    #                 'param3' : 4,
    #                 'flag' : 0,
    #             })
    # msp.read_ack(MSP_SET_WP)
    while True:
        print(msp.request(MSP_NAV_STATUS))
        time.sleep(1)

    print(msp.request(MSP_GET_RC_OVERRIDES))

    transport.close()
