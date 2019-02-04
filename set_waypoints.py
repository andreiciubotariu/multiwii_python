import sys

from msp import MSP, MSP_SET_WP, MSP_GET_WP
from serial import Serial

COMMENT_START_CHAR = '#'

MSP_WAYPOINT_ACTIONS = {
    "MISSION_WAYPOINT"     : 1,   # Set waypoint
    "MISSION_HOLD_UNLIM"   : 2,   # Poshold unlimited
    "MISSION_HOLD_TIME"    : 3,   # Hold for a predetermined time
    "MISSION_RTH"          : 4,   # Return to HOME
    "MISSION_SET_POI"      : 5,   # Set POINT of interest
    "MISSION_JUMP"         : 6,   # Jump to the given step (#times)
    "MISSION_SET_HEADING"  : 7,   # Set heading to a given orientation (parameter 1 is the waym 0-359 degree
    "MISSION_LAND"         : 8,   # Land at the given position
}

# Commented out flags seem to be internal to the flight controller
MSP_WAYPOINT_FLAGS = {
    "MISSION_FLAG_NONE"        : 0x00,
    "MISSION_FLAG_END"         : 0xA5, # Flags that this is the last step
    # "MISSION_FLAG_CRC_ERROR"   : 0xFE, # Returned WP had an EEPROM CRC error
    "MISSION_FLAG_HOME"        : 0x01, # Returned WP is the home position
    "MISSION_FLAG_HOLD"        : 0x02, # Returned WP is the hold position
    "MISSION_FLAG_DO_LAND"     : 0x20, # Land when reached desired point (used in RTH)
    # "MISSION_FLAG_NAV_IN_PROG" : 0xff, # Navigation is in progress, returned wp is home
}

# Indices in the waypoint string
IDX_LAT = 0
IDX_LON = 1
IDX_ACTION = 2
IDX_ALTITUDE = 3
IDX_PARAM1 = 4
IDX_PARAM2 = 5
IDX_PARAM3 = 6
IDX_FLAG = 7

MAX_WAYPOINTS = 255

def send_waypoint(protocol, params):
    protocol.provide(MSP_SET_WP, params)
    protocol.read_ack(MSP_SET_WP)

if __name__ == '__main__':
    filename = 'waypoints.txt'
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    print('Using {0}'.format(filename))
    with open(filename) as f:
        transport = Serial(port='/dev/ttyUSB0',
                       baudrate=115200,
                       timeout=5)
        protocol = MSP(transport, initialization_delay=15)
        for i in range(0,MAX_WAYPOINTS):
            wp_no = i+1

            line = f.readline().strip()
            if not line:
                break
            waypoint = line.split(' ')
            if waypoint[0].startswith(COMMENT_START_CHAR):
                continue

            print(waypoint)
            send_waypoint(protocol,
                {
                    'wp_no'  : wp_no,
                    'action' : MSP_WAYPOINT_ACTIONS[waypoint[IDX_ACTION]],
                    'lat' : int(waypoint[IDX_LAT]),
                    'lon' : int(waypoint[IDX_LON]),
                    'altitude' : int(waypoint[IDX_ALTITUDE]),
                    'param1' : int(waypoint[IDX_PARAM1]),
                    'param2' : int(waypoint[IDX_PARAM2]),
                    'param3' : int(waypoint[IDX_PARAM3]),
                    'flag' : MSP_WAYPOINT_FLAGS[waypoint[IDX_FLAG]],
                })
            print(protocol.request(MSP_GET_WP, {'wp_no': wp_no}))
