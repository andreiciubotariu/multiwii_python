import sys

# import msp

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
    "MISSION_FLAG_END"         : 0xA5, # Flags that this is the last step
    # "MISSION_FLAG_CRC_ERROR"   : 0xFE, # Returned WP had an EEPROM CRC error
    "MISSION_FLAG_HOME"        : 0x01, # Returned WP is the home position
    "MISSION_FLAG_HOLD"        : 0x02, # Returned WP is the hold position
    "MISSION_FLAG_DO_LAND"     : 0x20, # Land when reached desired point (used in RTH)
    # "MISSION_FLAG_NAV_IN_PROG" : 0xff, # Navigation is in progress, returned wp is home     
}

# Indices in the waypoint string
LAT = 0
LON = 1
ACTION = 2
ALTITUDE = 3
PARAM1 = 4
PARAM2 = 5
PARAM3 = 6
FLAG = 7

MAX_WAYPOINTS = 255

if __name__ == '__main__':
    filename = 'waypoints.txt'
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    print('Opening {0}'.format(filename))
    with open(filename) as f:
        print('Reading files')
        for i in range(0,MAX_WAYPOINTS):
            line = f.readline().strip()
            if not line:
                break
            waypoint = line.split(' ')
            print(waypoint)

