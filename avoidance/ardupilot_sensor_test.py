#!/usr/bin/env python

# This script just shows the distance messages sent out by MAVLink

import socket
import signal
import sys
import time
import dronekit
import dklib

MAVLINK_IP = '127.0.0.1'
MAVLINK_PORT = 14551
BUFFER_SIZE = 20

time_millis = lambda: int(round(time.time() * 1000))
copter = dronekit.connect('/dev/ttyS4', baud=57600, heartbeat_timeout=180, wait_ready=False) # Windows
copter.wait_ready(True, timeout=180)
# copter = dronekit.connect('/dev/ttyUSB0',baud=57600, heartbeat_timeout=180) # Linux

# Initialize copter
distances   = [0 for x in range(8)]
altitude    = 0.00
clearance   = 0.00
barometer   = 0.00
gps_info    = ""
last_updated = 0


# Print the sector distances every 500ms
def update_trajectory():
    global last_updated
    cur_time = time_millis()
    if cur_time > (last_updated + 1000):
        print('')
        print('      %5.2f     ' % (distances[0]))
        print('')
        print('  %5.2f   %5.2f  ' % (distances[1], distances[7]))
        print('')
        print('%5.2f       %5.2f' % (distances[2], distances[6]))
        print('')
        print('  %5.2f   %5.2f  ' % (distances[3], distances[5]))
        print('')
        print('      %5.2f      ' % (distances[4]))
        print('')
        print("Alt:  %.02f" % altitude)
        print("Clr:  %.02f" % clearance)
        print("Baro: %.02f" % barometer)
        print(gps_info)
        print("")
        last_updated = cur_time


# http://python.dronekit.io/guide/mavlink_messages.html
@copter.on_message('DISTANCE_SENSOR')
def distance_listener(self, name, message):
    global distances
    global altitude
    distance = min(message.max_distance,message.current_distance)
    orientation = message.orientation
    if orientation < 8:
        distances[orientation] = distance
    elif orientation == 25:
        altitude = distance


@copter.on_message('ALTITUDE')
def altitude_listener(self, name, message):
    global clearance
    global barometer
    clearance = message.bottom_clearance
    barometer = message.altitude_monotonic


@copter.on_message('GPS_RAW_INT')
def gps_fix_listener(self, name, message):
    global gps_info
    if message.fix_type == 0:
        gps_info = "No GPS Connected"
    elif message.fix_type == 1:
        gps_info = "No GPS Fix"
    elif message.fix_type == 2:
        gps_info = "2D Fix"
    elif message.fix_type == 3:
        gps_info = "3D Fix"
    elif message.fix_type == 4:
        gps_info = "DGPS/SBAS 3D Fix"
    elif message.fix_type == 5:
        gps_info = "RTK Float 3D Fix"
    elif message.fix_type == 6:
        gps_info = "RTX Fixed 3D Fix"
    elif message.fix_type == 7:
        gps_info = "Static Fixed (Base Station)"
    elif message.fix_type == 8:
        gps_info = "PPP 3D Fix"


def init_copter():
    print("Setting parameters\n")
    global copter
        
    # LIDAR Parameters (SERIAL2 is Port D)
    copter.parameters['SERIAL2_PROTOCOL'] = 11
    copter.parameters['SERIAL2_BAUD'] = 115
    copter.parameters['PRX_TYPE'] = 6        # SITL==10, Scanse==6
    copter.parameters['PRX_ORIENT'] = 0
    copter.parameters['PRX_YAW_CORR'] = 0
    copter.parameters['PRX_IGN_ANG1'] = 315
    copter.parameters['PRX_IGN_WID1'] = 20

    # Ardupilot built-in collision avoidance at 40cm
    copter.parameters['AVOID_ENABLE'] = 2
    copter.parameters['AVOID_MARGIN'] = 0.4

    # GPS Parameters (38400 Baud)
    copter.parameters['SERIAL3_PROTOCOL'] = 5
    copter.parameters['SERIAL3_BAUD'] = 38

    # Rangefinder parameters
    copter.parameters['RNGFND_TYPE'] = 17     # Sharp IR I2C
    copter.parameters['RNGFND_ADDR'] = 102    # I2C address 0x66
    copter.parameters['RNGFND_SCALING'] = 1
    copter.parameters['RNGFND_MIN_CM'] = 10
    copter.parameters['RNGFND_MAX_CM'] = 80
    copter.parameters['RNGFND_GNDCLEAR'] = 10 # Mounted 10cm from ground


if __name__ == "__main__":
    init_copter()
    while(True):
        update_trajectory()
