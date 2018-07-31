#!/usr/bin/env python

import time
import signal
import dronekit
import dklib
import sys
import re

## Global Variables
altitude = 0
distances = None
_max_vel = 0.5
_last_print = 0

## Helper functions
def is_ip_addr(connection_string):
###     Used to set appropriate communication parameters
    ip_match = re.match('\d+\.\d+\.\d+\.\d+:\d+',connection_string)
    return (ip_match is not None)

def print_sensor_vals():
###     Distance values pretty printing
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
    print('Altitude: %5.2f' % altitude)
    print('')

def connect_to_dronekit(connection_string):
    if is_ip_addr(connection_string):
        vehicle = dronekit.connect(connection_string)
    else:
        # COM port telemetry is over 57600 baud
        # it also needs more time (up to 180 seconds) for initialization
        vehicle = dronekit.connect(connection_string, baud=57600, 
                heartbeat_timeout=180, wait_ready=False)
        vehicle.wait_ready(True, timeout=180)
        initialize_vehicle(vehicle)

    # Set up distance sensor listener
    @vehicle.on_message('DISTANCE_SENSOR')
    def prx_listener(self, name, message):
        global distances
        global _last_print
        distance    = min(message.max_distance,message.current_distance)
        orientation = message.orientation
        if orientation < 8:
            if distances is None:
                distances = [2 for x in range(8)]
            distances[orientation] = distance/100.0
        elif orientation == 25:
            altitude = distance/100.0

        cur_time = int(round(time.time() * 1000))
        if cur_time > (_last_print + 500):
            print_sensor_vals()
            _last_print = cur_time

    return vehicle

def initialize_vehicle(vehicle):
###     LIDAR Parameters (SERIAL2 is Port D)
    vehicle.parameters['SERIAL2_PROTOCOL'] = 11
    vehicle.parameters['SERIAL2_BAUD'] = 115
    vehicle.parameters['PRX_TYPE'] = 6         # SITL==10, Scanse==6
    vehicle.parameters['PRX_ORIENT'] = 0
    vehicle.parameters['PRX_YAW_CORR'] = 0
    # Ignore post holding up GPS unit
    vehicle.parameters['PRX_IGN_ANG1'] = 315
    vehicle.parameters['PRX_IGN_WID1'] = 5
###     Built-in collision avoidance
    vehicle.parameters['AVOID_ENABLE'] = 2
    vehicle.parameters['AVOID_MARGIN'] = 0.4
###     Enable GPS
    vehicle.parameters['SERIAL3_PROTOCOL'] = 5
    vehicle.parameters['SERIAL3_BAUD'] = 115
###     Rangefinder parameters
    vehicle.parameters['RNGFND_TYPE'] = 17     # Sharp IR I2C
    vehicle.parameters['RNGFND_ADDR'] = 102    # I2C address 0x66
    vehicle.parameters['RNGFND_SCALING'] = 1
    vehicle.parameters['RNGFND_MIN_CM'] = 10
    vehicle.parameters['RNGFND_MAX_CM'] = 80
    vehicle.parameters['RNGFND_GNDCLEAR'] = 10 # Mounted 10cm from ground

def signal_handler(signal, frame):
### Ctrl-C lands the copter and closes the port
    global vehicle
    print("Caught signal: Landing")
    dklib.send_ned_velocity(vehicle, 0,0,0)
    dklib.land_and_close(vehicle)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    if len(sys.argv) == 2:
        connection_string = sys.argv[1]
    else:
        distances = [float('inf') for x in range(8)]

###     Connect over WiFi
#        connection_string = '127.0.0.1:14551'
###     Connect over Radio
        connection_string = '/dev/ttyS4'
###     Connect to simulated copter
#        print('Please run the following commands in new windows:')
#        print('  dronekit-sitl copter')
#        print('  mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 ' + 
#                '--out 127.0.0.1:14550 --out 127.0.0.1:14551')

        raw_input('Press ENTER to continue...')

    try:
        vehicle = connect_to_dronekit(connection_string)
        dklib.arm_and_takeoff(vehicle, 2, wait=True)

###     Move forward 50cm/second for 10 seconds
#        if distances is not None:
#            for i in range(10):
#                vel = min(_max_vel, distances[0]/2.0)
#                dklib.send_ned_velocity(vehicle, vel,0,0)
#                time.sleep(1.0)

###     Stop vehicle
        dklib.send_ned_velocity(vehicle, 0,0,0)
        time.sleep(3.0)

###     Turn right
#        for i in range(10):
#            dklib.condition_yaw(vehicle, 5)
#            time.sleep(1.0)

    except Exception as e:
        print(e)
    finally:
        dklib.send_ned_velocity(vehicle, 0,0,0)
        dklib.land_and_close(vehicle)
