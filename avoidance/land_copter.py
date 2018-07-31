#!/usr/bin/env python

# This script can be used if communication is lost with the copter, and it
# needs to land immediately

import dronekit
import dklib
import time
import sys, re, time

## Helper functions
def is_ip_addr(connection_string):
    ip_match = re.match('\d+\.\d+\.\d+\.\d+:\d+',connection_string)
    return (ip_match is not None)

def connect_to_dronekit(connection_string):
    if is_ip_addr(connection_string):
        vehicle = dronekit.connect(connection_string)
    else:
        # COM port telemetry is over 57600 baud
        vehicle = dronekit.connect(connection_string, baud=57600)
    return vehicle

if __name__ == '__main__':
    if len(sys.argv) == 2:
        connection_string = sys.argv[1]
    else:
        connection_string = '/dev/ttyS4'
#        connection_string = '127.0.0.1:14551'
    
    vehicle = connect_to_dronekit(connection_string)
    print("Connected, stopping vehicle")
    dklib.send_ned_velocity(vehicle, 0,0,0)
    time.sleep(2.0)
    print("Landing...")
    dklib.land_and_close(vehicle)
