#!/usr/bin/env python

# This is used to control either a real or simulated ArduPilot quadcopter
# BioPatRec needs to point to 127.0.0.1 port 8080 to communicate

import dronekit
import socket
import signal
import sys
import time
import math
import dklib
import avoidance
import pygame
from pygame.locals import *

black   = (0,0,0)
white   = (255,255,255)
red     = (232,9,9)
yellow  = (232,232,9)
green   = (29,179,56)

# Maximum vehicle values
MAX_VEL  = 0.4
VEL_INCR = 0.1
MAX_ROLL = 0.4
ROLL_INCR= 0.1

PHI_INCR = 5.0
MAX_ALT  = 0.2
ALT_INCR = 0.1
START_ALT= 1.0
LAND_ALT = 0.2

# TCP Link Parameters
TCP_IP = '127.0.0.1'
TCP_PORT = 8080
#connection_string = '127.0.0.1:14551'
connection_string = '/dev/ttyS4'
BUFFER_SIZE = 20

# Error codes
GEN_ERROR   = chr(1)
SUCCESS     = chr(0)

# Command definitions
READ_SENSOR     = 'S'
MOVE_MOTOR      = 'M'
ARM_VEHICLE     = 'T'
STOP_VEHICLE    = 'H'
# BioPatRec sends the connection check in two separate packets, so we need
#  to keep track if we've received the first one
CHECK_CONN_1    = 'A'
CHECK_CONN_2    = 'C'
CHECK_CONN      = False

# Vehicle statuses
class Status:
    Error, Stopped, Arming, Ready, Finished = range(5)

# Vehicle class
class BPR_Vehicle(dronekit.Vehicle):
    def __init__(self, *args):
        super(BPR_Vehicle, self).__init__(*args)
        self._proximity = [float("infinity") for i in range(9)]

        # http://python.dronekit.io/guide/mavlink_messages.html
        @self.on_message('DISTANCE_SENSOR')
        def prx_listener(self, name, message):
            distance    = min(message.max_distance,message.current_distance)
            orientation = message.orientation
            if orientation < 8:     # ROTATION_YAW_*
                # cm to m conversion
                self._proximity[orientation] = distance/100.0
            elif orientation == 25:  # ROTATION_PITCH_270
                self._proximity[8] = distance
            self.notify_attribute_listeners('prox', self._proximity)

    @property
    def proximity(self):
        return self._proximity

# Autopilot class
class Pilot:
    def __init__(self, connection_string, avoid):
        self.pitch  = 0
        self.roll   = 0
        self.avoid  = avoid
        self.prox   = None
        self.connection_string = connection_string
        self.alt    = 0
        self.last_update = 0
        self.start_alt = START_ALT
        try:
            self.vehicle = dronekit.connect(self.connection_string, 
                    vehicle_class=BPR_Vehicle, baud=57600,
                    heartbeat_timeout=180, wait_ready=False)
            self.vehicle.wait_ready(True, timeout=180)
            self.status = Status.Stopped
            self.vehicle.add_attribute_listener('prox', self.proximity_callback)
        except:
            print("Unable to connect to ArduPilot")
            self.vehicle = None
            self.status = Status.Error

    def reconnect(self, connection_string=""):
        if connection_string:
            self.connection_string
        try:
            self.vehicle = dronekit.connect(self.connection_string, 
                    vehicle_class=BPR_Vehicle, baud=57600,
                    heartbeat_timeout=180, wait_ready=False)
            self.vehicle.wait_ready(True, timeout=180)
            self.status = Status.Stopped
        except:
            print("Unable to connect to ArduPilot")
            self.vehicle = None
            self.status = Status.Error
        return self.status

    def arm(self, tAlt=START_ALT):
        if self.status == Status.Error:
            return GEN_ERROR
        elif self.status != Status.Stopped:
            return ARM_VEHICLE

        # LIDAR Parameters (SERIAL2 is Port D)
        self.vehicle.parameters['SERIAL2_PROTOCOL'] = 11
        self.vehicle.parameters['SERIAL2_BAUD'] = 115
        self.vehicle.parameters['PRX_TYPE'] = 6         # SITL==10, Scanse==6
        self.vehicle.parameters['PRX_ORIENT'] = 0
        self.vehicle.parameters['PRX_YAW_CORR'] = 0
        self.vehicle.parameters['PRX_IGN_ANG1'] = 315
        self.vehicle.parameters['PRX_IGN_WID1'] = 5

        self.vehicle.parameters['AVOID_ENABLE'] = 2
        self.vehicle.parameters['AVOID_MARGIN'] = 0.4

        # GPS Parameters (38400 Baud)
        self.vehicle.parameters['SERIAL3_PROTOCOL'] = 5
        self.vehicle.parameters['SERIAL3_BAUD'] = 38

        # Rangefinder parameters
        self.vehicle.parameters['RNGFND_TYPE'] = 17     # Sharp IR I2C
        self.vehicle.parameters['RNGFND_ADDR'] = 102    # I2C address 0x66
        self.vehicle.parameters['RNGFND_SCALING'] = 1
        self.vehicle.parameters['RNGFND_MIN_CM'] = 10
        self.vehicle.parameters['RNGFND_MAX_CM'] = 80
        self.vehicle.parameters['RNGFND_GNDCLEAR'] = 10 # Mounted 10cm from ground
        
        dklib.arm_and_takeoff(self.vehicle, tAlt, wait=False)
        self.start_alt = tAlt
        print("Vehicle Initializing")
        return ARM_VEHICLE

    def stop(self):
        if self.status != Status.Stopped and self.status != Status.Finished:
            dklib.land_and_close(self.vehicle)
            self.status = Status.Finished
            print("Vehicle stopped")
        else:
            print("Vehicle already stopped")
        return SUCCESS

    def proximity_callback(self, vehicle, attr_name, proximity):
        self.prox = proximity[0:-1]
        self.alt = proximity[-1]

    def read_sensor(self, sensor_id):
        if not self.prox:
            return 0x00
        elif sensor_id < 8:
            return min(0xFE, self.prox[sensor_id])
        elif sensor_id == 8:
            return min(0xFE, self.alt)
        else:
            return 0xFF

    def move(self, d_pitch=0, d_yaw=0, d_roll=0, d_alt=0):
        if self.status == Status.Error:
            return GEN_ERROR
        elif self.status != Status.Ready:
            if self.vehicle.armed and self.vehicle.location.global_relative_frame.alt >= self.start_alt*0.9:
                print("Vehicle initialized")
                self.status = Status.Ready
            else:
                self.arm()
                return ARM_VEHICLE

        # Velocity ramp here (until it gets put in BioPatRec)
        if d_pitch:
            self.pitch = self.pitch + d_pitch
        elif abs(self.pitch) > VEL_INCR:
            self.pitch = self.pitch - math.copysign(VEL_INCR, self.pitch)
        if d_roll:
            self.roll = self.roll + d_roll
        elif abs(self.roll) > ROLL_INCR:
            self.roll = self.roll - math.copysign(ROLL_INCR, self.roll)
        
        # Calculate direction of motion without crashing
        if (abs(self.roll) < 1e-6) or (abs(self.pitch) < 1e-6):
            heading = 0.0
        else:
            heading = math.atan(self.roll/self.pitch)
        
        # Collision avoidance routine
        if self.prox:
            (d_phi, d_vel) = avoid.update_trajectory(heading, self.pitch, self.prox)
            d_phi = d_phi*180.0/math.pi
            self.pitch = self.pitch + d_vel
            show_feedback(d_pitch, d_roll, d_yaw, d_alt, self.prox)
        else:
            show_feedback(d_pitch, d_roll, d_yaw, d_alt, [40 for x in range(8)])
            now = int(round(time.time() * 1000))
            if now > self.last_update + 500:
                self.last_update = now
                print("No proximity data available")
            d_phi = 0

        # Ignore anything less than 1cm/second and set max threshold
        if abs(self.pitch) < 1e-3:
            self.pitch = 0
        elif abs(self.pitch) > MAX_VEL:
            self.pitch = math.copysign(MAX_VEL, self.pitch)

        if abs(self.roll) < 1e-3:
            self.roll = 0
        elif abs(self.roll) > MAX_ROLL:
            self.roll = math.copysign(MAX_ROLL, self.roll)

        # If we're close to the ground and going down, just land
        if d_alt > 0 and self.vehicle.location.global_relative_frame.alt < LAND_ALT:
            self.stop()
            return 'H'
        else:
            # Continuously update trajectory, or vehicle will stop
            dklib.send_ned_velocity(self.vehicle, self.pitch, self.roll, d_alt)
            dklib.condition_yaw(self.vehicle, d_yaw + d_phi)
            return SUCCESS

# Connect to BioPatRec
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
conn = None

# Initialize collision avoidance
avoid = avoidance.Avoidance()
avoid.max_vel       = MAX_VEL
avoid.vel_inc       = VEL_INCR
avoid.phi_inc       = PHI_INCR
avoid.safe_radius   = 0.50
avoid.copter_radius = 0.36
avoid.method        = avoidance.AVOIDANCE_CONTACT
avoid.time_rate     = 2.0
avoid.halt_radius   = 0.75
avoid.beta_1        = 40.0
avoid.beta_2        = 0.70
avoid.c_v_obs       = 8.00
avoid.update_rate   = 1.0/150   # 1/(WindowSize - Overlap)

# Create an empty object to hold necessary variables
feedback = type('', (), {})()
pilot = Pilot(connection_string, avoid)

def send_command(mID, mDir):
    if mDir:
        direction = 1
    else:
        direction = -1
    d_vel = 0
    d_phi = 0
    d_alt = 0
    d_roll = 0

    mID = chr(mID)
    # NED Coordinate System
    if mID == 'A':      # Pitch, Open/Close, MID 1
        d_vel = VEL_INCR * direction
    elif mID == 'B':    # Yaw, Flex/Extend, MID 2
        d_phi = PHI_INCR * direction
    elif mID == 'C':    # Thrust, Pro/Sup, MID 3
        d_alt = ALT_INCR * (-direction)
    elif mID == 'D':    # Roll, MID 4
        d_roll = ROLL_INCR * direction
    elif mID != chr(0):
        print("ERR: Unknown command; %c/0x%x" % (mID,ord(mID)))
        return GEN_ERROR
    
    ret = pilot.move(d_vel,d_phi,d_roll,d_alt)
    if ret != SUCCESS:
        return ret
    else:
        return mID

def parse_message(message, conn):
    global CHECK_CONN
    data = message.decode("utf-8")

    if data[0] == MOVE_MOTOR:
        mType = ord(data[1])
        mID   = ord(data[2])
        mDir  = ord(data[3])
        mPwm  = ord(data[4])
        retVal = send_command(mID, mDir)
    elif data[0] == CHECK_CONN_1 and len(data) > 1 and data[1] == CHECK_CONN_2:
        print("Received Comm Test")
        retVal = CHECK_CONN_2
    elif data[0] == CHECK_CONN_1 and CHECK_CONN == False:
        CHECK_CONN = True
        return
    elif data[0] == CHECK_CONN_2 and CHECK_CONN == True:
        print("Received Comm Test")
        CHECK_CONN = False
        retVal = CHECK_CONN_2
    elif data[0] == ARM_VEHICLE:
        retVal = pilot.arm()
    elif data[0] == STOP_VEHICLE:
        retVal = pilot.stop()
    elif data[0] == READ_SENSOR:
        conn.send(data[1])
        retVal = pilot.readSensor(ord(data[1]))
    else:
        print("ERR: Invalid message: %d/0x%X" % (data[0],data[0]))
        retVal = GEN_ERROR

    if isinstance(retVal, basestring):
        conn.send(retVal)
    else:
        conn.send(chr(retVal))

def signal_handler(signal, frame):
### Land copter and clean up on CTRL-C
    print("Caught signal: Closing")
    if conn:
        conn.close()
    pygame.quit()
    s.close()
    pilot.stop()
    sys.exit(0)

def init_pygame():
###     Set up visual feedback
    global feedback
    global screen

    pygame.init()
    screen = pygame.display.set_mode((256,256), DOUBLEBUF)

    des_arrow   = pygame.image.load("img/arrow_grey.png")
    sel_arrow   = pygame.image.load("img/arrow_red.png")
    des_turn    = pygame.image.load("img/turn_grey.png")
    sel_turn    = pygame.image.load("img/turn_red.png")
    des_up      = pygame.image.load("img/up_grey.png")
    sel_up      = pygame.image.load("img/up_red.png")
    des_down    = pygame.image.load("img/down_grey.png")
    sel_down    = pygame.image.load("img/down_red.png")
    offset      = 32

    feedback.cw_des     = des_turn
    feedback.cw_sel     = sel_turn
    feedback.ccw_des    = pygame.transform.flip(des_turn, True, False)
    feedback.ccw_sel    = pygame.transform.flip(sel_turn, True, False)
    feedback.right_des  = des_arrow
    feedback.right_sel  = sel_arrow
    feedback.left_des   = pygame.transform.rotate(des_arrow, 180)
    feedback.left_sel   = pygame.transform.rotate(sel_arrow,  180)
    feedback.fwd_des    = pygame.transform.rotate(des_arrow,  90)
    feedback.fwd_sel    = pygame.transform.rotate(sel_arrow,   90)
    feedback.back_des   = pygame.transform.rotate(des_arrow, 270)
    feedback.back_sel   = pygame.transform.rotate(sel_arrow,  270)
    feedback.up_des     = des_up
    feedback.up_sel     = sel_up
    feedback.down_des   = des_down
    feedback.down_sel   = sel_down

    feedback.cw_rect    = des_turn.get_rect().move(128+offset, 0+offset)
    feedback.ccw_rect   = des_turn.get_rect().move(0+offset, 0+offset)
    feedback.right_rect = des_arrow.get_rect().move(128+offset, 64+offset)
    feedback.left_rect  = des_arrow.get_rect().move(0+offset, 64+offset)
    feedback.fwd_rect   = des_arrow.get_rect().move(64+offset, 0+offset)
    feedback.back_rect  = des_arrow.get_rect().move(64+offset, 128+offset)
    feedback.up_rect    = des_up.get_rect().move(0+offset, 128+offset)
    feedback.down_rect  = des_up.get_rect().move(128+offset, 128+offset)
        
    screen.fill(white)
    screen.blit(feedback.back_des,  feedback.back_rect)
    screen.blit(feedback.fwd_des,   feedback.fwd_rect)
    screen.blit(feedback.right_des, feedback.right_rect)
    screen.blit(feedback.left_des,  feedback.left_rect)
    screen.blit(feedback.cw_des,    feedback.cw_rect)
    screen.blit(feedback.ccw_des,   feedback.ccw_rect)
    screen.blit(feedback.up_des,    feedback.up_rect)
    screen.blit(feedback.down_des,  feedback.down_rect)
    pygame.display.flip()


def show_feedback(d_pitch, d_roll, d_yaw, d_alt, distances):
    global screen
    screen.fill(white)
    # Change arrow color based on user input
    if d_pitch > 0:
        screen.blit(feedback.fwd_sel, feedback.fwd_rect)
        screen.blit(feedback.back_des, feedback.back_rect)
    elif d_pitch < 0:
        screen.blit(feedback.fwd_des, feedback.fwd_rect)
        screen.blit(feedback.back_sel, feedback.back_rect)
    else:
        screen.blit(feedback.fwd_des, feedback.fwd_rect)
        screen.blit(feedback.back_des, feedback.back_rect)

    if d_roll > 0:
        screen.blit(feedback.left_des, feedback.left_rect)
        screen.blit(feedback.right_sel, feedback.right_rect)
    elif d_roll < 0:
        screen.blit(feedback.left_sel, feedback.left_rect)
        screen.blit(feedback.right_des, feedback.right_rect)
    else:
        screen.blit(feedback.left_des, feedback.left_rect)
        screen.blit(feedback.right_des, feedback.right_rect)

    if d_yaw > 0:
        screen.blit(feedback.cw_sel, feedback.cw_rect)
        screen.blit(feedback.ccw_des, feedback.ccw_rect)
    elif d_yaw < 0:
        screen.blit(feedback.cw_des, feedback.cw_rect)
        screen.blit(feedback.ccw_sel, feedback.ccw_rect)
    else:
        screen.blit(feedback.cw_des, feedback.cw_rect)
        screen.blit(feedback.ccw_des, feedback.ccw_rect)

    if d_alt > 0:
        screen.blit(feedback.down_sel, feedback.down_rect)
        screen.blit(feedback.up_des, feedback.up_rect)
    elif d_alt < 0:
        screen.blit(feedback.down_des, feedback.down_rect)
        screen.blit(feedback.up_sel, feedback.up_rect)
    else:
        screen.blit(feedback.down_des, feedback.down_rect)
        screen.blit(feedback.up_des, feedback.up_rect)

    # Draw arcs based on distance to objects
    for i in range(8):
        s_angle = (2*i-1)*math.pi/8.0 - math.pi/4
        e_angle = (2*i+1)*math.pi/8.0 - math.pi/4
        if distances[i] < avoid.safe_radius:
            pygame.draw.arc(screen, red, Rect(28,28,200,200), s_angle, e_angle, 4)
        elif distances[i]/avoid.time_rate < avoid.max_vel:
            pygame.draw.arc(screen, yellow, Rect(18,18,220,220), s_angle, e_angle, 4)
        else:
            pygame.draw.arc(screen, green, Rect(8,8,240,240), s_angle, e_angle, 4)

    pygame.display.flip()


signal.signal(signal.SIGINT, signal_handler)
if __name__ == "__main__":
    try:
        print("Listening for connection")
        conn, addr = s.accept()
        print("Connected to address: ", addr)
        init_pygame()
        while pilot.status != Status.Finished:
            # Wait for data to parse
            data = conn.recv(BUFFER_SIZE)
            if not data:
                break
            parse_message(data, conn)
        print("Flight finished")
    except Exception as e:
        print(e)
    finally:
        if conn:
            conn.close()
        s.close()
        pygame.quit()
        pilot.stop()
        print("Connection closed")
        
