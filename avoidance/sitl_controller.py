#!/usr/bin/env python

# This script is used to control a virtual vehicle in the SITL environment
# from BioPatRec

import dronekit
import dronekit_sitl
import socket
import signal
import sys
import math
import dklib
import pygame
from pygame.locals import *

black   = (0,0,0)
white   = (255,255,255)
red     = (232,9,9)
yellow  = (232,232,9)
green   = (29,179,56)

# Maximum vehicle values
MAX_VEL  = 4.0
VEL_INCR = 0.5
MAX_ROLL = 4.0
ROLL_INCR= 0.5
YAW_INCR = 5.0
MAX_ALT  = 0.2
ALT_INCR = 0.1
START_ALT= 1.0
LAND_ALT = 0.2

# TCP Link Parameters
TCP_IP = '127.0.0.1'
TCP_PORT = 8080
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

# Autopilot class
class Pilot:
    def __init__(self, connection_string):
        self.pitch  = 0
        self.roll   = 0
        self.conn   = connection_string
        self.alt    = 0
        self.last_update = 0
        self.start_alt = START_ALT
        try:
            self.vehicle = dronekit.connect(connection_string, wait_ready=True)
            self.status = Status.Stopped
        except:
            print("Unable to connect to ArduPilot")
            self.vehicle = None
            self.status = Status.Error

    def reconnect(self, connection_string=None):
        if connection_string:
            self.conn = connection_string
        try:
            self.vehicle = dronekit.connect(self.conn, wait_ready=True)
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
        show_feedback(d_pitch, d_roll, d_yaw, d_alt, [40 for x in range(8)])

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
            dklib.condition_yaw(self.vehicle, d_yaw)
            return SUCCESS

def send_command(mID, mDir):
    if mDir:
        direction = 1
    else:
        direction = -1
    d_vel = 0
    d_yaw = 0
    d_alt = 0
    d_roll = 0

    mID = chr(mID)
    # NED Coordinate System
    if mID == 'A':      # Pitch, Open/Close, MID 1
        d_vel = VEL_INCR * direction
    elif mID == 'B':    # Yaw, Flex/Extend, MID 2
        d_yaw = YAW_INCR * direction
    elif mID == 'C':    # Thrust, Pro/Sup, MID 3
        d_alt = ALT_INCR * (-direction)
    elif mID == 'D':    # Roll, MID 4
        d_roll = ROLL_INCR * direction
    elif mID != chr(0):
        print("ERR: Unknown command; %c/0x%x" % (mID,ord(mID)))
        return GEN_ERROR
    
    ret = pilot.move(d_vel,d_yaw,d_roll,d_alt)
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
    else:
        print("ERR: Invalid message: %d/0x%X" % (data[0],data[0]))
        retVal = GEN_ERROR

    if isinstance(retVal, basestring):
        conn.send(retVal)
    else:
        conn.send(chr(retVal))

def close_and_quit():
    if conn:
        conn.close()
    if s:
        s.close()
    pygame.quit()
    if pilot:
        pilot.stop()
    if sitl and sitl.poll() is None:
        sitl.stop()
    sys.exit(0)

def signal_handler(signal, frame):
    print("Caught signal: Closing")
    close_and_quit()

def init_pygame():
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
        pygame.draw.arc(screen, green, Rect(8,8,240,240), s_angle, e_angle, 4)

    pygame.display.flip()

signal.signal(signal.SIGINT, signal_handler)
if __name__ == "__main__":
    # Create an empty object to hold feedback variables
    feedback = type('', (), {})()
    s = None
    conn = None
    sitl = None

    try:
        # Connect to BioPatRec
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)

        # Start Dronekit SITL
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
        pilot = Pilot(connection_string)

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
    except Exception:
        print(e)
    finally:
        close_and_quit()
        
