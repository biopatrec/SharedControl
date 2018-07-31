#!/usr/bin/env python

# This is used to test the communication to ArduPilot in simulation
# DO NOT USE THIS IN A REAL VEHICLE

import socket, sys
import time, signal

TCP_IP = '127.0.0.1'
TCP_PORT = 8080
BUFFER_SIZE = 1024
FPS = 5
landed = False

REST        = '\0'
PITCH       = 'A'
YAW         = 'B'
ALT         = 'C'
ROLL        = 'D'
WAIT        = 'T'
HALT        = 'H'

REST_CMD    = bytearray(['M',0,REST,1,0])
NORTH_CMD   = bytearray(['M',0,PITCH,1,0])
SOUTH_CMD   = bytearray(['M',0,PITCH,0,0])
EAST_CMD    = bytearray(['M',0,ROLL,1,0])
WEST_CMD    = bytearray(['M',0,ROLL,0,0])
RIGHT_CMD   = bytearray(['M',0,YAW,1,0])
LEFT_CMD    = bytearray(['M',0,YAW,0,0])
UP_CMD      = bytearray(['M',0,ALT,1,0])
DOWN_CMD    = bytearray(['M',0,ALT,0,0])

def signal_handler(signal, frame):
    print("Caught signal: Closing")
    s.close()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

print("Testing Connection...")
s.send('A')
time.sleep(0.1)
s.send('C')
rsp = s.recv(1)
if rsp != 'C':
    print("Conn test failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
    s.close()
    sys.exit(1)
print("Connected!")

print("Sending Rest")
for i in range(45):
    s.sendall(REST_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp == WAIT:
        time.sleep(1)
    elif rsp == REST:
        print("System ready")
        break
    else:
        print("Init failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)

print("Moving North")
for i in range(20):
    s.sendall(NORTH_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp != PITCH:
        print("North failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

print("Moving South")
for i in range(20):
    s.sendall(SOUTH_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp != PITCH:
        print("South failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

print("Moving East")
for i in range(20):
    s.sendall(EAST_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp != ROLL:
        print("East failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

print("Moving West")
for i in range(20):
    s.sendall(WEST_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp != ROLL:
        print("West failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

print("Turning Right")
for i in range(20):
    s.sendall(RIGHT_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp != YAW:
        print("Right failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

print("Turning Left")
for i in range(20):
    s.sendall(LEFT_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp != YAW:
        print("Left failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

print("Moving Up")
for i in range(5):
    s.sendall(UP_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp != ALT:
        print("Up failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

print("Moving Down")
for i in range(60):
    s.sendall(DOWN_CMD)
    rsp = s.recv(BUFFER_SIZE)
    if rsp == HALT:
        print("System landed")
        landed = True
        break
    elif rsp != ALT:
        print("Down failure: Received: %d/0x%X" % (ord(rsp),ord(rsp)))
        s.close()
        sys.exit(1)
    time.sleep(1.0/FPS)

if landed:
    print("Success")
else:
    print("Not landed!")
s.close()
sys.exit(0)
