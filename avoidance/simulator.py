#!/usr/bin/env python

# This is a testing and data collection script used to test and optimize
# collision avoidance and user control parameters

import pygame
from pygame.locals import *
import sys
import math
import numpy as np
import avoidance
import time

# Centimerters, display is 7mx13m
S_HEIGHT = 700
S_WIDTH  = 1300
TAU      = 2*math.pi
FPS      = 20
VEL_INCR = 0.2
PHI_INCR = TAU/90
MAX_OMEGA = TAU/(4*FPS)
MAX_VEL  = 0.6
N_SENSORS = 32
RADIUS   = 32
SAFE_RADIUS = 40
FRONT_SENSOR = 0        # Index of forward-facing sensor

FONT  = None
BIG_FONT = None
white = (255,255,255)
black = (0,  0,  0)
blue  = (40, 40, 255)
red   = (255,0,  0)
green = (0,  255,0)
orange= (255,102,0)
yellow= (255,255,0)

avoid = avoidance.Avoidance()
avoid.safe_radius   = SAFE_RADIUS/100.0
avoid.copter_radius = RADIUS/100.0
#avoid.method        = avoidance.AVOIDANCE_POTENTIAL
avoid.time_rate     = 1.0
avoid.halt_radius   = 0.60
avoid.beta_1        = 40.0 # Change these!
avoid.beta_2        = 0.70
avoid.c_v_obs       = 8.0
avoid.c_v_tar       = 0.005
avoid.max_vel       = MAX_VEL
avoid.update()

# Key definitions
key_up      = K_w
key_left    = K_a
key_right   = K_d
key_down    = K_s

class Statistics:
    ## Data to collect:
    #   User Satisfaction
    #   Elapsed Time
    #   Idle Time
    #   Command changes

    def __init__(self, identifier):
        self.identifier = identifier
        self.collisions = 0
        self.elapsed_time = 0
        self.idle_time = 0
        self.command_changes = 0
        self.last_command = 0
        self.rating = -1
        self.saved_stats = False

    def collided(self):
        self.collisions = self.collisions + 1

    def command(self, key):
        if key != self.last_command:
            self.last_command = key
            self.command_changes = self.command_changes + 1
        if key == 0:
            self.idle_time = self.idle_time + (1.0/FPS)
    
    def start(self):
        self.start_time = time.time()

    def stop(self):
        self.elapsed_time = time.time() - self.start_time

    def get_stats(self):
        if self.elapsed_time == 0:
            self.stop()
        if self.rating == -1:
            print("User-reported satisfaction\n" +
                "(0 is terrible, 10 is easy and natural): ")
            rating = raw_input("")
            self.rating = rating
        return "Identifier," + str(self.identifier) + "\n" \
                "Method," + str(avoid.method) + "\n" \
                "Time (s)," + str(self.elapsed_time) + "\n" \
                "Collisions," + str(self.collisions) + "\n" \
                "Commands," + str(self.command_changes) + "\n" \
                "Idle Time (s)," + str(self.idle_time) + "\n" \
                "Satisfaction (0-10)," + str(self.rating) + "\n"

    def print_stats(self):
        print("==============")
        print(self.get_stats())
        print("==============")

    def save_stats(self):
        filename = self.identifier + '_' + str(avoid.method) + '_' + time.strftime("%d%m%y-%H%M") + '.csv'
        print(filename)
        with open(filename, 'w+') as file:
            file.write(self.get_stats())
        self.saved_stats = True
        print(filename)
        self.print_stats()

class TraceLib:
    @staticmethod
    def modulo(phi):
        phi = phi % TAU
        if phi > math.pi:
            return phi - TAU
        elif phi < -math.pi:
            return TAU + phi
        else:
            return phi

# https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
# https://stackoverflow.com/questions/4030565/line-and-line-segment-intersection
    @staticmethod
    def intersects(position, vector, wall):
        x1,y1 = position
        sx,sy = vector
        x2,y2 = (sx+x1, sy+y1)
        [x3,x4],[y3,y4] = wall.get_endpoints()
        
        # Is there any intersection?
        nx = y1-y2
        ny = x2-x1
        i1 = nx*(x3-x1) + ny*(y3-y1) # n*(p1-p)
        i2 = nx*(x4-x1) + ny*(y4-y1) # n*(p2-p)
        if np.sign(i1) == np.sign(i2):
            return False, float('inf')

        # If so, calculate the distance
        denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
        Px = (((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / denom) - x1
        Py = (((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)) / denom) - y1
        
        # Does this vector point in the right direction?
        if (sx == 0 or np.sign(Px) == np.sign(sx)) and (sy == 0 or np.sign(Py) == np.sign(sy)):
            return True, math.sqrt(Px*Px+Py*Py)
        else:
            return False, float('inf')

class Vehicle:
    def __init__(self, radius, safe_radius, n_sensors):
        self.pos_x = 100
        self.pos_y = 100
        self.vel = 0            # Actual velocity
        self.phi = 0            # Absolute heading angle
        self.d_vel = 0
        self.d_phi = 0
        self.accel = 0
        self.omega = 0
        self.rect = Rect(self.pos_x-radius, self.pos_y-radius, 
                         radius*2, radius*2)
        self.collided = False
        self.finished = False
        self.N_SENSORS = n_sensors
        self.RADIUS = radius    # Radius of vehicle
        self.M_RADIUS = float(radius)/100.0
        self.SAFE_RADIUS = safe_radius
        self.readings = []      # Distance sensor readings
        self.sensors = []       # Angle of sensors from heading
        for n in range(self.N_SENSORS):
            self.readings.append(float("inf"))
            self.sensors.append(n*TAU/self.N_SENSORS)

    def user_control(self):
        if self.collided or self.finished:
            return

        keys = pygame.key.get_pressed()

        if keys[key_down]:
            stats.command(keys[key_down])
            if (self.vel-VEL_INCR) > -MAX_VEL:
                self.d_vel = -VEL_INCR
            else:
                self.d_vel = 0
            self.d_phi = 0
        elif keys[key_up]:
            stats.command(keys[key_up])
            if (self.vel+VEL_INCR) < MAX_VEL:
                self.d_vel = +VEL_INCR
            else:
                self.d_vel = 0
            self.d_phi = 0
        elif keys[key_right]:
            stats.command(keys[key_right])
            self.d_phi = +PHI_INCR
            if avoid.method == avoidance.AVOIDANCE_HALT:
                self.d_vel = -self.vel
            else:
                self.d_vel = 0
        elif keys[key_left]:
            stats.command(keys[key_left])
            self.d_phi = -PHI_INCR
            if avoid.method == avoidance.AVOIDANCE_HALT:
                self.d_vel = -self.vel
            else:
                self.d_vel = 0
        else:
            if avoid.method == avoidance.AVOIDANCE_HALT:
                self.d_phi = 0
                self.d_vel = -self.vel
            else:
                self.d_phi = 0
                self.d_vel = 0
                stats.command(0)


    def update(self, screen, obstacles):
        if not (self.collided or self.finished):
            self.update_sensors(obstacles)
            self.update_trajectory()
            # Stop the simulation if there's a collision
            if any(dist < self.M_RADIUS for dist in self.readings):
                #self.collided = True
                stats.collisions = stats.collisions + 1
        self.redraw(screen)
    
    def get_sensor_vector(self, sensorIdx):
        if sensorIdx < self.N_SENSORS:
            vx = math.cos(self.phi + self.sensors[sensorIdx])
            if abs(vx) < 1e-6:
                vx = 0
            vy = math.sin(self.phi + self.sensors[sensorIdx])
            if abs(vy) < 1e-6:
                vy = 0
            return (vx,vy)
        else:
            print("Invalid sensor index %d" % sensorIdx)
            return (0,0)

    def update_sensors(self, obstacles):
        # Clear previous values
        self.readings = [float("inf") for x in range(self.N_SENSORS)]

        # Find closest obstacle seen by each sensor
        for obstacle in obstacles:
            for wall in obstacle.get_walls():
                for sensor in range(self.N_SENSORS):
                    intersects, distance = TraceLib.intersects(
                            (self.pos_x,self.pos_y), 
                            self.get_sensor_vector(sensor), 
                            wall)
                    distance = distance / 100.0
                    self.readings[sensor] = min(self.readings[sensor], distance)

    def update_trajectory(self):
        (d_heading, d_velocity) = avoid.update_trajectory(
                0.0, self.vel+self.d_vel, 
                avoid.convert(self.readings, N_SENSORS))

        self.accel = (self.accel*0.6) + (d_velocity+self.d_vel)*0.4
        self.vel = self.vel + self.accel
        if abs(self.vel) < VEL_INCR/10:
            self.vel = 0
            self.omega = (self.omega*0.6) + (self.d_phi)*0.4
        else:
            self.omega = (self.omega*0.6) + (d_heading+self.d_phi)*0.4
        self.omega = math.copysign(min(abs(self.omega), MAX_OMEGA), self.omega)
        self.phi = self.phi + self.omega

        # Update position
        if np.isfinite(self.vel) and np.isfinite(self.phi):
            vel_px = self.vel * 10.0 # Convert to pixels/second
            self.pos_x = self.pos_x + vel_px*math.cos(self.phi)
            self.pos_y = self.pos_y + vel_px*math.sin(self.phi)

        # Enforce bounding boxes (just in case)
        if self.pos_x < 0:
            self.pos_x = 0
        elif self.pos_x > S_WIDTH:
            self.pos_x = S_WIDTH
        if self.pos_y < 0:
            self.pos_y = 0
        elif self.pos_y > S_HEIGHT:
            self.pos_y = S_HEIGHT

        self.rect.center = (self.pos_x, self.pos_y)

    def redraw(self, screen):
        pygame.draw.circle(screen, white, (int(self.pos_x),int(self.pos_y)), self.RADIUS, 0)
        pygame.draw.circle(screen, yellow, (int(self.pos_x),int(self.pos_y)), self.SAFE_RADIUS, 1)
        for i in range(0, self.N_SENSORS, self.N_SENSORS/16):
            sen_x, sen_y = self.get_sensor_vector(i)
            # Convert meters to centimeters
            sen_x = sen_x * 100.0
            sen_y = sen_y * 100.0
            if i == FRONT_SENSOR:
                pygame.draw.aaline(screen, green, (self.pos_x,self.pos_y), 
                    (self.pos_x+self.readings[i]*sen_x, self.pos_y+self.readings[i]*sen_y))
            else:
                pygame.draw.aaline(screen, red, (self.pos_x,self.pos_y), 
                    (self.pos_x+self.readings[i]*sen_x, self.pos_y+self.readings[i]*sen_y))
        screen.blit(FONT.render("phi:   %f" % (self.phi), True, white), (10,10))
        screen.blit(FONT.render("vel:   %f" % (self.vel), True, white), (10,25))
        screen.blit(FONT.render("dist:  %f" % (self.readings[FRONT_SENSOR]), True, white), (10, 40))
        if self.collided:
            text = BIG_FONT.render("GAME OVER", True, red)
            posx = int(S_WIDTH/2-text.get_rect().centerx)
            posy = int(S_HEIGHT/4-text.get_rect().centery)
            screen.blit(text, (posx, posy))
        if self.finished:
            text = BIG_FONT.render("Good Job!", True, red)
            posx = int(S_WIDTH/2-text.get_rect().centerx)
            posy = int(S_HEIGHT/4-text.get_rect().centery)
            screen.blit(text, (posx, posy))

class Wall:
    def __init__(self, p1, p2):
        self.xs = [p1[0],p2[0]]
        self.ys = [p1[1],p2[1]]

    def get_endpoints(self):
        return self.xs, self.ys

class Obstacle:
    def __init__(self, rect):
        self.rect = rect
        self.walls = [Wall(rect.topleft, rect.bottomleft),
                 Wall(rect.bottomleft, rect.bottomright),
                 Wall(rect.bottomright, rect.topright),
                 Wall(rect.topright, rect.topleft)]

    def get_walls(self):
        return self.walls

    def redraw(self, screen):
        pygame.draw.rect(screen, blue, self.rect)

class Target(Obstacle):
    def redraw(self, screen):
        pygame.draw.rect(screen, orange, self.rect)

def main():
    ###### INITIALIZATION ######
    pygame.init()
    global FONT
    FONT  = pygame.font.Font(pygame.font.get_default_font(),12)
    global BIG_FONT
    BIG_FONT  = pygame.font.Font(pygame.font.get_default_font(),48)

    clock = pygame.time.Clock()
    screen = pygame.display.set_mode((S_WIDTH, S_HEIGHT), DOUBLEBUF)
    vehicle = Vehicle(RADIUS, SAFE_RADIUS, N_SENSORS)

    obstacles = []
    # Bounding boxes
    obstacles.append(Obstacle(Rect(0, 0, 5, S_HEIGHT)))
    obstacles.append(Obstacle(Rect(S_WIDTH-5, 0, 5, S_HEIGHT)))
    obstacles.append(Obstacle(Rect(0, 0, S_WIDTH, 5)))
    obstacles.append(Obstacle(Rect(0, S_HEIGHT-5, S_WIDTH, 5)))
    # Extra obstacles
    obstacles.append(Obstacle(Rect(0,200,S_WIDTH-150,50)))
    obstacles.append(Obstacle(Rect(200,350,50,50)))
    obstacles.append(Obstacle(Rect(200,550,50,50)))
    obstacles.append(Obstacle(Rect(700,450,50,50)))

    # Targets
    targets = []
    targets.append(Target(Rect(75,450,50,50)))
    targets.append(Target(Rect(75,75,50,50)))

    screen.fill(black)
    vehicle.update(screen,obstacles)
    for obstacle in obstacles:
        obstacle.redraw(screen)
    if targets:
        targets[0].redraw(screen)
    pygame.display.flip()

    ###### Wait to Begin ######
    started = False
    while (not started):
        text = BIG_FONT.render("Press ENTER to begin", True, red)
        posx = int(S_WIDTH/2-text.get_rect().centerx)
        posy = int(S_HEIGHT/4-text.get_rect().centery)
        screen.blit(text, (posx, posy))
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif hasattr(event, 'key') and event.type == pygame.KEYDOWN:
                if event.key == K_q:
                    pygame.quit()
                    sys.exit()
                elif event.key == K_RETURN:
                    started = True
    stats.start()

    ###### MAIN GAME LOOP ######
    while (True):
        clock.tick(FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                stats.print_stats()
                pygame.quit()
                sys.exit()
            elif hasattr(event, 'key') and event.type == pygame.KEYDOWN:
                if event.key == K_q:
                    stats.print_stats()
                    pygame.quit()
                    sys.exit()
        vehicle.user_control()
        screen.fill(black)
        vehicle.update(screen, obstacles)
        for obstacle in obstacles:
            obstacle.redraw(screen)
        if targets:
            # If the user reached the first target, remove it and show the next
            if targets[0].rect.collidepoint(vehicle.pos_x, vehicle.pos_y):
                targets.pop(0)
            if targets:
                targets[0].redraw(screen)
            else:
                vehicle.finished = True
        else:
            vehicle.finished = True
        if vehicle.finished and not stats.saved_stats:
            stats.stop()
            stats.save_stats()
            pygame.quit()
            sys.exit()
        pygame.display.flip()

if __name__ == "__main__":
    ###### Get Parameters ######
    identifier = raw_input("Enter subject ID: ")
    while not identifier:
        identifier = raw_input("Invalid input. Enter subject ID: ")
    stats = Statistics(identifier)

    print("Enter avoidance method:")
    print("  0: Halt on proximity")
    print("  1: Gain attenuation")
    print("  2: Dynamic control")
    method = raw_input("")
    while (not method.isdigit()) or int(method) > 2:
        method = raw_input("Invalid input. Enter avoidance method: ")
    avoid.method = int(method)

    main()
