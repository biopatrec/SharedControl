#!/usr/bin/env python

import sys, math
import numpy as np

# Convenience definitions
TAU = 2*math.pi
N_SECTORS = 8
SECTOR_WIDTH = TAU/N_SECTORS
SECTOR_ANGLES = [s*TAU/N_SECTORS-((s>N_SECTORS/2)*TAU) for s in range(N_SECTORS)]
RADS_TO_SECTOR = lambda x: int((x+TAU+SECTOR_WIDTH/2)*N_SECTORS/TAU) % N_SECTORS

# Collision Avoidance Enumeration
AVOIDANCE_HALT      = 0     # Stop when getting too close to an obstacle
AVOIDANCE_CONTACT   = 1     # Make minimum time to contact constant
AVOIDANCE_POTENTIAL = 2     # Use virtual potentials to adjust speed and direction

# Math Optimization
N_SQRT_E = math.exp(-1.0/2)
SIGMA_2 = (SECTOR_WIDTH/2)**2

# System defaults
DEFAULT_MAX_VEL = 10.0      # Maximum velocity (m/s)
DEFAULT_VEL_INC = 0.1       # Velocity increment (m/s)
DEFAULT_PHI_INC = TAU/90.0  # Heading increment (rad/s)

DEFAULT_BETA_1  = 50.0      # Max repellor strength
DEFAULT_BETA_2  = 50.0      # Repellor strength decay
DEFAULT_C_V_OBS = 30.0      # Velocity repellor strength

DEFAULT_GAIN_RATE = 50.0
DEFAULT_TIME_RATE = 2.0

DEFAULT_HALT_RADIUS = 30  # Minimum safe radius from copter (cm)
DEFAULT_SAFE_RADIUS = 20  # Minimum safe radius from copter (cm)
DEFAULT_COPTER_RADIUS = 10 # Copter radius (cm)
DEFAULT_METHOD  = AVOIDANCE_HALT
DEFAULT_UPDATE_RATE = 20

class Avoidance:

    def __init__(self):
        # Load defaults
        self.max_vel = DEFAULT_MAX_VEL
        self.vel_inc = DEFAULT_VEL_INC
        self.phi_inc = DEFAULT_PHI_INC
        self.beta_1 = DEFAULT_BETA_1
        self.beta_2 = DEFAULT_BETA_2
        self.c_v_obs = DEFAULT_C_V_OBS
        self.safe_radius = DEFAULT_SAFE_RADIUS
        self.copter_radius = DEFAULT_COPTER_RADIUS
        self.method = DEFAULT_METHOD
        self.halt_radius = DEFAULT_SAFE_RADIUS
        self.gain_rate = DEFAULT_GAIN_RATE
        self.time_rate = DEFAULT_TIME_RATE
        self.update_rate = DEFAULT_UPDATE_RATE
        self.update()

    def update(self):
        self.potential = 0
        self.sigma_v = (2*self.max_vel)**2

    def convert(self, sensors, n_sensors):
        sector_vals = [0 for x in range(N_SECTORS)]
        sensors_per_sector = int(n_sensors/N_SECTORS)
        sector = 0
        for sensor in range(0,n_sensors,sensors_per_sector):
            sector_vals[sector] = min(sensors[sensor:sensor+sensors_per_sector])
            sector = sector + 1
        return sector_vals

    def potential_velocity(self, heading, velocity, distances):
        heading_sector = RADS_TO_SECTOR(heading)
        distance = distances[heading_sector]
        alpha = math.atan(100.0*self.potential)/math.pi
        c_obs = self.c_v_obs*(math.pi/2 + alpha)
        v_obs = self.avoidance_contact(heading, velocity, distances)
        # Obstacle velocity change
        d_velocity = c_obs*v_obs*math.exp(-(v_obs**2)/(2*self.sigma_v))/self.update_rate
        return d_velocity

    def potential_heading(self, heading, velocity, distances):
        delta_phi = 0
        self.potential = 0
        for sector in range(N_SECTORS):
            psi = SECTOR_ANGLES[sector] + heading
            sec = RADS_TO_SECTOR(psi)
            distance = distances[sec]
            phi = -SECTOR_ANGLES[sector]
            lmbda = self.beta_1 * math.exp(-distance/self.beta_2)
            gamma = math.exp(-(phi**2)/(2*SIGMA_2))
            delta_phi = delta_phi + lmbda*phi*gamma
            self.potential = self.potential + lmbda*SIGMA_2*(gamma-N_SQRT_E)
        d_heading = delta_phi/self.update_rate
        return d_heading

    # Use dynamic potentials to adjust heading and velocity
    def avoidance_potential(self, heading, velocity, distances):
        d_heading = self.potential_heading(heading, velocity, distances)
        d_velocity = self.potential_velocity(heading, velocity, distances)
        #(_,d_velocity) = self.avoidance_contact(heading, velocity, distances)
        return (d_heading, d_velocity)

    # Limit velocity by time to contact with obstacle
    def avoidance_contact(self, heading, velocity, distances):
        new_velocity = min(velocity, self.max_vel)
        fwd_sectors = [s for s in range(N_SECTORS)
                       if math.cos(SECTOR_ANGLES[s]+heading) > 1e-6]
        for sector in fwd_sectors:
            new_velocity = min((distances[sector]-self.safe_radius) / 
                                 self.time_rate, new_velocity)
        return new_velocity - velocity

    # Set user control gain as a factor of distance (0.0 - 1.0)
    def avoidance_gain(self, heading, velocity, distances):
        gain = 1.0
        fwd_sectors = [s for s in range(N_SECTORS)
                       if math.cos(SECTOR_ANGLES[s]+heading) > 1e-6]
        for sector in fwd_sectors:
            gain = min(gain, (distances[sector]-self.safe_radius) / self.gain_rate)
        # Make sure gain is >= 0
        gain = max(0.0, gain)
        new_velocity = min(self.max_vel, velocity * gain)
        return new_velocity - velocity
    
    # Set velocity to 0.0 if vehicle is too close to something
    def avoidance_halt(self, heading, velocity, distances):
        new_velocity = velocity
        fwd_sectors = [s for s in range(N_SECTORS)
                       if math.cos(SECTOR_ANGLES[s]+heading) > 1e-6]
        for sector in fwd_sectors:
            if distances[sector] <= self.halt_radius:
                new_velocity = 0.0
        return new_velocity - velocity

    # Set velocity to 0.0 if vehicle is too close to something
    def safety(self, heading, velocity, distances):
        new_velocity = velocity
        fwd_sectors = [s for s in range(N_SECTORS)
                       if math.cos(SECTOR_ANGLES[s]+heading) > 1e-6]
        for sector in fwd_sectors:
            if distances[sector] <= self.safe_radius:
                new_velocity = 0.0
                print("Safety stop!")
        return new_velocity - velocity

    def update_trajectory(self, heading, velocity, distances):
        phi = heading + math.pi*(velocity < 0)
        vel_sign = math.copysign(1.0, velocity)
        vel = abs(velocity)

        if self.method == AVOIDANCE_CONTACT:
            d_phi = 0.0
            d_vel = self.avoidance_contact(phi, vel, distances)
        elif self.method == AVOIDANCE_POTENTIAL:
            (d_phi, d_vel) = self.avoidance_potential(phi, vel, distances)
        else:
            d_phi = 0.0
            d_vel = self.avoidance_halt(phi, vel, distances)

        # Hard stop safety
        safety = self.safety(d_phi + phi, d_vel + vel, distances)
        if abs(safety) > 1e-6:
            d_vel = -vel

        d_vel = vel_sign * d_vel
        return d_phi, d_vel
