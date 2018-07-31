import pygame, sys, os
import math
from pygame.locals import *

FPS     = 30
black   = (0,0,0)
white   = (255,255,255)
red     = (232,9,9)
yellow  = (232,232,9)
green   = (29,179,56)

time_rate     = 1.0
safe_radius   = 0.40
velocity            = 1.5

pygame.init()
screen = pygame.display.set_mode((256,256), DOUBLEBUF)
clock = pygame.time.Clock()

black_arrow = pygame.image.load("arrow_black.png")
blue_arrow  = pygame.image.load("arrow_blue.png")
black_turn  = pygame.image.load("turn_black.png")
blue_turn   = pygame.image.load("turn_blue.png")
offset      = 32

cw_black    = black_turn
cw_blue     = blue_turn
ccw_black   = pygame.transform.flip(black_turn, True, False)
ccw_blue    = pygame.transform.flip(blue_turn, True, False)
right_black = black_arrow
right_blue  = blue_arrow
left_black  = pygame.transform.rotate(right_black, 180)
left_blue   = pygame.transform.rotate(right_blue,  180)
up_black    = pygame.transform.rotate(right_black,  90)
up_blue     = pygame.transform.rotate(right_blue,   90)
down_black  = pygame.transform.rotate(right_black, 270)
down_blue   = pygame.transform.rotate(right_blue,  270)

right_rect  = right_black.get_rect().move(128+offset, 64+offset)
left_rect   = right_black.get_rect().move(0+offset,   64+offset)
up_rect     = right_black.get_rect().move(64+offset,   0+offset)
down_rect   = right_black.get_rect().move(64+offset, 128+offset)
cw_rect     = cw_black.get_rect().move(   128+offset,  0+offset)
ccw_rect    = cw_black.get_rect().move(     0+offset,  0+offset)

def update(distances):
    screen.fill(white)
    # Change arrow color based on user input
    keys = pygame.key.get_pressed()
    if keys[K_w]:
        screen.blit(up_blue, up_rect)
    else:
        screen.blit(up_black, up_rect)
    if keys[K_a]:
        screen.blit(left_blue, left_rect)
    else:
        screen.blit(left_black, left_rect)

    if keys[K_s]:
        screen.blit(down_blue, down_rect)
    else:
        screen.blit(down_black, down_rect)
    if keys[K_d]:
        screen.blit(right_blue, right_rect)
    else:
        screen.blit(right_black, right_rect)

    if keys[K_q]:
        screen.blit(ccw_blue, ccw_rect)
    else:
        screen.blit(ccw_black, ccw_rect)
    if keys[K_e]:
        screen.blit(cw_blue, cw_rect)
    else:
        screen.blit(cw_black, cw_rect)
    # Draw arcs based on distance to objects
    for i in range(8):
        s_angle = (2*i-1)*math.pi/8.0 - math.pi/4
        e_angle = (2*i+1)*math.pi/8.0 - math.pi/4
        if distances[i] < safe_radius:
            pygame.draw.arc(screen, red, Rect(28,28,200,200), s_angle, e_angle, 4)
        elif distances[i]/time_rate < velocity:
            pygame.draw.arc(screen, yellow, Rect(18,18,220,220), s_angle, e_angle, 4)
        else:
            pygame.draw.arc(screen, green, Rect(8,8,240,240), s_angle, e_angle, 4)

    pygame.display.flip()

while 1:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    update([0,1,2,3,4,5,6,7,8])
    clock.tick(FPS)
