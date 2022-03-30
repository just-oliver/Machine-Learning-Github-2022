import pygame
import pymunk
import sys
from pygame.locals import *
import numpy as np
pygame.init()

display = pygame.display.set_mode((800, 800))
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = (0, -900)
FPS = 120
# def angle_to_coords(radius, position)


def convert_coordinates(point):
    return int(point[0]), int(800-point[1])


def get_angle(pos_body1, pos_body2):
    x_1, y_1 = pos_body1[0], pos_body1[1]
    x_2, y_2 = pos_body2[0], pos_body2[1]
    x_snip = (x_1-x_2)
    y_snip = (y_1-y_2)
    if x_snip == 0:
        return 0
    elif y_snip == 0:
        return np.pi/2
    else:
        angle = np.arctan(x_snip/y_snip)
        if angle < 0:
            angle = angle * -1
        return angle

def get_position_leg(angle, pos_body2, radius):
    x = tuple(pos_body2)[0] - (np.sin(angle) * radius)
    y = tuple(pos_body2)[1] + (np.cos(angle) * radius)
    return x, y
    
    

class Ball():
    def __init__(self, x, y):
        self.body = pymunk.Body()
        self.body.position = x, y
        self.shape = pymunk.Circle(self.body, 10)
        self.shape.density = 1
        self.shape.elasticity = 1
        # for dynamic bodies we have to add to pymunk space
        space.add(self.body, self.shape)

    def draw(self):
        pygame.draw.circle(display, (255, 0, 0),
                           convert_coordinates(self.body.position), 10)



class String():
    def __init__(self, body1, attachment, identifier="body"):
        self.body1 = body1
        if identifier == "body":
            self.body2 = attachment
        elif identifier == "position":
            self.body2 = pymunk.Body(body_type=pymunk.Body.STATIC)
            self.body2.position = attachment
        joint = pymunk.PinJoint(self.body1, self.body2)
        space.add(joint)  # for dynamic bodies we have to add to pymunk space

    def draw(self):
        pos1 = convert_coordinates(self.body1.position)
        pos2 = convert_coordinates(self.body2.position)
        pygame.draw.line(display, (0, 0, 0), pos1, pos2, 2)


ball_1 = Ball(400, 200)
ball_2 = Ball(300, 200)
string_1 = String(ball_1.body, (400, 800), "position")
string_2 = String(ball_1.body, ball_2.body)

def rotation_fn(x_1, y_1, x_2, y_2, max_angle):
    if x_1 > x_2 or y_2 > y_1:
        position = x_2, y_2
        return position
    else:
        delta_angle = max_angle/FPS
        x, y = x_2 - x_1, y_2 - y_1 # treating the pos of ball_1 as origin
        x_coo = x*np.cos(delta_angle) - y*np.sin(delta_angle) # coo change of origin
        y_coo = x*np.sin(delta_angle) + y*np.cos(delta_angle)
        position = x_coo + x_1, y_coo + y_1
        return position
    

def game():
    pygame.display.set_caption("Double pendulum interactive Simulation")
    frames = 0
    while True:
        limit = False
        ball_1_x = tuple(ball_1.body.position)[0]
        ball_1_y = tuple(ball_1.body.position)[1]
        ball_2_x = tuple(ball_2.body.position)[0]
        ball_2_y = tuple(ball_2.body.position)[1]
        rad = np.sqrt((ball_1_x - ball_2_x)**2 + (ball_1_y - ball_2_y)**2)
        if frames == 0:
            ball_2.body.position = (ball_1_x, (ball_1_y - rad))
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            
        keys = pygame.key.get_pressed()
        if keys[pygame.K_SPACE]: # kick input
            position = rotation_fn(ball_1_x, ball_1_y, ball_2_x, ball_2_y, np.pi/2)
            ball_2.body.position = position
            
                    
        pygame.display.update()  # rendering frame
        display.fill((255, 255, 255))  # background colour
        ball_1.draw()
        ball_2.draw()
        string_1.draw()
        string_2.draw()
        pygame.display.update()
        clock.tick(FPS)  # limiting frames per second to 120
        space.step(1/FPS)
        frames += 1
        

game()
pygame.quit()
