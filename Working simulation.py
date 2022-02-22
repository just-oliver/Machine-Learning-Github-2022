import pymunk, sys
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d

import pygame
from pygame.locals import *
import numpy as np
from PIL import Image
from pymunk.pygame_util import DrawOptions

size = 800, 800
display = pygame.display.set_mode((size))
options = DrawOptions(display)
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = 0, 981
b0 = space.static_body
FPS = 120

# def convert_coordinates(point):
    # return int(point[0]), int(800-point[1])

def get_theta(x_h, x_1,y_h,y_1):
        return np.arctan2(x_1 - x_h, y_1 - y_h)

def get_phi(x1,x2,y1,y2, theta):
    return np.pi/2 + np.arctan2(x2 - x1, y2- y1) - theta

class Segment:
    def __init__(self, p0, v, radius=10):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.v = v
        shape = pymunk.Segment(self.body, (0, 0), self.v, radius)
        shape.density = 0.1
        shape.elasticity = 0.5
        shape.filter = pymunk.ShapeFilter(group=1)
        shape.color = (0, 255, 0, 0)
        space.add(self.body, shape)
        
    # def draw(self):
        # pos1 = convert_coordinates(self.body.position)
        # pos2 = convert_coordinates(self.body.position + self.v)
        # pygame.draw.line(display, (0, 0, 0), pos1, pos2, 2)
    
    
class SimpleMotor:
    def __init__(self, b, b2, rate=5, switch="off"):
        self.rate = rate
        self.simplemotor = pymunk.constraints.SimpleMotor(b, b2, self.rate)
        self.switch = switch
    def drive(self, constraints):
        if self.switch == "off" and len(constraints) == 4:
            space.remove(self.simplemotor)
        if self.switch == "on" and len(constraints) != 4:
            space.add(self.simplemotor)
            
class RotaryLimitJoint:
    def __init__(self, b, b2, min, max, collide=True):
        joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)

class PivotJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), collide=True):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        joint.collide_bodies = collide
        space.add(joint)

p = Vec2d(400,400)
v = Vec2d(0,60)

ang_vel = -2
segment = Segment(p, 5*v)
PivotJoint(b0, segment.body, p)
segment2 = Segment(p+5*v, v)
PivotJoint(segment.body, segment2.body, 5*v)
simplemotor = SimpleMotor(segment.body, segment2.body, ang_vel)
pivotjoint = RotaryLimitJoint(segment.body, segment2.body, 0, np.pi/2)


def game():
    pygame.display.set_caption("Double pendulum interactive Simulation")
    while True:
        # x_h, y_h = segment.body.position[0], segment.body.position[1]
        # x_1, y_1 = segment2.body.position[0], segment2.body.position[1]
        # x_2, y_2 = segment2.body.position[0] + .v[0], segment2.body.position[1] + v[1]
        # theta = get_theta(x_h, x_1, y_h, y_1)
        # print(segment2.normal)
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        # if event.type == KEYDOWN and event.key == K_LEFT:
            # simplemotor.rate = rotation_rate
        keys = pygame.key.get_pressed()
        if keys[pygame.K_SPACE]: # kick input
            simplemotor.switch = "on"
        else:
            simplemotor.switch ="off"
        simplemotor.drive(space.constraints)
        
        # rendering frame
        display.fill((255, 255, 255)) 
        space.debug_draw(options)
        # self.draw()
        pygame.display.update()
        clock.tick(FPS)  # limiting frames per second to 120
        space.step(1/FPS)
        

game()
pygame.quit()