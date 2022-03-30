import pymunk, sys
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d
import pygame
from pygame.locals import *
import numpy as np
from PIL import Image
from pymunk.pygame_util import DrawOptions
import neat


size = 800, 800
display = pygame.display.set_mode((size))
options = DrawOptions(display)
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = 0, 981
b0 = space.static_body

FPS = 120

def get_theta(x_h, x_1, y_h, y_1):
    return np.arctan2(x_1 - x_h, y_1 - y_h)

def get_phi(x1, x2, y1, y2, theta):
    return np.arctan2(x2 - x1, y2- y1) - theta

def get_iota(x1, x2, y1, y2, theta, phi):
    return np.arctan2(x2 -x1, y2 - y1) - theta - phi

class Swing_robot:
    def __init__(self):
        # upper pendulum
        self.penbody = pymunk.Body()
        self.penbody.position = (400 , 500)
        self.pen_a = (0, -100)
        self.pen_b = (0, 100)
        self.penshape = pymunk.Segment(self.penbody, self.pen_a, self.pen_b, 10)
        self.penshape.density = 0.01
        self.penshape.filter = pymunk.ShapeFilter(group=1)
        self.penshape.color = (0, 255, 0, 0)
        space.add(self.penbody, self.penshape)
        # swing
        self.swingbody = pymunk.Body()
        self.swingbody.position = (400, 625)
        self.s1 = pymunk.Segment(self.swingbody, (30, -25), (0, -25) , 10)
        self.s1.filter = pymunk.ShapeFilter(group = 1)
        self.s1.density = 0.05
        self.s2 = pymunk.Segment(self.swingbody, (0, -25), (0, 25), 10)
        self.s2.filter = pymunk.ShapeFilter(group = 1)
        self.s2.density = 0.05
        self.s3 = pymunk.Segment(self.swingbody, (-20, 25),(20, 25), 10)
        self.s3.filter = pymunk.ShapeFilter(group = 1)
        self.s3.density = 0.05
        self.s4 = pymunk.Segment(self.swingbody, (0,25), (-20,-25), 10)
        self.s4.filter = pymunk.ShapeFilter(group = 1)
        self.s4.density = 0.05
        space.add(self.swingbody, self.s1,self.s2,self.s3,self.s4)
        # leg and foot
        self.legbody = pymunk.Body()
        self.legbody.position = (420,680)
        self.legradius = 10
        self.footradius = 10
        self.leg_a = (0, -30)
        self.leg_b = (0, 30)
        self.foot_a = (0,30)
        self.foot_b = (15,30)
        self.leg = pymunk.Segment(self.legbody, self.leg_a, self.leg_b , self.legradius)
        self.leg.filter = pymunk.ShapeFilter(group = 1)
        self.leg.density = 0.01
        self.leg.color = (0, 255, 0, 0)
        self.foot = pymunk.Segment(self.legbody, self.foot_a, self.foot_b, self.footradius)
        self.foot.filter = pymunk.ShapeFilter(group = 1)
        self.foot.density = 0.05
        self.foot.filter = pymunk.ShapeFilter(group=1)
        self.foot.color = (0, 255, 0, 0)
        space.add(self.legbody, self.leg, self.foot)
        # constraints
        self.pinjoint1 = pymunk.constraints.PinJoint(b0, self.penbody, (400,400), (0, -100))
        space.add(self.pinjoint1)
        self.pinjoint2 = pymunk.constraints.PinJoint(self.penbody, self.swingbody, (0, 100), (0, -25))
        space.add(self.pinjoint2)
        self.pinjoint3 = pymunk.constraints.PinJoint(self.swingbody, self.legbody, (20, 25), (0, -30))
        space.add(self.pinjoint3)
        # rotation lim joint
        self.rotation_joint = pymunk.constraints.RotaryLimitJoint(self.penbody, self.swingbody, -np.pi/4, np.pi/4)
        self.rotation_joint.collide_bodies = True
        space.add(self.rotation_joint)
        # motor
    def add_motor(self, rate):
        self.rate = rate
        self.simplemotor = pymunk.SimpleMotor(self.swingbody, self.legbody, self.rate)
        space.add(self.simplemotor)
    def remove_motor(self):
        space.remove(self.simplemotor)
    

body1 = Swing_robot()
b
body2 = Swing_robot()
def game():
    pygame.display.set_caption("Double pendulum interactive Simulation")
    while True:

        print(body1.penbody.position[0])
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                # print(f"Highest angle reached was:{np.rad2deg(high_score)}")
                pygame.quit()
                sys.exit()
        # high_score = angle_reached(theta, high_score)
        display.fill((255, 255, 255))
        space.debug_draw(options)
        pygame.display.update()
        clock.tick(FPS)  # limiting frames per second to 120
        space.step(1/FPS)


game()
pygame.quit()
