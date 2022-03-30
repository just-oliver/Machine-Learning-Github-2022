# -*- coding: utf-8 -*-
"""
Created on Thu Mar  3 17:34:04 2022

@author: user
"""

import gym
from gym import Env
from gym.spaces import Discrete, Box, Dict, MultiDiscrete

from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import StopTrainingOnMaxEpisodes

import os

import random as rd

import pymunk, sys
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d

import pygame
from pygame.locals import *
import numpy as np
from PIL import Image
from pymunk.pygame_util import DrawOptions

from pymunk._chipmunk_cffi import ffi, lib
import xlwings as xw

pygame.init()
# size = 800, 800
# display = pygame.display.set_mode((size))
# options = DrawOptions(display)
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = 0, 981
b0 = space.static_body
b1 = space.static_body
FPS = 120

filepath1 = r'C:\Users\62488\Project\Body Model\Data.xlsx'
filepath2 = r'C:\Users\62488\Project\Body Model\Project.xlsx'

wb1 = xw.Book(filepath1)
wb2 = xw.Book(filepath2)

sht1 = wb1.sheets['1000Episode']
sht2 = wb2.sheets['1000Episode']

def data_save1(time,angle,action1,action2):
    
    sht1.range(1,1).options(transpose=True).value = time
    sht1.range(1,2).options(transpose=True).value = angle
    sht1.range(1,3).options(transpose=True).value = action1
    sht1.range(1,4).options(transpose=True).value = action2
    
def data_save2(max_angle, score,a):
    sht2.range(a,1).options(transpose=True).value = max_angle
    sht2.range(a,2).options(transpose=True).value = score
    
    
    

    
    


def convert_coordinates(point):
    return int(point[0]), int(800-point[1])

def get_theta(x_h, x_1, y_h, y_1):
    return np.arctan2(x_1 - x_h, y_1 - y_h)

def get_phi(x1, x2, y1, y2, theta):
    return np.arctan2(x2 - x1, y2- y1) - theta

def get_iota(x1, x2, y1, y2, theta, phi):
    return np.arctan2(x2 -x1, y2 - y1) - theta - phi

def get_ieta(x1, x2, y1, y2, theta, phi):
    return np.arctan2(x2 -x1, y2 - y1) - theta - phi + np.pi

def Get_ieta(x1,x2,x3,y1,y2,y3):
    x = np.array([x1-x2, y2-y1])
    y = np.array([x3-x2, y2-y3])
    
    l_x = np.sqrt(x.dot(x))
    l_y = np.sqrt(y.dot(y))
    
    cos_ = x.dot(y)/(l_x * l_y)
    
    return np.arccos(cos_)


class measurement_body:
    def __init__(self):
        self.body = pymunk.Body()
        self.body.position = (400,40)
        self.shape = pymunk.Circle(self.body, 1)
        self.shape.color = (255,0,0)
        space.add(self.body, self.shape)
    
class Segment:
    def __init__(self, p0, a, b, radius=2, center_of_gravity = (0,0), density=0.00206):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.body.center_of_gravity = center_of_gravity
        self.shape = pymunk.Segment(self.body, self.a, self.b, radius)
        self.shape.density = density
        self.shape.elasticity = 0
        self.shape.filter = pymunk.ShapeFilter(group=1)
        self.shape.color = (0, 255, 0, 0)
        space.add(self.body, self.shape)

class Leg:
    def __init__(self, p0, a, b, c, d, radius=5, center_of_gravity = (0,0), density=0.00352):
        self.body = pymunk.Body()#mass of leg
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.body.center_of_gravity = center_of_gravity
        self.leg= pymunk.Segment(self.body, self.a, self.b , radius=7)
        self.leg.filter = pymunk.ShapeFilter(group = 1)
        self.leg.density = density
        self.leg.color = (255, 0, 150, 0)
        self.foot= pymunk.Segment(self.body, self.c, self.d, radius=radius)
        self.foot.filter = pymunk.ShapeFilter(group = 1)
        self.foot.density = density
        self.foot.filter = pymunk.ShapeFilter(group=1)
        self.foot.color = (255, 0, 150, 0)
        space.add(self.body, self.leg, self.foot)
        
        
        
class Simplemotor:
    def __init__(self, b, b2, rate=5, switch="off"):
        self.rate = rate
        self.b = b
        self.b2 = b2
        self.simplemotor = pymunk.SimpleMotor(self.b, self.b2, self.rate)
        space.add(self.simplemotor)
    def remove(self):
        space.remove(self.simplemotor)

class RotaryLimitJoint:
    def __init__(self, b, b2, min, max, collide=True):
        joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, max)
        joint.collide_bodies = collide
        space.add(joint)
    
        

                  
# class dead_hang_joint:
#     def __init__(self, b, b2, min, max, collide=True):
#         joint = pymunk.constraints.RotaryLimitJoint(b, b2, min, angvel1}\nseg2:{angvel2}")
        # print(leg.bomax)
#         joint.collide_bodies = collide
#     def dead_position(self, constraints, phi):
#         if phi == 0 and len(constraints) < 6:  
            
        

class PivotJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0), collide=True):
        self.joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        self.joint.collide_bodies = collide
        space.add(self.joint)
    def remove(self):
        space.remove(self.joint)

class PinJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0)):
        self.joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        space.add(self.joint)
    def remove(self):
        space.remove(self.joint)

class Swing_body:
    def __init__(self,p0, vx1,vy1,vx2,vy2,vx3,vy3, radius=3, center_of_gravity = (0,0),  density = 0.00107):
        self.body = pymunk.Body() #(hip + hand + Thigh)(1.85558) + swing
        self.body.position = p0
        self.s1 = pymunk.Segment(self.body, vx1, vy1 , radius=radius)
        self.s1.filter = pymunk.ShapeFilter(group = 1)
        self.s1.density = density
        self.s2 = pymunk.Segment(self.body, vx2, vy2, radius=radius)
        self.s2.filter = pymunk.ShapeFilter(group = 1)
        self.s2.density = density
        self.s3 = pymunk.Segment(self.body, vx3,vy3, radius=radius)
        self.s3.filter = pymunk.ShapeFilter(group = 1)
        self.s3.density = density*10
        space.add(self.body, self.s1,self.s2,self.s3)
        
class Robot_body:
    def __init__(self, p0, a, b, radius=10, center_of_gravity = (0,0), density=0.00478):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.body.center_of_gravity = center_of_gravity
        self.shape = pymunk.Segment(self.body, self.a, self.b, radius)
        self.shape.density = density
        self.shape.elasticity = 0
        self.shape.filter = pymunk.ShapeFilter(group=1)
        self.shape.color = (255, 0, 150, 0)
        space.add(self.body, self.shape)
        
# def angle_reached(theta, high_score):
#     if len(high_score) == 0:
#         high_score.append(theta)
#     elif high_score[0] < abs(theta):
#         high_score[0] = abs(theta)
#         self.score + 1
        
#     highest_score = high_score[0]
#     return highers_score, self.score


# b1 = measurement_body()

# hinge_point1 = (0, -100) # seg 1
# hinge_point2 = (0, 100)
# swing_body = (400, 625)
# swing_top1 = (30, -25)
# swing_top2 = (-30, -25)
# swing_mid1 = (0, -25)
# swing_mid2 = (0, 25)
# swing_bottom1 = (-20, 25)
# swing_bottom2 = (20, 25)
# hinge_point3 = (0, -30) # seg 2
# hinge_point4 = (0, 30)

# "Pymunk Bodies"

# segment = Segment((400 , 500), hinge_point1 , hinge_point2)
# leg = Leg((420,680), hinge_point3, hinge_point4, (0,30), (15,30), density= 0.05)
# swing = Swing_body(swing_body, swing_top1,swing_top2, swing_mid1, swing_mid2, swing_bottom1, swing_bottom2)
# PinJoint(swing.body, leg.body, swing_bottom2, hinge_point3)
# PinJoint(segment.body, swing.body, hinge_point2, swing_mid1)
# PinJoint(b0, segment.body, (400,400), hinge_point1)

class ModelEnv(Env):
    def __init__(self):
        self.state = None
        self.action_space = MultiDiscrete([3,3])
        self.model_length = 545
        # self.observation_space = Box(np.array([0, 0, 0, 0, 0, 0, 0 ,0,-3,-3,-3,-3]), np.array([800, 800, 800, 800,800,800,800,800, 3, 3, 3, 3]))
        self.observation_space = Box(np.array([-np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -10, -10, -10, -10]), np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, 10, 10, 10, 10]))
        self.a = 1
        self.b = 0

        
    def angle_reached(self, theta, high_score):
        
        a = 0
        
        if len(high_score) == 0:
            high_score.append(theta)
            a = 0 
            
        elif high_score[0] < abs(theta):
            high_score[0] = abs(theta)
            a = np.rad2deg(high_score[0])*1000
        highest_score = high_score[0]
        return highest_score, a
    
    def angle_vel_reach(self,ang_vel):
        if ang_vel >= 0.5:
            return 1
        else:
            return 0
    
    def punish(self, theta1, theta2):
        if theta1 < theta2:
            return -20
        else:
            return 0
    
    def reset(self):
        self.size = 800, 800
        self.display = pygame.display.set_mode((self.size))
        self.options = DrawOptions(self.display)


        self.hinge_point1 = (0, -150) # seg 1
        self.hinge_point2 = (0, 150)
        self.swing_body = (400, 717.5)
        self.swing_top1 = (30, -17.5)
        self.swing_top2 = (0, -17.5)
        self.swing_mid1 = (0, -17.5)
        self.swing_mid2 = (0, 17.5)
        self.swing_bottom1 = (-20, 17.5)
        self.swing_bottom2 = (20, 17.5)
        self.hinge_point3 = (0, -15) # seg 2
        self.hinge_point4 = (0, 15)
        self.robot_point1 = (-13.782,-30.8624)
        self.robot_point2 = (13.782,30.8624)
        
        "Pymunk Bodies"
        
        self.segment = Segment((400 , 550), self.hinge_point1 , self.hinge_point2)
        self.leg = Leg((420,750), self.hinge_point3, self.hinge_point4, (0,15), (13,15))
        self.swing = Swing_body(self.swing_body, self.swing_top1,self.swing_top2, self.swing_mid1, self.swing_mid2, self.swing_bottom1, self.swing_bottom2)
        self.robot_b = Robot_body((386.218,704.138), self.robot_point1, self.robot_point2)
        self.pinjoint1 = PinJoint(self.swing.body, self.leg.body, self.swing_bottom2, self.hinge_point3)
        self.pinjoint2 = PinJoint(self.segment.body, self.swing.body, self.hinge_point2, self.swing_mid1)
        self.pinjoint3 = PinJoint(b0, self.segment.body, (400,400), self.hinge_point1)
        self.pinjoint4 = PinJoint(self.robot_b.body, self.swing.body, self.robot_point2, self.swing_mid2)
        self.x = "const motor"
        self.y = "const motor"
        self.constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
        self.constant_motor2 = Simplemotor(self.robot_b.body, self.swing.body, 0)
        
        self.model_length = 545
        self.reward = 0
        self.whole = 0
        self.punish_n = 0
        self.time_n = 0
        self.time = []
        self.angle = []
        self.act1 = []
        self.act2= []
        self.action_n = 0
        
        # ang_vel_segment = self.segment.body.angular_velocity
        # ang_vel_swing = self.swing.body.angular_velocity
        # ang_vel_leg = self.leg.body.angular_velocity
        # ang_vel_robot = self.robot_b.body.angular_velocity
        
        ang_vel_segment = round(self.segment.body.angular_velocity, 3)
        ang_vel_swing = round(self.swing.body.angular_velocity, 3)
        ang_vel_leg = round(self.leg.body.angular_velocity,3)
        ang_vel_robot = round(self.robot_b.body.angular_velocity,3)
        
        xh, yh = (400,400)
        x1, y1 = self.segment.body.position[0], self.segment.body.position[1]
        theta = get_theta(xh, x1, yh, y1)
        x2, y2 = self.segment.body.position[0] + 150*np.sin(theta) , self.segment.body.position[1] + 150*np.cos(theta) 
        x3, y3 = self.swing.body.position[0], self.swing.body.position[1]
        phi = get_phi(x2, x3, y2, y3, theta)
        x4, y4 = self.swing.body.position[0] + 17.5*np.sin(theta+phi) + 20*np.cos(theta+phi), self.swing.body.position[1] + 17.5*np.cos(theta+phi) - 20*np.sin(theta+phi) 
        x5, y5 = self.leg.body.position[0], self.leg.body.position[1]
        iota = get_iota(x4, x5, y4, y5, theta, phi)
        x6, y6 = self.robot_b.body.position[0], self.robot_b.body.position[1]
        x7, y7 = self.swing.body.position[0] + 17.5*np.sin(theta+phi) , self.swing.body.position[1] + 17.5*np.cos(theta+phi) 
        # ieta = get_ieta(x7, x6, y7, y6, theta, phi)
        
        ieta = Get_ieta(x6, x7, x3, y6, y7, y3)
        
        theta, phi, iota, ieta = round(theta,3),round(phi,3),round(iota,3), round(ieta,3)
        # x1,y1,x3,y3,x5,y5,x6,y6 = round(x1,3), round(y1,3),round(x3,3),round(y3,3),round(x5,3),round(y5,3),round(x6,3),round(y6,3)
         

        self.high_score = []
        self.state = np.array([theta, phi, iota, ieta, ang_vel_segment, ang_vel_swing, ang_vel_leg, ang_vel_robot])
        # self.state = np.array([x1,y1,x3,y3,x5,y5,x6,y6,ang_vel_segment, ang_vel_swing, ang_vel_leg, ang_vel_robot])
        
        
        # print(x1,y1)
        # print(x2,y2)
        # print(x3,y3)
        # print(x6,y6)
        # print(x7,y7)
        return self.state
            
    def clean(self):
        space.remove(self.segment.body)
        space.remove(self.segment.shape)
        space.remove(self.leg.body)
        space.remove(self.leg.leg)
        space.remove(self.leg.foot)
        space.remove(self.swing.body)
        space.remove(self.swing.s1)
        space.remove(self.swing.s2)
        space.remove(self.swing.s3)
        space.remove(self.robot_b.body)
        space.remove(self.robot_b.shape)
        self.pinjoint1.remove()
        self.pinjoint2.remove()
        self.pinjoint3.remove()
        self.pinjoint4.remove()

    def close(self):
        pygame.quit()
        
    def step(self, action):
        pygame.display.set_caption("Double pendulum interactive Simulation")

        # ang_vel_segment = self.segment.body.angular_velocity
        # ang_vel_swing = self.swing.body.angular_velocity
        # ang_vel_leg = self.leg.body.angular_velocity
        # ang_vel_robot = self.robot_b.body.angular_velocity
        ang_vel_segment = round(self.segment.body.angular_velocity, 3)
        ang_vel_swing = round(self.swing.body.angular_velocity, 3)
        ang_vel_leg = round(self.leg.body.angular_velocity,3)
        ang_vel_robot = round(self.robot_b.body.angular_velocity,3)
        # if self.whole % 11 == 0:
        #     self.action1 = action[0]
        #     self.action2 = action[1]

        self.action1 = action[0]
        self.action2 = action[1]
        # print(self.action1, self.action2)
        # print(len(space.constraints))

        xh, yh = (400,400)
        x1, y1 = self.segment.body.position[0], self.segment.body.position[1]

        theta = get_theta(xh, x1, yh, y1)
        x2, y2 = self.segment.body.position[0] + 150*np.sin(theta) , self.segment.body.position[1] + 150*np.cos(theta) 
        x3, y3 = self.swing.body.position[0], self.swing.body.position[1]
        phi = get_phi(x2, x3, y2, y3, theta)
        x4, y4 = self.swing.body.position[0] + 17.5*np.sin(theta+phi) + 20*np.cos(theta+phi), self.swing.body.position[1] + 17.5*np.cos(theta+phi) - 20*np.sin(theta+phi) 
        x5, y5 = self.leg.body.position[0], self.leg.body.position[1]
        iota = get_iota(x4, x5, y4, y5, theta, phi)
        x6, y6 = self.robot_b.body.position[0], self.robot_b.body.position[1]
        x7, y7 = self.swing.body.position[0] + 17.5*np.sin(theta+phi) , self.swing.body.position[1] + 17.5*np.cos(theta+phi) 
        ieta = Get_ieta(x6, x7, x3, y6, y7, y3)
        x1,y1,x3,y3,x5,y5,x6,y6 = round(x1,3), round(y1,3),round(x3,3),round(y3,3),round(x5,3),round(y5,3),round(x6,3),round(y6,3)
        #print(f"iota={iota}")
        theta, phi, iota, ieta = round(theta,3),round(phi,3),round(iota,3), round(ieta,3)
        const_angvel = self.swing.body.angular_velocity
        self.state = np.array([theta, phi, iota, ieta, ang_vel_segment, ang_vel_swing, ang_vel_leg, ang_vel_robot])
        # print(self.state)
        # self.state = np.array([x1,y1,x3,y3,x5,y5,x6,y6,ang_vel_segment, ang_vel_swing, ang_vel_leg, ang_vel_robot])
        
        # if np.rad2deg(np.abs(theta)) < 5:
        #     reward = -1
        # elif np.rad2deg(np.abs(theta)) >= 5 and np.rad2deg(np.abs(theta)) < 6:
        #     reward = 10
        # elif np.rad2deg(np.abs(theta)) >= 6 and np.rad2deg(np.abs(theta)) < 7:
        #     reward = 20
        # elif np.rad2deg(np.abs(theta)) >= 7 and np.rad2deg(np.abs(theta)) < 8:
        #     reward = 30
        # elif np.rad2deg(np.abs(theta)) >= 8 and np.rad2deg(np.abs(theta)) < 9:
        #     reward = 40
        # elif np.rad2deg(np.abs(theta)) >= 9 and np.rad2deg(np.abs(theta)) < 10:
        #     reward = 50
        # elif np.rad2deg(np.abs(theta)) >= 10:  
        #     reward = 100
            
        # if np.rad2deg(np.abs(theta)) >= 6.5:
        #     reward = 100
        # else:
        #     reward = -1
        reward = self.angle_reached(theta, self.high_score)[1]
        self.reward += reward
        
        if  self.model_length <= 0 or reward >= 2000000:
            done = True
        else:
            done = False
        # print(f"seg1:{angvel1}\nseg2:{angvel2}")
        # print(leg.body.angular_velocity)
        # abs_vel = np.sqrt(segment.body.velocity[0]**2 + segment.body.velocity[1]**2)
        # if segment.body.velocity[0]< 1:
        #     rad_vel = -abs_vel/150
        # else:
        #     rad_vel = abs_vel/150
        # print(rad_vel)
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                #print(f"Highest angle reached was:{np.rad2deg(highest_score)}")
                pygame.quit()
                sys.exit()

        
        if self.model_length == 0 or reward >= 2000000:
            self.clean()
            if self.x == "positive motor":
                self.pos_motor.remove()
            elif self.x == "negetive motor":
                self.neg_motor.remove()
            else:
                self.constant_motor.remove()
                
            if self.y == "positive motor":
                self.pos_motor2.remove()
            elif self.y == "negetive motor":
                self.neg_motor2.remove()
            else:
                self.constant_motor2.remove()
                
            if self.b == 0:
                data_save1(self.time, self.angle,self.action1, self.action2)
                self.b += 1
                
            data_save2(self.max_angle, self.reward, self.a)
            self.a += 1
            return self.state, reward, done, {}

            # pygame.display.quit()
            # pygame.init()
            # self.display = pygame.display.set_mode((self.size))
            # self.options = DrawOptions(self.display)
        # else:
        #     if iota < 1.4 and self.x == "const motor":
        #         self.constant_motor.remove()
        #         self.
        
        else:
            if self.action1 == 0:
                if iota < 1.4 and self.x == "const motor":
                    self.constant_motor.remove()
                    self.x = "positive motor"
                    self.pos_motor = Simplemotor(self.swing.body, self.leg.body, 2)
            elif self.action1 == 1:
                if iota > -0.9 and self.x == "const motor":
                    self.constant_motor.remove()
                    self.x = "negetive motor"
                    self.neg_motor = Simplemotor(self.swing.body, self.leg.body, -2)
            elif self.action1 == 2:
                if iota > -0.9 and iota <= 1.4:
                    if self.x == "negetive motor":
                        self.neg_motor.remove()
                        self.x = "const motor"
                        self.constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
                    elif self.x == "positive motor":
                        self.pos_motor.remove()
                        self.x = "const motor"
                        self.constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
            # elif self.action == 3:
            #     if iota < 1.4 and ieta > 0.42 and self.x == "const motor" and self.y == "const motor":
            #         self.constant_motor.remove()
            #         self.constant_motor2.remove()
            #         self.x = "positive motor"
            #         self.y = "negetive motor"
            #         self.pos_motor = Simplemotor(self.swing.body, self.leg.body, 1.5)
            #         self.neg_motor2 = Simplemotor(self.robot_b.body,self.swing.body, -1.5/3.0666)
                    
            # elif self.action == 4:
            #     if iota > -0.9 and ieta < 1.17 and self.x == "const motor" and self.y == "const motor":
            #         self.constant_motor.remove()
            #         self.constant_motor2.remove()
            #         self.x = "negetive motor"
            #         self.y = "positive motor"
            #         self.neg_motor = Simplemotor(self.swing.body, self.leg.body, -1.5)
            #         self.pos_motor2 = Simplemotor(self.robot_b.body, self.swing.body, 1.5/3.0666)
            if self.action2 == 0:
                if ieta > 0.1 and self.y == "const motor":
                    self.constant_motor2.remove()
                    self.y = "negetive motor"
                    self.neg_motor2 = Simplemotor(self.robot_b.body, self.swing.body, -0.5)
            elif self.action2 == 1:
                if ieta < 1.5 and self.y == "const motor":
                    self.constant_motor2.remove()
                    self.y = "positive motor"
                    self.pos_motor2 = Simplemotor(self.robot_b.body, self.swing.body, 0.5)
            elif self.action2 == 2:
                if ieta > 0.2 and ieta <= 1.5:
                    if self.y == "negetive motor":
                        self.neg_motor2.remove()
                        self.y = "const motor"
                        self.constant_motor2 = Simplemotor(self.robot_b.body, self.swing.body, 0)
                    elif self.y == "positive motor":
                        self.pos_motor2.remove()
                        self.y = "const motor"
                        self.constant_motor2 = Simplemotor(self.robot_b.body, self.swing.body, 0)
                        
            if iota >= 1.4 and self.x == "positive motor":
                self.pos_motor.remove()
                self.constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
                self.x = "const motor"
            if iota <= -0.9 and self.x == "negetive motor":
                self.neg_motor.remove()
                self.constant_motor = Simplemotor(self.swing.body, self.leg.body, 0)
                self.x = "const motor"
            if ieta >= 1.17 and self.y == "negetive motor":
                self.neg_motor2.remove()
                self.constant_motor2 = Simplemotor(self.robot_b.body, self.swing.body, 0)
                self.y = "const motor"
            if ieta <=  0.42 and self.y == "positive motor":
                self.pos_motor2.remove()
                self.constant_motor2 = Simplemotor(self.robot_b.body, self.swing.body, 0)
                self.y = "const motor"
            # print(ieta)
            # print(phi)
            # if ieta >= 1.17 and self.y == "positive motor"
            #     self.
                            
        
        # keys = pygame.key.get_pressed()
        # if keys[pygame.K_SPACE]: # kick input
        #     simplemotor.switch = "on"
        #     if iota >= np.pi/2:
        #         if len(space.constraints) == 5:
        #             space.remove(simplemotor.simplemotor)
        #         leg.body.angular_velocity = angvel1
        #     else:
        #         simplemotor.drive(space.constraints, phi)
        # else:
        #     simplemotor.switch = "off"
        #     if iota <= 0:
        #         leg.body.angular_velocity = angvel1
        #     else:
        #         simplemotor.drive(space.constraints, phi
        # punish = 0
        # if self.punish_n >= 600:
        #     punish =  self.punish(np.abs(theta), self.angle_reached(theta, self.high_score)[0])
        #     self.punish_n = 0

        # self.reward = self.reward + self.angle_reached(theta, self.high_score)[1] + self.angle_vel_reach(np.abs(ang_vel_segment)) + punish

        self.max_angle = np.rad2deg(self.angle_reached(theta, self.high_score)[0])
        print("The max angle is:{0}".format(self.max_angle))

            
        # print(np.abs(ang_vel_segment))
        # if np.abs(theta) < self.angle_reached(theta, self.high_score)[0]:
        #     if self.punish >= 600:
        #         self.reward -= 5
        #         self.punish = 0
        #         self.punish_n +=5
        
        # for x in range(11):
        #     # self.display.fill((255, 255, 255)) 
        #     # space.debug_draw(self.options)
        #     space.step(1/120)
            # clock.tick(120) 
            # pygame.display.update()

        

        
        # self.display.fill((255, 255, 255)) 
        # space.debug_draw(self.options)
        # pygame.display.update()
        # clock.tick(FPS)  # limiting frames per second to 120
        
        for x in range(11):
            space.step(1/120)
        # space.step(1/120)
        # print(len(space.constraints))
        #print(x)
        info = {}
        # print(len(space.constraints))
        # print(iota)
        # print(action)
        if self.whole % 15 == 0:
            self.time_n += 0.5
            self.time.append(self.time_n)
            self.angle.append(theta)
            
        
        # print(self.time,self.angle)
        
        self.whole += 1
        self.punish_n += 1
        self.model_length -= 1
        # print(self.model_length)

        return self.state, reward, done, info
    
    def render(self):
        for x in range(11):
            self.display.fill((255, 255, 255))  
            space.debug_draw(self.options)
            space.step(1/120)
            clock.tick(120) 
            pygame.display.update()
        # for i in range(11):
        #     self.display.fill((255, 255, 255)) 
        #     space.debug_draw(self.options)
        #     pygame.display.update()

        

            # space.step(1/300)
        # clock.tick(FPS)  # limiting frames per second to 120

# check_env(env, warn=True)env = ModelEnv()
# episodes = 7

env = ModelEnv()

# for episode in range(1, episodes +1):
#     state = env.reset()
#     done = False
#     score = 0
#     pygame.display.set_caption("Double pendulum interactive Simulation")
#     while not done:
#         env.render()
#         action = env.action_space.sample()
#         # print(action)
#         state, reward, done ,info = env.step(action)
#         print(info)
#         print(len(space.constraints))
#         print(reward)
# pygame.quit()
# 
# save_path = os.path.join('Train','Model Saved')
# best_path = os.path.join('Train','Model Saved','best_model')
log_path = os.path.join('Train','Logs')
model = PPO("MlpPolicy", env, verbose = 1, tensorboard_log=log_path)
ppo_path = os.path.join('Train','Model Saved','PPO_NewStep_model1000episode')
# # Stops training when the model reaches the maximum number of episodes
# stop_callback = StopTrainingOnRewardThreshold(reward_threshold=3000000, verbose=1)
# eval_callback = EvalCallback(env, 
#                              callback_on_new_best=stop_callback, 
#                              eval_freq=10000, 
#                              best_model_save_path = save_path, 
#                              verbose=1)
callback_max_episodes = StopTrainingOnMaxEpisodes(max_episodes=1000, verbose=1)
# ppo_path = os.path.join('Train','Model Saved','PPO_NewRewardF_n_100episodes')
# model = PPO.load(ppo_path, env)
# model = PPO.load(best_path, env)
model.learn(total_timesteps = 500000000000000000000000000, callback=(callback_max_episodes))
model.save(ppo_path)
# ppo_path = os.path.join('Train','Model Saved', 'PPO_10w_Segment1') # New Reward One
# ppo_path1 = os.path.join('Train','Model Saved', 'PPO_20w_Segment') # Good One
# ppo_path1 = os.path.join('Training','Model Saved', 'PPO_20w_Segment1')

# model= PPO.load(ppo_path1,env)

# ppo_path = os.path.join('Training','Model Saved','PPO3w_Segment')
# model.save(ppo_path)
# pygame.quit()

# evaluate_policy(model, env, n_eval_episodes=10, render=True)

obs = env.reset()
the = []
while True:
    env.render()
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    print("The score is : {}".format(rewards))

