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
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = 0, 981
b0 = space.static_body
b1 = space.static_body
FPS = 120
Act_contin = 11
Episode_time = 600

filepath1 = r'C:\Users\62488\Project\Body Model\Data.xlsx'
filepath2 = r'C:\Users\62488\Project\Body Model\Copy of Project1.xlsx'

wb1 = xw.Book(filepath1)
wb2 = xw.Book(filepath2)

'''
Change here to save the data in different sheet
'''
sht1 = wb1.sheets['First']
sht2 = wb2.sheets['sheet2']

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
        

class ModelEnv(Env):
    def __init__(self):
        self.state = None
        self.action_space = MultiDiscrete([3,3])
        self.model_length = round(Episode_time/(Act_contin/FPS))
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
        
        self.model_length = round(Episode_time/(Act_contin/FPS))
        self.reward = 0
        self.whole = 0
        self.punish_n = 0
        self.time_n = 0
        self.time = []
        self.angle = []
        self.act1 = []
        self.act2= []
        self.action_n = 0
    
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
        ieta = Get_ieta(x6, x7, x3, y6, y7, y3)
        
        theta, phi, iota, ieta = round(theta,3),round(phi,3),round(iota,3), round(ieta,3)
         

        self.high_score = []
        self.state = np.array([theta, phi, iota, ieta, ang_vel_segment, ang_vel_swing, ang_vel_leg, ang_vel_robot])
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

        ang_vel_segment = round(self.segment.body.angular_velocity, 3)
        ang_vel_swing = round(self.swing.body.angular_velocity, 3)
        ang_vel_leg = round(self.leg.body.angular_velocity,3)
        ang_vel_robot = round(self.robot_b.body.angular_velocity,3)


        self.action1 = action[0]
        self.action2 = action[1]

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
        
        theta, phi, iota, ieta = round(theta,3),round(phi,3),round(iota,3), round(ieta,3)
        const_angvel = self.swing.body.angular_velocity
        self.state = np.array([theta, phi, iota, ieta, ang_vel_segment, ang_vel_swing, ang_vel_leg, ang_vel_robot])

        '''
        Reward Function
        
        '''
        if np.rad2deg(np.abs(theta)) < 5:
            reward = -1
        elif np.rad2deg(np.abs(theta)) >= 5 and np.rad2deg(np.abs(theta)) < 6:
            reward = 10
        elif np.rad2deg(np.abs(theta)) >= 6 and np.rad2deg(np.abs(theta)) < 7:
            reward = 20
        elif np.rad2deg(np.abs(theta)) >= 7 and np.rad2deg(np.abs(theta)) < 8:
            reward = 30
        elif np.rad2deg(np.abs(theta)) >= 8 and np.rad2deg(np.abs(theta)) < 9:
            reward = 40
        elif np.rad2deg(np.abs(theta)) >= 9 and np.rad2deg(np.abs(theta)) < 10:
            reward = 50
        elif np.rad2deg(np.abs(theta)) >= 10:  
            reward = 100
            
        # if np.rad2deg(np.abs(theta)) >= 6.5:
        #     reward = 100
        # else:
        #     reward = -1
        # reward = self.angle_reached(theta, self.high_score)[1]
        self.reward += reward
        
        '''
        Justify whether the episode finish or not 
        '''
        if  self.model_length <= 0:
            done = True
        else:
            done = False
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                #print(f"Highest angle reached was:{np.rad2deg(highest_score)}")
                pygame.quit()
                sys.exit()

        '''
        Code to save the data each episode
        '''
        if self.model_length == 0:
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
                data_save1(self.time, self.angle,self.act1, self.act2)
                self.b += 1
                
            data_save2(self.max_angle, self.reward, self.a)
            self.a += 1
            return self.state, reward, done, {}
        #Add action
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

        self.max_angle = np.rad2deg(self.angle_reached(theta, self.high_score)[0])
        print("The max angle is:{0}".format(self.max_angle))

        for x in range(Act_contin):
            space.step(1/FPS)
        
        self.time_n += Act_contin/FPS
        self.time.append(self.time_n)
        self.angle.append(theta)
        self.act1.append(self.action1)
        self.act2.append(self.action2)
            
        # space.step(1/120)
        info = {}
        self.whole += 1
        self.punish_n += 1
        self.model_length -= 1
        # print(self.model_length)

        return self.state, reward, done, info
    
    def render(self):
        # for x in range(Act_contin):
        #     self.display.fill((255, 255, 255))  
        #     space.debug_draw(self.options)
        #     space.step(1/FPS)
        #     clock.tick(FPS) 
        #     pygame.display.update()
            
        self.display.fill((255, 255, 255))  
        space.debug_draw(self.options)
        clock.tick(FPS) 
        pygame.display.update()

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
log_path = os.path.join('Train','Logs')
model = PPO("MlpPolicy", env, verbose = 1, tensorboard_log=log_path)
ppo_path = os.path.join('Train','Model Saved','shit')
callback_max_episodes = StopTrainingOnMaxEpisodes(max_episodes=1000, verbose=1)
model.learn(total_timesteps = 500000000000000000000000000, callback=(callback_max_episodes))
model.save(ppo_path)
# model = PPO.load(ppo_path, env)

# obs = env.reset()
# the = []
# while True:
#     env.render()
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     print("The score is : {}".format(rewards))

