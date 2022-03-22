#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 20 15:53:43 2022

@author: justoliver
"""

import pymunk, sys
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d
import pygame
from pygame.locals import *
import numpy as np
from PIL import Image
from pymunk.pygame_util import DrawOptions
import neat
import random
import time
import os
import visualize
import pickle

generation = 0
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

def get_delta(x1,x2,x3,y1,y2,y3): 
    
    x = np.array([x1-x2, y2-y1])
    y = np.array([x3-x2, y2-y3])

    l_x = np.sqrt(x.dot(x))
    ly = np.sqrt(y.dot(y))

    cos = x.dot(y)/(l_x * ly)

    return np.arccos(cos)

def get_angles(body):
    theta = get_theta(400, body.penbody.position[0], 400, body.penbody.position[1])
    phi = get_phi(body.penbody.position[0] + 150*np.sin(theta), body.swingbody.position[0],
                        body.penbody.position[1] + 150*np.cos(theta), body.swingbody.position[1], theta)
    iota = get_iota(body.swingbody.position[0] + 17.5*np.sin(theta+phi) + 20*np.cos(theta+phi),
                    body.legbody.position[0],
                    body.swingbody.position[1] + 25*np.cos(theta+phi) - 20*np.sin(theta+phi),
                    body.legbody.position[1], theta, phi)
    
    delta = get_delta(body.robobody.position[0], body.swingbody.position[0] + 17.5*np.sin(theta+phi),
                      body.swingbody.position[0], body.robobody.position[1],
                      body.swingbody.position[1] + 17.5*np.cos(theta+phi), body.swingbody.position[1])
    return (theta, phi, iota, delta)
    
class Swing_robot:
    def __init__(self):
        # upper pendulum
        self.penbody = pymunk.Body()
        self.penbody.position = (400 , 550)
        self.pen_a = (0, -150)
        self.pen_b = (0, 150)
        self.penshape = pymunk.Segment(self.penbody, self.pen_a, self.pen_b, 10)
        self.penshape.density = 0.01
        self.penshape.filter = pymunk.ShapeFilter(group=1)
        self.penshape.color = (0, 255, 0, 0)
        space.add(self.penbody, self.penshape)
        # swing
        self.swingbody = pymunk.Body()
        self.swingbody.position = (400, 717.5)
        self.s1 = pymunk.Segment(self.swingbody, (30, -17.5), (0, -17.5) , 10)
        self.s1.filter = pymunk.ShapeFilter(group = 1)
        self.s1.density = 0.05
        self.s2 = pymunk.Segment(self.swingbody, (0, -17.5), (0, 17.5), 10)
        self.s2.filter = pymunk.ShapeFilter(group = 1)
        self.s2.density = 0.05
        self.s3 = pymunk.Segment(self.swingbody, (-20, 25),(20, 25), 10)
        self.s3.filter = pymunk.ShapeFilter(group = 1)
        self.s3.density = 0.05
        space.add(self.swingbody, self.s1,self.s2,self.s3)
        # leg and foot
        self.legbody = pymunk.Body()
        self.legbody.position = (420,750)
        self.legradius = 10
        self.footradius = 10
        self.leg_a = (0, -15)
        self.leg_b = (0, 15)
        self.foot_a = (0,15)
        self.foot_b = (13,15)
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
        # robot body
        self.robobody = pymunk.Body()
        self.robobody.position = (386.218,704.138)
        self.roboradius = 7
        self.roboa = (-13.782,-30.8624)
        self.robob = (13.782,30.8624)
        self.roboshape = pymunk.Segment(self.robobody, self.roboa, self.robob, self.roboradius)
        self.roboshape.density = 0.01
        self.roboshape.filter = pymunk.ShapeFilter(group=1)
        self.roboshape.color = (255, 0, 0, 0)
        space.add(self.robobody, self.roboshape)
        # constraints
        self.pinjoint1 = pymunk.constraints.PinJoint(b0, self.penbody, (400,400), (0, -150)) #problem
        space.add(self.pinjoint1)
        self.pinjoint2 = pymunk.constraints.PinJoint(self.penbody, self.swingbody, (0, 150), (0, -17.5)) # problem
        space.add(self.pinjoint2)
        self.pinjoint3 = pymunk.constraints.PinJoint(self.swingbody, self.legbody, (20, 17.5), (0, -15))
        space.add(self.pinjoint3)
        self.pinjoint4 = pymunk.constraints.PinJoint(self.robobody, self. swingbody,(13.782,30.8624), (0, 17.5))
        space.add(self.pinjoint4)
        # rotation lim joint
        self.rotation_joint = pymunk.constraints.RotaryLimitJoint(self.penbody, self.swingbody, -np.pi/4, np.pi/4)
        self.rotation_joint.collide_bodies = True
        space.add(self.rotation_joint)  
        # robo swing motor
        # motor
    def add_legmotor(self, rate):
        self.legrate = rate
        self.legmotor = pymunk.SimpleMotor(self.swingbody, self.legbody, self.legrate)
        space.add(self.legmotor)
    def remove_legmotor(self):
        space.remove(self.legmotor)
    def add_bodymotor(self, rate):
        self.bodyrate = rate
        self.bodymotor = pymunk.SimpleMotor(self.swingbody, self.robobody, self.bodyrate)
        space.add(self.bodymotor)
    def remove_bodymotor(self):
        space.remove(self.bodymotor)
    
def eval_genomes(genome, config):
    cooldown = 0
    frames = 0
    max_theta = []
    thetas = []
    deltas = []
    iotas = []
    leg_states = []
    body_states = []
    # genome bunch of neural networks for each body
    nets = []
    ge = []
    bodies = []
    death_thetas = []
    stag_checker = []
    for genome_id, genome in genome:
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        nets.append(net)
        bodies.append(Swing_robot())
        genome.fitness = 0
        ge.append(genome)
    for body in bodies:
        body.add_bodymotor(0)
        body.add_legmotor(0)
        death_thetas.append(0)
        leg_states.append("stationary")
        body_states.append("stationary")
        
    pygame.display.set_caption("Double pendulum interactive Simulation")
    run = True
    global generation
    generation += 1
    while run:
        frames += 1
        cooldown += 1
                
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()
                sys.exit()
        
        # + abs(body.penbody.angular_velocity)
        display.fill((255, 255, 255))
        space.debug_draw(options)
        pygame.display.update()
        clock.tick(FPS)  # limiting frames per second to 120
        space.step(1/FPS)
        angles = []
        for body in bodies:
            angles.append(get_angles(body)) # iota is angle 3
        for x, body in enumerate(bodies):
            iota = angles[x][2]
            delta = angles[x][3]
            if leg_states[x] == "up" and iota < np.pi/2:
                pass
            elif leg_states[x] == "down" and iota > -np.pi/6:
                pass
            if body_states[x] == "forward" and delta < 1.17:
                pass
            elif body_states[x] == "backward" and delta > 0.42:
                pass
            else:
                outputs = nets[x].activate((angles[x][0], angles[x][1], angles[x][2],
                                            angles[x][3], body.penbody.angular_velocity,
                                            body.swingbody.angular_velocity, body.robobody.angular_velocity,
                                            body.legbody.angular_velocity))
                if outputs[0] > 0.25 and iota < np.pi/2 - 0.1:
                    body.remove_legmotor()
                    body.add_legmotor(5)
                    leg_states[x] = "up"
                elif outputs[0] < -0.25 and iota > (-np.pi/6 + 0.08):
                    body.remove_legmotor()
                    body.add_legmotor(-5)
                    leg_states[x] = "down"
                else:
                    body.remove_legmotor()
                    body.add_legmotor(0)
                    leg_states[x] = "stationary"
                if outputs[1] > 0.25 and delta > 0.42 +0.5:
                    body.remove_bodymotor()
                    body.add_bodymotor(-1)
                    body_states[x] = "backward"
                elif outputs[1] < -0.25 and delta < 1.17 - 0.05:
                    body.remove_bodymotor()
                    body.add_bodymotor(1)
                    body_states[x] = "forward"
                else:
                    body.remove_bodymotor()
                    body.add_bodymotor(0)
                    body_states[x] = "stationary"
                
            if iota >= np.pi/2 or iota <= -np.pi/6:
                body.remove_legmotor()
                body.add_legmotor(0)
                leg_states[x] = "stationary"
            if delta >= 1.17 or delta <= 0.42:
                body.remove_bodymotor()
                body.add_bodymotor(0)
                body_states[x] = "sttionary"
        for x, body in enumerate(bodies): # fitess function reward punishment
            theta = angles[x][0]
            thetas.append(theta)
            iota = angles[x][2]
            iotas.append(iota)
            delta = angles[x][3]
            deltas.append(delta)
            if frames != 1:
                delta_theta = abs(thetas[1] - thetas[0])
                thetas.pop(0)
#                delta_iota = abs(iotas[1] - iotas[0])
#                iotas.pop(0)
#                delta_delta = abs(deltas[1] - deltas[0])
#                deltas.pop(0)
                ge[x].fitness += np.rad2deg(delta_theta) + np.rad2deg(abs(theta))*100
            if frames == 1:
                max_theta.append(abs(theta))
                stag_checker.append(body_states[x])
            elif abs(np.rad2deg(theta)) > np.rad2deg(max_theta[x]) + 1:
                max_theta[x] = abs(theta)
                ge[x].fitness += abs(np.rad2deg(theta))*1000
                death_thetas[x] = 0
                
                if abs(np.rad2deg(theta)) >= 30:
                    pickle.dump(nets[x],open("winner.pkl", "wb"))
                    run = False
                    break
        if frames % 120 == 0:
            x = max_theta.index(min(max_theta))
            max_theta.pop(x)
            ge[x].fitness -= -1000
            nets.pop(x)
            ge.pop(x)
            body = bodies[x]
            bodies.pop(x)
            body.remove_legmotor()
            body.remove_bodymotor()
            space.remove(body.penbody, body.penshape, body.swingbody,
                         body.s1,body.s2,body.s3, body.legbody,
                         body.leg, body.foot, body.pinjoint1, body.pinjoint2,
                         body.pinjoint3, body.pinjoint4, body.rotation_joint,
                         body.robobody, body.roboshape)
            
#            if frames != 1:
#                if body_stag_checker[x] == body_states:
                    
            
            
                    
            
            # if maximum == None or maximum < ge[x].fitness:
            #     maximum = ge[x].fitness
            #     max_index = x
        ''' When 5 seconds have past or no bodies remain generation over '''
        if frames == 600 or len(bodies) < 0: 
            run = False
            for body in bodies:
                body.remove_legmotor()
                body.remove_bodymotor()
                space.remove(body.penbody, body.penshape, body.swingbody,
                             body.s1,body.s2,body.s3, body.legbody,
                             body.leg, body.foot, body.pinjoint1, body.pinjoint2,
                             body.pinjoint3, body.pinjoint4, body.rotation_joint,
                             body.robobody, body.roboshape)
            break

def replay_genome(config_path, genome_path="winner.pkl"):
    # Load requried NEAT config
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)

    # Unpickle saved winner
    with open(genome_path, "rb") as f:
        genome = pickle.load(f)

    # Convert loaded genome into required data structure
    genomes = [(1, genome)]

    # Call game with only the loaded genome
    eval_genomes(genomes, config)
    
if __name__ == '__main__':
    # Set configuration file
   config_path = ".\config-feedforward.txt"
   config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                               neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)

   # Create core evolution algorithm class
   p = neat.Population(config)

   # Add reporter for fancy statistical result
   p.add_reporter(neat.StdOutReporter(True))
   stats = neat.StatisticsReporter()
   p.add_reporter(stats)
   
   # Run NEAT
   winner = p.run(eval_genomes, 100)
   with open("winner.pkl", "wb") as f:
       pickle.dump(winner, f)
       f.close()
   replay_genome(config_path) 
   pygame.quit()
   sys.exit()   


