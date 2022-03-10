import pymunk, sys
from pymunk.pygame_util import *
from pymunk.vec2d import Vec2d
import pygame
from pygame.locals import *
import numpy as np
from PIL import Image
from pymunk.pygame_util import DrawOptions
from ActorCritic import Agent




def convert_coordinates(point):
    return int(point[0]), int(800-point[1])

def get_theta(x_h, x_1, y_h, y_1):
    return np.arctan2(x_1 - x_h, y_1 - y_h)

def get_phi(x1, x2, y1, y2, theta):
    return np.arctan2(x2 - x1, y2 - y1) - theta

def get_iota(x1, x2, y1, y2, theta, phi):
    return np.arctan2(x2 -x1, y2 - y1) - theta - phi


    
class Segment:
    def __init__(self, p0, a, b, radius=10, center_of_gravity = (0,0), density=0.01):
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
    def __init__(self, p0, a, b, c, d,radius=10, center_of_gravity = (0,0), density=0.01):
        self.body = pymunk.Body()
        self.body.position = p0
        self.radius = radius
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.body.center_of_gravity = center_of_gravity
        self.leg= pymunk.Segment(self.body, self.a, self.b , radius=radius)
        self.leg.filter = pymunk.ShapeFilter(group = 1)
        self.leg.density = density
        self.leg.color = (0, 255, 0, 0)
        self.foot= pymunk.Segment(self.body, self.c, self.d, radius=radius)
        self.foot.filter = pymunk.ShapeFilter(group = 1)
        self.foot.density = density
        self.foot.filter = pymunk.ShapeFilter(group=1)
        self.foot.color = (0, 255, 0, 0)
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
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        joint.collide_bodies = collide
        space.add(joint)

class PinJoint:
    def __init__(self, b, b2, a=(0, 0), a2=(0, 0)):
        joint = pymunk.constraints.PinJoint(b, b2, a, a2)
        space.add(joint)

class Swing_body:
    def __init__(self,p0, vx1,vy1,vx2,vy2,vx3,vy3, a, b, radius=10, center_of_gravity = (0,0), density=0.05):
        self.body = pymunk.Body()
        self.body.position = p0
        s1 = pymunk.Segment(self.body, vx1, vy1 , radius)
        s1.filter = pymunk.ShapeFilter(group = 1)
        s1.density = density
        s2 = pymunk.Segment(self.body, vx2, vy2, radius)
        s2.filter = pymunk.ShapeFilter(group = 1)
        s2.density = density
        s3 = pymunk.Segment(self.body, vx3,vy3, radius)
        s3.filter = pymunk.ShapeFilter(group = 1)
        s3.density = density
        s4 = pymunk.Segment(self.body, a, b, radius)
        s4.filter = pymunk.ShapeFilter(group = 1)
        s4.density = density
        space.add(self.body, s1,s2,s3,s4)

def angle_reached(theta, high_score):
    if len(high_score) == 0:
        high_score.append(theta)
    elif high_score[0] < abs(theta):
        high_score[0] = abs(theta)
    highest_score = high_score[0]
    return high_score


def main(agent, score_history, best_score):
    pygame.display.set_caption("Double pendulum interactive Simulation")
    high_score = []
    x = "const motor"
    constant_motor = Simplemotor(swing.body, leg.body, 0)
    done = False
    timestep = 0
    
    theta = 0
    phi = 0
    iota = 0
    prev_theta = 0
    prev_phi = 0
    prev_iota = 0
    score = 0
    state = np.zeros(6)
    datastore = np.array([[*state, 0]])
    vel = 0
    max_abs_theta = 0
    
    while True:
        xh, yh = (400,400)
        x1, y1 = segment.body.position[0], segment.body.position[1]
        theta = get_theta(xh, x1, yh, y1)
        x2, y2 = segment.body.position[0] + 100*np.sin(theta) , segment.body.position[1] + 100*np.cos(theta) 
        x3, y3 = swing.body.position[0], swing.body.position[1]
        phi = get_phi(x2, x3, y2, y3, theta)
        x4, y4 = swing.body.position[0] + 25*np.sin(theta+phi) + 20*np.cos(theta+phi), swing.body.position[1] + 25*np.cos(theta+phi) - 20*np.sin(theta+phi) 
        x5, y5 = leg.body.position[0], leg.body.position[1]
        iota = get_iota(x4, x5, y4, y5, theta, phi)
        const_angvel = swing.body.angular_velocity
        #print(len(space.constraints))
        diff_theta = theta - prev_theta
        diff_phi = phi - prev_phi
        diff_iota = iota - prev_iota
        prev_theta = theta
        prev_phi = phi
        prev_iota = iota
        max_abs_theta = max(max_abs_theta, abs(theta))
        
        state = np.array([theta, diff_theta, phi, diff_phi, iota, diff_iota])
        if timestep > 2000:
            done = True
        action = agent.choose_action(state)
        reward = (np.rad2deg(theta) ** 2) + (np.rad2deg(diff_theta) ** 2)
        if action != 1:
            reward = -reward / 10
        datastore = np.append(datastore, np.array([[*state, reward]]), axis = 0)
        score += reward
        
        for event in pygame.event.get():  # checking for user input
            if event.type == pygame.QUIT:
                done = True
            if event.type == KEYDOWN:
                if event.key == K_RIGHT:
                    vel = min(vel + 1, 5)
                    #if iota < np.pi/2 and x == "const motor":
                    #    constant_motor.remove()
                    #    x = "positive motor"
                    #    pos_motor = Simplemotor(swing.body, leg.body, 5)
                    #elif iota < np.pi/2 and x == "negetive motor":
                    #    neg_motor.remove()
                    #    x = "positive motor"
                    #    pos_motor = Simplemotor(swing.body, leg.body, 5)
                elif event.key == K_LEFT:
                    vel = max(vel - 1, -5)
                    #if iota > 0 and x == "const motor":
                    #    constant_motor.remove()
                    #    x = "negetive motor"
                    #    neg_motor = Simplemotor(swing.body, leg.body, -5)
                    #elif iota > 0 and x == "positive motor":
                    #    pos_motor.remove()
                    #    x = "negetive motor"
                    #    neg_motor = Simplemotor(swing.body, leg.body, -5)
                elif event.key == K_UP:
                    if vel > 0:
                        vel = max(vel - 1, 0)
                    if vel < 0:
                        vel = min(vel + 1, 0)
                    #if x == "positive motor":
                    #    pos_motor.remove()
                    #    x = "const motor"
                    #   constant_motor = Simplemotor(swing.body, leg.body, 0)
                    #elif x == "negetive motor":
                    #    neg_motor.remove()
                    #    x = "const motor"
                    #    constant_motor = Simplemotor(swing.body, leg.body, 0)
        
        if action == 0:
            vel = max(vel - 1, -5)
        elif action == 1:
            if vel > 0:
                vel = max(vel - 1, 0)
            if vel < 0:
                vel = min(vel + 1, 0)
        elif action == 2:
            vel = min(vel + 1, 5)
        
        constant_motor.remove()
        constant_motor = Simplemotor(swing.body, leg.body, vel) 
        
        
        if done:
            pygame.quit()
            break
                        
        #if iota >= (np.pi/2-0.1) and x == "positive motor":
        if iota >= (np.pi/2-0.1) and vel > 0:
            vel = 0
            #pos_motor.remove()
            constant_motor.remove()
            constant_motor = Simplemotor(swing.body, leg.body, 0)
            x = "const motor"
        #if iota <= (-np.pi/6 + 0.08) and x == "negetive motor":
        if iota <= (-np.pi/6 + 0.08) and vel < 0:
            vel = 0
            #neg_motor.remove()
            constant_motor.remove()
            constant_motor = Simplemotor(swing.body, leg.body, 0)
            x = "const motor"
        high_score = angle_reached(theta, high_score)
        display.fill((255, 255, 255))
        space.debug_draw(options)
        pygame.display.update()
        clock.tick(FPS)  # limiting frames per second to 120
        space.step(1/FPS)
        timestep += 1
    
    print(max_abs_theta)
    for i in range(datastore.shape[0] - 1):
        obs = datastore[i,0:6]
        rew = datastore[i+1,6]
        obs_ = datastore[i+1,0:6]
        if i == datastore.shape[0] - 1:
            done = True
        else:
            done = False
        if update_model:
            agent.learn(obs, rew, obs_, done)
    
    score_history.append(score)
    avg_score = np.mean(score_history[-10:])
    if avg_score > best_score:
        best_score = avg_score
        if update_model:
            agent.save_models()
    


if __name__ == "__main__":
    agent = Agent(alpha=1e-5, n_actions=3)
    global best_score
    best_score = 0
    score_history = []
    load_checkpoint = False
    update_model = True
    if load_checkpoint:
        agent.load_models()
    
    for x in range(5):
        size = 800, 800
        display = pygame.display.set_mode((size))
        options = DrawOptions(display)
        clock = pygame.time.Clock()
        space = pymunk.Space()
        space.gravity = 0, 981
        b0 = space.static_body
        b1 = space.static_body
        FPS = 120
        
        # b1 = measurement_body()

        hinge_point1 = (0, -100) # seg 1
        hinge_point2 = (0, 100)
        swing_body = (400, 625)
        swing_top1 = (30, -25)
        swing_top2 = (0, -25)
        swing_mid1 = (0, -25)
        swing_mid2 = (0, 25)
        swing_bottom1 = (-20, 25)
        swing_bottom2 = (20, 25)
        hinge_point3 = (0, -30) # seg 2
        hinge_point4 = (0, 30)

        "Pymunk Bodies"

        segment = Segment((400 , 500), hinge_point1 , hinge_point2)
        leg = Leg((420,680), hinge_point3, hinge_point4, (0,30), (15,30), density= 0.05)
        swing = Swing_body(swing_body, swing_top1,swing_top2, swing_mid1, swing_mid2, swing_bottom1, swing_bottom2,(0,25),(-20,-25))
        PinJoint(swing.body, leg.body, swing_bottom2, hinge_point3)
        PinJoint(segment.body, swing.body, hinge_point2, swing_mid1)
        PinJoint(b0, segment.body, (400,400), hinge_point1)
        rotation_lim = RotaryLimitJoint(segment.body,swing.body , -np.pi/4, np.pi/4)

        main(agent, score_history, best_score)
    