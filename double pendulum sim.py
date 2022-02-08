import pygame
import pymunk
import sys


pygame.init()
 
display = pygame.display.set_mode((800,800))
clock = pygame.time.Clock()
space = pymunk.Space()
space.gravity = (0, -900)
FPS = 120

def convert_coordinates(point):
    return int(point[0]), int(800-point[1])

class Ball():
    def __init__(self, x, y):
        self.body = pymunk.Body()
        self.body.position = x, y
        self.shape = pymunk.Circle(self.body, 10)
        self.shape.density = 1
        self.shape.elasticity = 1
        space.add(self.body, self.shape) # for dynamic bodies we have to add to pymunk space
    def draw(self):
        pygame.draw.circle(display, (255, 0, 0), convert_coordinates(self.body.position), 10)


def swinging_up(position):
    
    
    
    
class Leg():
    def __init__(self, x, y):
        self.body = pymunk.Body()
        self.body.position = [x, y]
        self.shape = pymunk.Circle(self.body, 10)
        self.shape.density = 1
        self.shape.elasticity = 1
        space.add(self.body, self.shape) # for dynamic bodies we have to add to pymunk space
    def draw(self):
        pygame.draw.circle(display, (255, 0, 0), convert_coordinates(self.body.position), 10)
        
class String():
    def __init__(self, body1, attachment, identifier="body"):
        self.body1 = body1
        if identifier == "body":
            self.body2 = attachment
        elif identifier == "position":
            self.body2 = pymunk.Body(body_type=pymunk.Body.STATIC)
            self.body2.position = attachment
        joint = pymunk.PinJoint(self.body1, self.body2)
        space.add(joint) # for dynamic bodies we have to add to pymunk space
    def draw(self):
        pos1 = convert_coordinates(self.body1.position)
        pos2 = convert_coordinates(self.body2.position)
        pygame.draw.line(display, (0, 0 , 0), pos1, pos2, 2)

ball_1 = Ball(400, 400)
ball_2 = Ball(100, 100)
string_1 = String(ball_1.body, (400, 800), "position")
string_2 = String(ball_1.body, ball_2.body)

def game():
    pygame.display.set_caption("Double pendulum interactive Simulation")
    while True:
        for event in pygame.event.get(): # checking for user input
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
        pygame.display.update() #rendering frame
        display.fill((255,255,255)) # background colour
        ball_1.draw()
        ball_2.draw()
        string_1.draw()
        string_2.draw()
        pygame.display.update()
        clock.tick(FPS) # limiting frames per second to 120
        space.step(1/FPS)
        
game()
pygame.quit()
