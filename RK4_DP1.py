#!/usr/bin/env python
# coding: utf-8

# In[1]:


import math
import numpy
import pygame


# In[2]:


class DoublePendulumLagrangian:

    def __init__(self, g, m1, m2, t1, t2, w1, w2, L1, L2):
        """
        Constructs a double pendulum simulator based on its
        Euler-Lagrange equations. Bob #1 is the one attached to the
        fixed pivot.
        g - The gravitational acceleration.
        m1 - The mass of bob #1.
        m2 - The mass of bob #2.
        t1 - The initial angle of bob #1.
        t2 - The initial angle of bob #2.
        w1 - The initial angular velocity of bob #1.
        w2 - The initial angular velocity of bob #2.
        L1 - The length of the rod for bob #1.
        L2 - The length of the rod for bob #2.
        """
        self.g = g
        self.m1 = m1
        self.m2 = m2
        self.t1 = t1
        self.t2 = t2
        self.w1 = w1
        self.w2 = w2
        self.L1 = L1
        self.L2 = L2

    def potential_energy(self):
        """Computes the potential energy of the system."""

        m1 = self.m1
        t1 = self.t1
        L1 = self.L1
        m2 = self.m2
        t2 = self.t2
        L2 = self.L2

        g = self.g

        # compute the height of each bob
        y1 = -L1 * math.cos(t1)
        y2 = y1 - L2 * math.cos(t2)

        return m1 * g * y1 + m2 * g * y2

    def kinetic_energy(self):
        """Computes the kinetic energy of the system."""

        m1 = self.m1
        t1 = self.t1
        w1 = self.w1
        L1 = self.L1
        m2 = self.m2
        t2 = self.t2
        w2 = self.w2
        L2 = self.L2

        # compute the kinetic energy of each bob
        K1 = 0.5 * m1 * (L1 * w1)**2
        K2 = 0.5 * m2 * ((L1 * w1)**2 + (L2 * w2)**2 +
                         2 * L1 * L2 * w1 * w2 * math.cos(t1 - t2))

        return K1 + K2

    def mechanical_energy(self):
        """
        Computes the mechanical energy (total energy) of the
        system.
        """

        return self.kinetic_energy() + self.potential_energy()

    def lagrange_rhs(self, t1, t2, w1, w2):
        """
        Computes the right-hand side of the Euler-Lagrange equations
        for the double pendulum and returns it as an array.
        t1 - The angle of bob #1.
        t2 - The angle of bob #2.
        w1 - The angular velocity of bob #1.
        w2 - The angular velocity of bob #2.
        """
            
        m1 = self.m1
        L1 = self.L1
        m2 = self.m2
        L2 = self.L2

        g = self.g

        a1 = (L2 / L1) * (m2 / (m1 + m2)) * math.cos(t1 - t2)
        a2 = (L1 / L2) * math.cos(t1 - t2)

        f1 = -(L2 / L1) * (m2 / (m1 + m2)) * (w2**2) * math.sin(t1 - t2) -             (g / L1) * math.sin(t1)
        f2 = (L1 / L2) * (w1**2) * math.sin(t1 - t2) - (g / L2) * math.sin(t2)

        g1 = (f1 - a1 * f2) / (1 - a1 * a2)
        g2 = (f2 - a2 * f1) / (1 - a1 * a2)
        

        return numpy.array([w1, w2, g1, g2])

    def time_step(self, dt):
        """
        Advances one time step using RK4 (classical Runge-Kutta
        method).
        """

        m1 = self.m1
        t1 = self.t1
        w1 = self.w1
        L1 = self.L1
        m2 = self.m2
        t2 = self.t2
        w2 = self.w2
        L2 = self.L2

        # y is an array with the generalized coordinates (angles +
        # angular velocities)
        y = numpy.array([t1, t2, w1, w2])

        # compute the RK4 constants
        k1 = self.lagrange_rhs(*y)
        k2 = self.lagrange_rhs(*(y + dt * k1 / 2))
        k3 = self.lagrange_rhs(*(y + dt * k2 / 2))
        k4 = self.lagrange_rhs(*(y + dt * k3))

        # compute the RK4 right-hand side
        R = 1.0 / 6.0 * dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # update the angles and angular velocities
        self.t1 += R[0]
        self.t2 += R[1]
        self.w1 += R[2]
        self.w2 += R[3]


# # Drawing function

# In[3]:


def draw(S,window,Nx,Ny,dt):
    """    
    Draws the double pendulum system on a window.
    S - The double pendulum object.
    window - The window where the double pendulum will be shown.
    Nx - The window width (in pixels).
    Ny - The window height (in pixels).
    dt - The simulation time step.
    """
    
    m1 = S.m1  # m1 - The mass of bob #1.
    m2 = S.m2  # m2 - The mass of bob #2
    t1 = S.t1  # t1 - The initial angle of bob #1.
    t2 = S.t2  # t2 - The initial angle of bob #2.
    L1 = S.L1  # L1 - The length of the rod for bob #1.
    L2 = S.L2  # L2 - The length of the rod for bob #2

    # radius (in pixels) of each bob (min/max: 3/12 pixels)
    R1 = max(3, int(12 * (m1 / (m1 + m2))))
    R2 = max(3, int(12 * (m2 / (m1 + m2))))

    # length (in pixels) of each rod
    P1 = 0.85 * min(Nx / 2, Ny / 2) * (L1 / (L1 + L2))
    P2 = 0.85 * min(Nx / 2, Ny / 2) * (L2 / (L1 + L2))

    # positions (in (pixels,pixels)) of each bob
    X0 = numpy.array([int(Nx / 2), int(Ny / 2)])
    X1 = X0 + numpy.array([int(P1 * math.sin(t1)), int(P1 * math.cos(t1))])
    X2 = X1 + numpy.array([int(P2 * math.sin(t2)), int(P2 * math.cos(t2))])

    # color: rods and bobs
    color_L1 = (255, 255, 255)
    color_L2 = (128, 128, 128)
    color_m1 = (255, 0, 0)
    color_m2 = (0, 0, 255)

    # clear the window
    window.fill((0, 0, 0))

    # draw the rods and the bobs
    pygame.draw.line(window, color_L1, X0, X1, 3)
    pygame.draw.line(window, color_L2, X1, X2, 3)
    pygame.draw.circle(window, color_m1, X1, int(R1))
    pygame.draw.circle(window, color_m2, X2, int(R2))

    # write the time step value on the window
    myfont = pygame.font.SysFont("Arial", 15)
    label = myfont.render("dt = %.3g" % dt, 1, (128, 128, 128))
    window.blit(label, (10, 10))

    # update the screen
    pygame.display.flip()


# In[4]:


import getopt
import os
import sys


# In[5]:


def main():

    # default simulation parameter values
    g = 9.81
    dt = 1/30
    m1 = 5
    m2 = 10
    t1 = 1
    t2 = 2
    w1 = w2 = 0.0
    L1 = 1.5
    L2 = 0.35

    # default window dimensions
    Nx = Ny = 500
    verbose = False
    
    S = DoublePendulumLagrangian(g, m1, m2, t1, t2, w1, w2, L1, L2)
    
    # E0 = initial mechanical energy of the system
    E0 = S.mechanical_energy()
    step = 0
    
    # maximum energy change (compared to E0): too large => unstable simulation
    max_dE = 0
    
    
    pygame.init()
    clock = pygame.time.Clock()
    window = pygame.display.set_mode((Nx, Ny), pygame.RESIZABLE)
    pygame.display.set_caption("double pendulum")
    
    # keep running the simulation until the user closes the window
    while True:

        # redraw the double pendulum at a maximum rate of 25 fps
        draw(S, window, Nx, Ny, dt)
        clock.tick(30)
        if verbose:
            Et = S.mechanical_energy()
            max_dE = max(abs(Et - E0), max_dE)
            line = "[%u] t = %f   Et = %f   E0 = %f   |Et - E0| = %f   " +                 "|Et - E0|_max = %f\n"
            sys.stdout.write(line % (step, step * dt, Et, E0, abs(Et - E0), max_dE))
            """
            # check window events: quit, resize, key presses
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return False
                elif event.type == pygame.VIDEORESIZE:
                    (Nx, Ny) = event.size
                    window = pygame.display.set_mode((Nx, Ny), pygame.RESIZABLE)
                elif event.type == pygame.KEYDOWN:
                    if event.unicode == u"v":
                        verbose = not verbose

            # the up and down arrow keys increase and decrease dt respectively
            pressed_keys = pygame.key.get_pressed()
            if pressed_keys[273]:
                dt *= 1.05
            if pressed_keys[274]:
                dt /= 1.05"""
        else:
            line = "%f,%f,%f,%f\n"
            sys.stdout.write(line % (step * dt, S.t1, S.t2, S.mechanical_energy()))
            if step == 1000:
                break

        # advance one time step
        S.time_step(dt)
        step += 1
    
    


# In[6]:


if __name__ == "__main__":
    main()


# In[ ]:




