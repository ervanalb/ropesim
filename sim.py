#!/usr/bin/env python

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import math
import numpy
import time

quadric = gluNewQuadric()
epsilon = 0.00001

screen=(2000,1000)

def material():
   mat_specular = (1.0, 1.0, 1.0, 1.0)
   mat_shininess = 50.0
   light_position = (1.0, 1.0, 1.0, 0.0)
   glClearColor(0.0, 0.0, 0.0, 0.0)
   glShadeModel(GL_SMOOTH)

   glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular)
   glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess)
   glLightfv(GL_LIGHT0, GL_POSITION, light_position)

   glEnable(GL_LIGHTING)
   glEnable(GL_LIGHT0)
   glEnable(GL_DEPTH_TEST)

def view():
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45.0,800/600.0,0.1,100.0)    #setup lens
    glTranslatef(0.0, 0.0, -30.0)               #move back
    glRotatef(25, 1, 0, 0)                      #orbit higher
    glTranslatef(0.0, 2.0, 0.0)
    glMatrixMode(GL_MODELVIEW)

def drawrope(pts):
    up = numpy.array([0, 0, 1])
    for pt1, pt2 in zip(pts[:-1], pts[1:]):
        z = pt2 - pt1
        x = numpy.cross(z, up)
        mag = numpy.linalg.norm(x)
        x = x / mag
        y = numpy.cross(z, x)
        mag = numpy.linalg.norm(y)
        y = y / mag
        mat = numpy.identity(4)
        mat[0, 0:3] = x
        mat[1, 0:3] = y
        mat[2, 0:3] = z
        mat[3, 0:3] = pt1
        glPushMatrix()
        glMultMatrixf(mat)
        gluCylinder(quadric, 0.1, 0.1, 1, 12, 1)
        glPopMatrix()

damping_c = 0.1
structural_spring_x = .1
structural_spring_k = 1500
bending_spring_k = 0
mass_per_length = 0.5
g = 100

n = 20
init_xs = numpy.array([[i / math.sqrt(2), i / math.sqrt(2), 0] for i in numpy.linspace(0, n * structural_spring_x, n)])
init_vs = numpy.zeros((len(init_xs),3))

def accels(x, v):
    forces = numpy.empty((0,3))
    for i in range(len(x)):
        structural_spring = numpy.zeros(3)
        bending_spring = numpy.zeros(3)

        if i > 0:
            vec = (x[i - 1] - x[i])
            mag = numpy.linalg.norm(vec)
            force = (mag - structural_spring_x) * structural_spring_k * structural_spring_x
            structural_spring += force * vec / mag

        if i < len(x) - 1:
            vec = (x[i + 1] - x[i])
            mag = numpy.linalg.norm(vec)
            force = (mag - structural_spring_x) * structural_spring_k * structural_spring_x
            structural_spring += force * vec / mag

        if i > 0 and i < len(x) - 1:
            vec1 = x[i] - x[i-1]
            vec2 = x[i+1] - x[i]
            n1 = numpy.linalg.norm(vec1)
            n2 = numpy.linalg.norm(vec2)
            sintheta = numpy.linalg.norm(numpy.cross(vec1, vec2)) / (n1 * n2)
            curvature = sintheta / (n1 + n2)
            dir_vec = (x[i-1] + x[i+1]) / 2 - x[i]
            mag = numpy.linalg.norm(dir_vec)
            if mag != 0:
                dir_vec /= mag
                bending_spring += bending_spring_k * curvature * dir_vec

        damping = v[i] * damping_c * structural_spring_x

        gravity = numpy.array([0, -g * structural_spring_x * mass_per_length, 0])
        total_force = structural_spring + gravity + damping + bending_spring

        forces = numpy.vstack((forces, total_force))
    return forces / (structural_spring_x * mass_per_length)

def tweak_positions(x):
    x[0] = numpy.array([0, 0, 0])
    return x

def tweak_velocities(x, v):
    return v

def step(dt, x, v):
    # http://doswa.com/2009/01/02/fourth-order-runge-kutta-numerical-integration.html
    x1 = tweak_positions(x)
    v1 = tweak_velocities(x, v)
    a1 = accels(x1, v1)

    x2 = tweak_positions(x + 0.5 * v1 * dt)
    v2 = tweak_velocities(x2, v + 0.5 * a1 * dt)
    a2 = accels(x2, v2)

    x3 = tweak_positions(x + 0.5 * v2 * dt)
    v3 = tweak_velocities(x3, v + 0.5 * a2 * dt)
    a3 = accels(x3, v3)

    x4 = tweak_positions(x + v3 * dt)
    v4 = tweak_velocities(x4, v + a3 * dt)
    a4 = accels(x4, v4)

    xf = x + (dt / 6.0) * (v1 + 2 * v2 + 2 * v3 + v4)
    vf = v + (dt / 6.0) * (a1 + 2 * a2 + 2 * a3 + a4)

    x = tweak_positions(xf)
    v = tweak_velocities(xf, vf)

    return (x, v)

def main():

    pygame.init()
    pygame.display.set_mode(screen, OPENGL|DOUBLEBUF)

    material()

    view()

    xs = init_xs
    vs = init_vs

    deg = 0
    lt = time.time()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        glLoadIdentity()
        #deg += 1
        #glRotatef(deg, 0, 1, 0)

        t = time.time()
        dt = lt - t
        lt = t
        xs, vs = step(dt, xs, vs)
        drawrope(xs)

        pygame.display.flip()
        pygame.time.wait(10)

if __name__ == '__main__': main()
