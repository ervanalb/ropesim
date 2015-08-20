#!/usr/bin/env python

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

import math
import numpy

quadric = gluNewQuadric()
epsilon = 0.00001


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
    glTranslatef(0.0, 0.0, -10.0)                #move back
    glRotatef(25, 1, 0, 0)                      #orbit higher
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

pointlist = numpy.array([[math.sin(i), i / 10, math.cos(i)] for i in numpy.linspace(0, 10, 100)])
#pointlist = numpy.array([[0, 0, 0], [0, 0.5, 0], [0.5, 0.5, 0]])

def main():
    pygame.init()
    pygame.display.set_mode((800,600), OPENGL|DOUBLEBUF)

    material()

    view()

    deg = 0
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

        glLoadIdentity()
        deg += 1
        glRotatef(deg, 0, 1, 0)

        drawrope(pointlist)
        pygame.display.flip()
        pygame.time.wait(10)


if __name__ == '__main__': main()
