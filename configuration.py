# -*- coding: utf-8 -*-
"""
Created on Thu Dec 03 14:29:07 2015

@author: Jak
"""
import math
import numpy

#define coord system origin as the centre of the bottom plate
#Find base plate attachment locations
#([0.,0.,2*math.pi/3,2*math.pi/3,4*math.pi/3,4*math.pi/3])
bAngles = numpy.array([0.,0.,2*math.pi/3,2*math.pi/3,4*math.pi/3,4*math.pi/3])
#bAngles = [math.radians(x) for x in bAngles]
bR = 2.52/math.sqrt(3)
bPos = numpy.zeros((6,3))
for i in range(0,6):
    bPos[i] = [bR*math.cos(bAngles[i]), bR*math.sin(bAngles[i]), 0]

#Platform attachment locations
pAngles = bAngles + numpy.array([-math.pi/3,math.pi/3,-math.pi/3,math.pi/3,-math.pi/3,math.pi/3])

#pAngles = [math.radians(x) for x in pAngles]
pR = 1.23/math.sqrt(3)
pPos = numpy.zeros((6,3))
for i in range(0,6):
    pPos[i] = [pR*math.cos(pAngles[i]), pR*math.sin(pAngles[i]), 0]

height = 1.83

legMin = [1.7272]*6
legMax = [2.1336]*6

#Base UV joint limits
A = [math.pi/4]*6
#Platform ball joint limits
B = [math.pi/2]*6

