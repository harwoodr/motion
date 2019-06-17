import numpy as np
import math
from kinematics import inverseKinematics
import csv
import sys
import time
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#assumptions
#+/-yaw <= +/-60 degrees
#+/-pitch and +/-roll <= +/-45 degrees
#-1.83 <= z <= -1.23 (diff of 60cm)
#+/-x and +/-y <=60cm
#given
#1.72 <= muscle length <= 2.13

#pseudocode
#z from -1.83 to -1.23 (5cm inc: 24 values)
#  x from -60 to +60 (5cm inc: 24 values)
#    y from -60 to +60 (5cm inc: 24 values)
#      yaw from -60 to +60 (5 deg inc: 24 values)
#       pitch from -45 to +45 (5 deg inc: 18 values)
#         roll from -45 to +45 (5 deg inc: 18 values)
#           calc muscles
#           if any muscle < 1.72 or >2.13 throw out
#           else mark as good space
#107495424 rows
#z from -1.83 to -1.23 (10cm inc: 7 values)
#  x from -30 to +30 (10cm inc: 7 values)
#    y from -30 to +30 (10cm inc: 7 values)
#      yaw from -30 to +30 (10 deg inc: 7 values)
#       pitch from -30 to +30 (10 deg inc: 7 values)
#         roll from -30 to +30 (10 deg inc: 7 values)
#           calc muscles
#           if any muscle < 1.72 or >2.13 throw out
#           else mark as good space
#753571 rows
ik = inverseKinematics()
#results = np.zeros((107495424,18))
#results = np.zeros((117649,18))
start = time.monotonic()
rows = 10000
counter = 0

results = np.empty((1,3))
#liner = np.empty((1,9))
#print(results.shape)
print(ik.max_pam_length)
print(ik.min_pam_length)
for z in range(0,61):
    for y in range(-30,31):
        for x in range(-30,31):
            ik.translation_vector=np.array([x/100,y/100,z/100])
            ik.update_platform()
            if (np.amax(ik.pam_lengths)<ik.max_pam_length and np.amin(ik.pam_lengths)>ik.min_pam_length and ik.translation_vector[2]!=0):
            #if (np.amax(ik.pam_lengths)<=ik.max_pam_length and np.amin(ik.pam_lengths)>=ik.min_pam_length):
                #results[counter]=[ik.translation_vector[0],ik.translation_vector[1],ik.translation_vector[2],math.degrees(ik.rotational_vector[0]),math.degrees(ik.rotational_vector[1]),math.degrees(ik.rotational_vector[2]),ik.pam_lengths[0],ik.pam_lengths[1],ik.pam_lengths[2],ik.pam_lengths[3],ik.pam_lengths[4],ik.pam_lengths[5],ik.pam_pressures[0],ik.pam_pressures[1],ik.pam_pressures[2],ik.pam_pressures[3],ik.pam_pressures[4],ik.pam_pressures[5]]
                #np.liner = np.array([ik.translation_vector[0],ik.translation_vector[1],ik.translation_vector[2],ik.pam_lengths[0],ik.pam_lengths[1],ik.pam_lengths[2],ik.pam_lengths[3],ik.pam_lengths[4],ik.pam_lengths[5]])
                results=np.append(results,[ik.translation_vector],axis=0)
            #ik.reset_platform()
            counter += 1
            if counter % rows == 0:
                #print(results[counter-1])
                print(counter,end=" - ")
                print(time.monotonic() - start,end=" seconds - ")
                print(counter/(time.monotonic()-start),end=" rows/s\n")
                print(ik.translation_vector,end=" - ")
                print()
#print(results)
np.savetxt("xresults.csv",results,delimiter=",")
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')

#ax.plot(xa, ya, za, label='parametric curve')
#plt.show()


