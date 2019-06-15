import numpy as np
import math
from kinematics import inverseKinematics
import csv
import sys
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d
fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
ax = Axes3D(fig)
#results = np.zeros((117649,18))
x,y,z = np.loadtxt("xresults.csv",skiprows=1,delimiter=",",usecols=(0,1,2),unpack=True)
#pts = np.loadtxt("results.csv",skiprows=1,delimiter=",",usecols=(0,1,2))
print("loaded")
#X, Y, Z = axes3d.get_test_data(0.05)
#z,z = np.meshgrid(z,x,sparse=True)
#x,y = np.meshgrid(x,y,sparse=True)


#print(z.shape)
#ax.plot(x, y, z, label='parametric curve')
surf = ax.plot_trisurf(x,y,z, cmap=cm.jet, linewidth=0.1, antialiased=False)
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
