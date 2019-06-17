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
from scipy.spatial import ConvexHull
from scipy.spatial import Delaunay

fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
ax = Axes3D(fig)
#results = np.zeros((117649,18))
#x,y,z = np.loadtxt("xresults.csv",skiprows=1,delimiter=",",usecols=(0,1,2),unpack=True)
pts = np.loadtxt("xresults.csv",skiprows=1,delimiter=",",usecols=(0,1,2))
print("loaded")
#X, Y, Z = axes3d.get_test_data(0.05)
#z,z = np.meshgrid(z,x,sparse=True)
#x,y = np.meshgrid(x,y,sparse=True)
#hull = ConvexHull(pts)
#hull = Delaunay(pts)

#print(z.shape)
#ax.plot(x, y, z, label='parametric curve')
#surf = ax.plot_trisurf(pts[hull.vertices,0], pts[hull.vertices,1], pts[hull.vertices,2], cmap=cm.jet, linewidth=0.1, antialiased=False)
#surf = ax.plot_trisurf(x, y, z, cmap=cm.jet, linewidth=0.1, antialiased=False)
#fig.colorbar(surf, shrink=0.5, aspect=5)
#ax = plt.axes(projection='3d')
#ax.scatter(pts[hull.vertices,0], pts[hull.vertices,1], pts[hull.vertices,2], c=pts[hull.vertices,2], cmap='viridis', linewidth=0.5);
ax.scatter(pts[:,0], pts[:,1], pts[:,2], c=pts[:,2], cmap='viridis', linewidth=0.5);
plt.show()
