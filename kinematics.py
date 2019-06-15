import math
import numpy as np
import scipy
import scipy.spatial
class inverseKinematics():

    def __init__(self):
        #PAM parameters
        #self.min_pam_length = 1.72                          #minimum length of PAM in metres
        self.min_pam_length = 1.4                          #minimum length of PAM in metres
        self.max_pam_length = 2.13                           #maximum length of PAM in metres
        self.min_pam_pressure = 30                          #ambient pressure as read by valve controller
        self.max_pam_pressure = 150                         #~60psi as read by valve controller
        self.pam_pressures = np.zeros(6,dtype=int)

        #simulator parameters
        self.initial_platform_anchors = np.zeros((6,3))
        self.base_anchors = np.zeros((6,3))
        self.platform_base_distance = -1.83                                      #platform hangs 72" below base = 1.83m
        self.platform_side = 1.23                                                #side of the platform: 48" or 1.23
        self.platform_radius = self.platform_side/math.sqrt(3)                        #radius of circle inscribing platform
        self.base_side = 2.52                                                    #side length of base: 99.39" or 2.52m
        self.base_radius = self.base_side/math.sqrt(3)                                #radius of the circle inscribing the base
        self.init_platform_orientation = np.zeros(3)                             #pitch, roll, yaw of platform center
        self.init_platform_position = np.array([0.,0.,self.platform_base_distance])     #x,y,z of platform center
        self.base_position = np.zeros(3)                                         #x,y,z of base center
        self.base_orientation = np.zeros(3)                                      #pitch, roll, yaw of base center

        #anchor point relative angles - in radians
        self.base_anchor_angles = np.array([0.,0.,2*math.pi/3,2*math.pi/3,4*math.pi/3,4*math.pi/3])
        self.platform_anchor_angles = self.base_anchor_angles + [-math.pi/3,math.pi/3,-math.pi/3,math.pi/3,-math.pi/3,math.pi/3]
        self.platform_anchors = np.zeros((6,3))                                 #Current x,y,z of platform anchors
        self.pam_lengths = np.zeros(6)                                        #length of each PAM

        #assign x,y,x coordinates for base and platform based on radius and relative angles
        for i in range(0,6):
            self.initial_platform_anchors[i] = [self.platform_radius*math.cos(self.platform_anchor_angles[i]),self.platform_radius*math.sin(self.platform_anchor_angles[i]),self.platform_base_distance]
            self.base_anchors[i] = [self.base_radius*math.cos(self.base_anchor_angles[i]),self.base_radius*math.sin(self.base_anchor_angles[i]),0.]


        self.translation_vector = np.zeros(3)                             #linear displacement vector x,y,z
        self.rotational_vector = np.zeros(3)                                     #angular displacement vector pitch, roll, yaw

    def __call__(self, *args, **kwargs):
        return self.pam_pressures

    def rotational_matrix(self, rotational_vector):
        pitch = rotational_vector[0]
        roll = rotational_vector[1]
        yaw = rotational_vector[2]
        #print(rotational_vector)
        r_x = np.array([  [1., 0., 0.],  [0., math.cos(roll), -math.sin(roll)],  [0., math.sin(roll), math.cos(roll)]])
        r_y = np.array([  [math.cos(pitch), 0., math.sin(pitch)],  [0., 1., 0.],  [-math.sin(pitch), 0., math.cos(pitch)]])
        r_z = np.array([  [math.cos(yaw), -math.sin(yaw), 0.],  [math.sin(yaw), math.cos(yaw), 0.],  [0., 0., 1.]])
        return (r_z.dot(r_y).dot(r_x))

    def update_platform(self):
        for i in range(0,6):
            self.platform_anchors[i]=self.translation_vector + self.rotational_matrix(self.rotational_vector).dot(self.initial_platform_anchors[i])
            self.pam_lengths[i]=scipy.spatial.distance.euclidean(self.platform_anchors[i]+self.translation_vector,self.base_anchors[i])

            if (self.pam_lengths[i]>self.max_pam_length):
                self.pam_lengths[i] = self.max_pam_length
            elif (self.pam_lengths[i]<self.min_pam_length):
                self.pam_lengths[i] = self.min_pam_length

            pdiff = self.max_pam_pressure - self.min_pam_pressure
            ldiff = self.max_pam_length - self.min_pam_length
            self.pam_pressures[i] = int(self.min_pam_pressure+(self.max_pam_length-self.pam_lengths[i])*pdiff/ldiff)
            #self.pam_pressures[i] = int(self.pam_lengths[i]*pdiff/ldiff)
