import numpy as np
from scipy import signal
from scipy import constants
import math
from filter import RealtimeFilter



class motionCueing():
    def __init__(self):
        self.tGain = 4
        self.rGain = 1
        self.yGain = 4
        self.freq = 20
        self.omega = 25
        self.zeta = 1
        self.gVector = np.array([[0, 0, constants.g]]).T
        self.max_translational_acceleration = 10                                     #in m/s^2
        self.max_rotational_velocity = np.deg2rad(30)                               #in deg/s
        self.max_rotational_acceleration = math.sin(np.deg2rad(30)) * constants.g   #in deg/s^2

        self.translational_vector = np.zeros(3)
        self.rotational_vector = np.zeros(3)

        #max degrees/second = +/-55 from simtools - converted to radians/s
        self.aconversion = 55/128*math.pi/180
        #max sway/surge = +/-3G - expressed as -127 to +128 from simtools - converted m/s^2
        self.tconversion = 3/128*constants.g
        #max heave = +/-5G - expressed as -127 to +128 from simtools - converted to m/s^2
        self.yconversion = 5/128*constants.g

        #filters
        #tuning will require individial omega/zeta parameters for filters...
        #surge and pitch filters

        self.surge_hp2 = self.secondhp_filter(self.zeta,self.omega)
        self.surge_hp1 = self.firsthp_filter(self.omega)
        self.surge_dint = self.dint_filter()
        self.pitch_hp2 = self.secondhp_filter(1,5)
        self.pitch_sint = self.sint_filter()
        self.sp_tilt_lp = self.firstlp_filter(self.omega)

        #sway and roll filters
        self.sway_hp2 = self.secondhp_filter(self.zeta,self.omega)
        self.sway_hp1 = self.firsthp_filter(self.omega)
        self.sway_dint = self.dint_filter()
        self.roll_hp2 = self.secondhp_filter(1,5)
        self.roll_sint = self.sint_filter()
        self.sr_tilt_lp = self.firstlp_filter(self.omega)

        #heave and yaw filters
        self.heave_hp2 = self.secondhp_filter(self.zeta,self.omega)
        self.heave_hp1 = self.firsthp_filter(self.omega)
        self.heave_dint = self.dint_filter()
        self.yaw_hp2 = self.secondhp_filter(self.zeta,self.omega)
        self.yaw_sint = self.sint_filter()

        self.xIn = 0.
        self.yIn = 0.
        self.zIn = 0.
        self.pitchIn = 0.
        self.rollIn = 0.
        self.yawIn = 0.

        self.xOut = 0.
        self.yOut = 0.
        self.zOut = 0.
        self.pitchOut = 0.
        self.rollOut = 0.
        self.yawOut = 0.


    #high pass filter - third order
    def thirdhp_filter(self, z,w):
        b=[1,0,0,0]
        a=[1,(2*z*w + w),(w**2 + 2*z*w**2), w**3]
        return RealtimeFilter(*signal.bilinear(b,a,fs=self.freq))

    #low pass filter - first order
    def firstlp_filter(self, w):
        b= [0,w]
        a= [1,w]
        return RealtimeFilter(*signal.bilinear(b,a,fs=self.freq))


    #high pass filter - second order
    def secondhp_filter(self, z,w):
        b=[1,0,0]
        a=[1,2*z*w,w**2]
        return RealtimeFilter(*signal.bilinear(b,a,fs=self.freq))

    def firsthp_filter(self, w):
        b=[1,0]
        a=[1,w]
        return RealtimeFilter(*signal.bilinear(b,a,fs=self.freq))

    #single integrator - 1/s
    def sint_filter(self):
        b= [0,1]
        a= [1,0]
        return RealtimeFilter(*signal.bilinear(b,a,fs=self.freq))

    #double integrator - 1/s**2
    def dint_filter(self):
        b= [0,0,1]
        a= [1,0,0]
        return RealtimeFilter(*signal.bilinear(b,a,fs=self.freq))

    def tilt_scaling(self, scalar):

        return scalar * (self.max_rotational_acceleration / self.max_translational_acceleration)

    def apply_scaling(self, x, max_x, max_y):
        if max_x == 0:
            return 0
        sign = np.sign(x)
        if abs(x) > max_x:
            return sign * max_y
        else:
            return (max_y / max_x) * x

    def apply_movement_scaling(self, scalar):

        return self.apply_scaling(scalar, 3, self.max_translational_acceleration)

    def apply_rotate_scaling(self, scalar):

        return self.apply_scaling(scalar,2, self.max_rotational_velocity)

    def dataInput(self, vector):
        self.xIn = vector[0]*self.tconversion
        self.yIn = vector[1]*self.tconversion
        self.zIn = vector[2]*self.yconversion

        #print(self.xIn)
        #print(self.yIn)
        #print(self.zIn)
        self.pitchIn = vector[3]*self.aconversion
        self.rollIn = vector[4]*self.aconversion
        self.yawIn = vector[5]*self.aconversion
        #print(vector[3], end=", ")
        #print(vector[4], end=", ")
        #print(vector[5])
        #print(self.pitchIn, end=", ")
        #print(self.rollIn, end=", ")
        #print(self.yawIn)

        self.xOut = self.tGain*self.apply_movement_scaling(self.surge_dint.apply(self.surge_hp2.apply(self.surge_hp1.apply(self.xIn))))
        self.pitchOut = -self.rGain*self.apply_rotate_scaling(self.pitch_sint.apply(self.pitch_hp2.apply(self.pitchIn)) + self.tilt_scaling(self.sp_tilt_lp.apply(self.xIn)/constants.g))

        #TODO: check if y-axis is backwards
        self.yOut = self.tGain*self.apply_movement_scaling(self.sway_dint.apply(self.sway_hp2.apply(self.sway_hp1.apply(self.yIn))))
        self.rollOut = -self.rGain*self.apply_rotate_scaling(self.roll_sint.apply(self.roll_hp2.apply(self.rollIn)) + self.tilt_scaling(self.sr_tilt_lp.apply(self.yIn)/constants.g))

        self.zOut = self.tGain*self.apply_movement_scaling(self.heave_dint.apply(self.heave_hp2.apply(self.heave_hp1.apply(self.zIn))))
        self.yawOut = self.yGain*self.apply_rotate_scaling(self.yaw_sint.apply(self.yaw_hp2.apply(self.yawIn)))

        self.translational_vector[0] = self.xOut
        self.translational_vector[1] = self.yOut
        self.translational_vector[2] = self.zOut +0.22
        #self.translational_vector =[self.xOut,self.yOut,self.zOut+0.3]
        self.rotational_vector[0] = self.pitchOut
        self.rotational_vector[1] = self.rollOut
        self.rotational_vector[2] = self.yawOut
        #self.rotational_vector = [self.pitchOut,self.rollOut,self.yawOut]

        #print(self.translational_vector,end=" - ")
        #print(self.rotational_vector)


