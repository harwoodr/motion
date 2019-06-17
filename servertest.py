import sys
import random
from PyQt5.QtCore import Qt
from PyQt5 import QtNetwork
from PyQt5 import QtWidgets
from PyQt5 import QtCore
from PyQt5 import QtGui
import numpy as np
import math
import time
import scipy
from scipy import signal
from scipy import constants
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import serial
import csv

from kinematics import inverseKinematics
from mca import motionCueing



class MyWidget(QtWidgets.QWidget):
    def __init__(self, parent = None):
        super(MyWidget, self).__init__(parent)

        self.serialOp = False
        if self.serialOp:
            #ser = serial.serial_for_url('loop://', timeout=1)
            self.ser = serial.Serial('/dev/ttyACM0',115200, timeout=0.05)
            self.ser.write(b'0C\n')

        self.sl = []
        self.tlabel = []
        self.muscle = []
        self.mlabel = []
        self.plabel = []

        self.udpPort = 9000
        self.udpSocket = QtNetwork.QUdpSocket()
        self.udpSocket.bind(QtNetwork.QHostAddress("0.0.0.0"),self.udpPort)
        self.udpSocket.readyRead.connect(self.readPendingDatagrams)

        names = ['X','Y','Z','Pitch','Roll','Yaw']

        self.layout = QtWidgets.QGridLayout()

        self.initValue = 0
        for i in range(0,6):

            name = QtWidgets.QLabel(names[i])
            name.setAlignment(Qt.AlignRight)
            self.layout.addWidget(name,i,0)

            self.sl.append(QtWidgets.QSlider(Qt.Horizontal))
            if i==2:
                self.sl[i].setMinimum(0)
                self.sl[i].setMaximum(60)
                self.sl[i].setValue(30)
            else:
                self.sl[i].setMinimum(-45)
                self.sl[i].setMaximum(+45)
                self.sl[i].setValue(self.initValue)
            self.sl[i].setTickPosition(QtWidgets.QSlider.TicksAbove)
            self.sl[i].setTickInterval(5)
            self.sl[i].valueChanged.connect(self.valuechange)
            self.layout.addWidget(self.sl[i],i,1)

            self.tlabel.append(QtWidgets.QLabel())
            self.tlabel[i].setNum(self.initValue)
            self.tlabel[i].setAlignment(Qt.AlignRight)
            self.layout.addWidget(self.tlabel[i],i,2)

        self.b1 = QtWidgets.QPushButton("Reset")
        #self.b1.setCheckable(True)
        #self.b1.toggle()
        self.b1.clicked.connect(self.buttonReset)
        self.layout.addWidget(self.b1,6,0)
        self.b2 = QtWidgets.QPushButton("Enable Network")
        self.b2.setCheckable(True)
        self.b2.clicked.connect(self.disableSliders)
        self.layout.addWidget(self.b2,6,2)

        for i in range(0,6):
            name = QtWidgets.QLabel()
            name.setNum(i)
            name.setAlignment(Qt.AlignRight)
            self.layout.addWidget(name,i+7,0)
            self.muscle.append(QtWidgets.QSlider(Qt.Horizontal))
            self.muscle[i].setEnabled(False)
            self.muscle[i].setValue(0)
            self.muscle[i].setMinimum(0)
            self.muscle[i].setMaximum(100)
            self.muscle[i].setTickPosition(QtWidgets.QSlider.TicksAbove)
            self.muscle[i].setTickInterval(5)
            self.layout.addWidget(self.muscle[i],i+7,1)
            self.mlabel.append(QtWidgets.QLabel())
            self.mlabel[i].setNum(self.initValue)
            self.mlabel[i].setAlignment(Qt.AlignRight)
            self.layout.addWidget(self.mlabel[i],i+7,2)
            self.plabel.append(QtWidgets.QLabel())
            self.plabel[i].setNum(self.initValue)
            self.plabel[i].setAlignment(Qt.AlignRight)
            self.layout.addWidget(self.plabel[i],i+7,3)
        self.setLayout(self.layout)

    def readPendingDatagrams(self):
        global mca
        global ik
        while self.udpSocket.hasPendingDatagrams():
            datagram = QtCore.QByteArray()
            datagram.resize(self.udpSocket.pendingDatagramSize())
            s = self.udpSocket.readDatagram(datagram.size())

        if (self.b2.isChecked()):

            #parse network input and changle slider values
            input = s[0].decode("utf-8").strip("\n").strip("\r").split(sep=',')
            vector = np.array([int(input[0]),int(input[1]),int(input[2]),int(input[6]),int(input[7]),int(input[8])])
            vector -= np.array([127,127,153,127,127,127])
            #print(vector)
            mca.dataInput(vector)
            for i in range(0,6):
                if i<3:
                    self.sl[i].setValue(mca.translational_vector[i]*100)
                    #print(mca.translational_vector[i])
                else:
                    self.sl[i].setValue(math.degrees(mca.rotational_vector[i-3]))
                    #print(mca.rotational_vector[i-3])

            #print("---")
            self.valuechange()


    def valuechange(self):
        global m1
        global ik
        #translation_vector = [0,0,0]
        #rotational_vector = [0,0,0]

        for i in range(0,6):
            size = self.sl[i].value()
            self.tlabel[i].setNum(size)
            if i<3:
                ik.translation_vector[i] = size/100
            else:
                ik.rotational_vector[i-3] = math.radians(size)
            #translation_vector[2] += 0.3
        #ik.reset_platform()
        ik.update_platform()
        for i in range(0,6):
            self.muscle[i].setValue((ik.pam_lengths[i]-1)*50)
            self.mlabel[i].setNum(ik.pam_lengths[i])
            self.plabel[i].setNum(ik.pam_pressures[i])
        verts = np.array([
            ik.platform_anchors[0]+[0,0,0],
            ik.platform_anchors[2]+[0,0,0],
            ik.platform_anchors[4]+[0,0,0]
        ])
        #print(ik.platform_anchors)
        m1.setMeshData(vertexes=verts, faces=faces, faceColors=colors, smooth=False)
        m1.meshDataChanged()

        sendstring = "0M"
        for i in range(0,6):
            sendstring += str(ik.pam_pressures[i]).rjust(3,'0')
        sendstring += "\n"
        if self.serialOp:
            self.ser.write(sendstring.encode('utf-8'))
        #else:
            #print(sendstring)
        if np.average(ik.pam_pressures)<35:
            print("LONG!")
        elif np.average(ik.pam_pressures)>145:
            print("SHORT!")
        if (np.average(ik.pam_pressures)<40 or np.average(ik.pam_pressures)>140):
            for i in range(0,12):
                if i<3:
                    print(mca.translational_vector[i],end="")
                elif i<6:
                    print(math.degrees(mca.rotational_vector[i-3]),end="")
                else:
                    print(ik.pam_pressures[i-6],end="")
                if i<11:
                    print(", ",end="")
                else:
                    print("")

    def buttonReset(self):

        for i in range(0,6):
            self.tlabel[i].setNum(self.initValue)
            self.sl[i].setValue(self.initValue)

    def disableSliders(self):
        for i in range(0,6):
            self.sl[i].setEnabled(not self.b2.isChecked())



if __name__ == "__main__":




    ik = inverseKinematics()
    mca = motionCueing()
    app = QtWidgets.QApplication([])



    widget = MyWidget()
    widget.resize(800, 200)
    widget.show()
    w = gl.GLViewWidget()
    w.opts['azimuth'] = 160
    w.opts['elevation'] = 0
    w.opts['distance'] = 10
    w.show()
    w.setWindowTitle('pyqtgraph example: GLLinePlotItem')
    gx = gl.GLAxisItem()
    gx.setSize(20,20,20)
    gx.translate(0,0,-1.83)
    w.addItem(gx)

    verts = np.array([
        ik.initial_platform_anchors[0],
        ik.initial_platform_anchors[2],
        ik.initial_platform_anchors[4]
    ])
    #print(ik.initial_platform_anchors)
    faces = np.array([
        [0, 1, 2]
    ])
    colors = np.array([
        [1, 0, 0, 1]
    ])

    colorsb = np.array([
        [0, 0, 1, 1]
    ])

    m1 = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False)

    m1.setGLOptions('additive')
    w.addItem(m1)
    widget.valuechange()

    def letsGo():
        app.exec_()
        if widget.serialOp:
            widget.ser.write(b'0O\n')
        return 0

    sys.exit(letsGo())

