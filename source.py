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

class MyWidget(QtWidgets.QWidget):

    def __init__(self, parent = None):
        super(MyWidget, self).__init__(parent)
        self.layout = QtWidgets.QGridLayout()
        self.sl = []
        self.tlabel = []
        self.initValue = 127
        self.udpPort = 9000
        self.udpSocket = QtNetwork.QUdpSocket()


        names = ['X','Y','Z','Pitch','Roll','Yaw']
        for i in range(0,6):

            name = QtWidgets.QLabel(names[i])
            name.setAlignment(Qt.AlignRight)
            self.layout.addWidget(name,i,0)

            self.sl.append(QtWidgets.QSlider(Qt.Horizontal))

            self.sl[i].setMinimum(0)
            self.sl[i].setMaximum(255)
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
        self.setLayout(self.layout)

    def valuechange(self):
        for i in range(0,6):
            size = self.sl[i].value()
            self.tlabel[i].setNum(size)
        sendstring = ""
        for i in range(0,9):
            if i<3:
                sendstring += str(self.sl[i].value())
            elif i<6:
                sendstring += "127"
            else:
                sendstring += str(self.sl[i-3].value())
            if i<8:
                sendstring += ","
        sendstring += "\n"
        print(sendstring)
        sendstring = sendstring.encode('utf-8')
        datagram = QtCore.QByteArray()
        datagram.resize(len(sendstring))
        datagram = sendstring
        self.udpSocket.writeDatagram(datagram,QtNetwork.QHostAddress.LocalHost,self.udpPort)

    def buttonReset(self):

        for i in range(0,6):
            self.tlabel[i].setNum(self.initValue)
            self.sl[i].setValue(self.initValue)


if __name__ == "__main__":
    app = QtWidgets.QApplication([])



    widget = MyWidget()
    widget.resize(800, 200)
    widget.show()

    t = QtCore.QTimer()
    t.timeout.connect(widget.valuechange)
    t.start(20)

    sys.exit(app.exec_())
