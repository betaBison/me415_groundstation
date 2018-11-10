from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import numpy as np
import sys
import pyqtgraph as pg
from PyQt5 import QtWidgets

class Groundstation(QtCore.QThread):
    def __init__(self):
        QtCore.QThread.__init__(self)
        pg.setConfigOptions(antialias=True)
        self.app = QtGui.QApplication([])
        self.w = gl.GLViewWidget()
        self.w.setWindowTitle('Dewey and the Aeronauts Official Groundstation')
        self.w.opts['distance'] = 0.0
        self.w.show()
        self.w.setBackgroundColor('k')

    def update(self):
        self.app.processEvents()
        
