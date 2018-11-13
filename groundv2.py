
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from gpslogv2 import Gpslogger
import threading

win = pg.GraphicsWindow()
pg.setConfigOption('foreground', 'k')
win.setBackground('w')
setBoundWarn = 0

win.setWindowTitle('Dewey and the Aeronauts Official Groundstation')

# FPV plot
p1 = win.addPlot(row=1,col=1,rowspan=6,title="FPV plot")


# Ground Track Plot
p2 = win.addPlot(row=1,col=3,rowspan=2,title="Ground Track Plot")
g2x = [40.26764106, 40.26678857,]
g2y = [-111.63587334,-111.63552403]
g2 = p2.plot(x=g2x,y=g2y,pen =None,symbol='o', symbolPen=None, symbolSize=10, symbolBrush=(255,0,0,255))

# Elevation Profile
p3 = win.addPlot(row=3,col=3,rowspan=2,title="Elevation Profile")
g3 = p3.plot(pen=(0,255,0))

# 3D Trajectory
p4 = win.addPlot(row=5,col=3,rowspan=2,title="3D Trajectory")



# Warnings
l1 = win.addLabel("Altitude Warning:",row=1,col=2)
l2 = win.addLabel("None",row=2,col=2)
l3 = win.addLabel("Boundary Warning:",row=3,col=2)
l4 = win.addLabel("None",row=4,col=2)

# update all plots
def updateGraphs():
    global p1,p2,p3,p4
    #p2.setData(gps.lat_history,gps.lon_history)
    #p3.setData(gps.time_history,gps.alt_history)
    pass
    #p4.setData(x=gps.lat_history,y=gps.lon_history,z=gps.alt_history)
    #update1()
    #update2()
    #update3()

def updateAltWarn(words):
    global l2
    l2.setText(words)

def updateBoundWarn(setBoundWarn):
    global l4
    if setBoundWarn == 1:
        l4.setText("Out of bounds! Return!")
    if setBoundWarn == 2:
        l4.setText("Close to boundary!")
    else:
        pass



def updateGui():
    updateBoundWarn(gps.setBoundWarn)

timer = pg.QtCore.QTimer()
timer.timeout.connect(updateGui)
timer.start(50)

gps = Gpslogger(updateGraphs,setBoundWarn)
gpsthread = threading.Thread(target=gps.startgpslog, name="_proc", args=['/dev/ttyUSB0'])
gpsthread.start()



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        gps.stop()
