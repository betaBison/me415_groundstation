
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
p1.setLabel('left',"Altitude",units='m')
p1.setXRange(-5.0,5.0)
p1.setYRange(20,40)
p1.hideButtons()
p1.plot([-10,10],[30,30],pen=pg.mkPen('g',width=3,style=QtCore.Qt.DotLine))
p1.plot([-10,10],[35,35],pen=pg.mkPen('r',width=3,style=QtCore.Qt.DotLine))
p1.plot([-10,10],[25,25],pen=pg.mkPen('r',width=3,style=QtCore.Qt.DotLine))
altitude_marker = pg.ScatterPlotItem(x=[0.0], y=[0.0],
                               pen='b', brush='b', size=50)
p1.addItem(altitude_marker)
# Ground Track Plot
p2 = win.addPlot(row=1,col=3,rowspan=2,title="Ground Track Plot")
g2x = [-111.63587334,-111.63552403]
g2y = [40.26764106, 40.26678857]
g2 = p2.plot(x=g2x,y=g2y,pen =None,symbol='o', symbolPen=None, symbolSize=10, symbolBrush=(255,0,0,255))

# Elevation Profile
p3 = win.addPlot(row=3,col=3,rowspan=2,title="Elevation Profile")
g3 = p3.plot(pen=(0,255,0))

# 3D Trajectory
p4 = win.addPlot(row=5,col=3,rowspan=2,title="3D Trajectory")



# Warnings
l1 = win.addLabel("Altitude:",row=1,col=2)
l5 = win.addLabel("0.0",row=2,col=2)
l2 = win.addLabel("None",row=3,col=2)
l3 = win.addLabel("Boundary Warning:",row=4,col=2)
l4 = win.addLabel("None",row=5,col=2)

# update all plots
first_time = True
time_p = 0.0
def updateGraphs():
    global p1,p2,p3,p4, first_time
    global time_p, lat_p, lon_p, alt_p
    global altitude_marker
    if gps.initialized == True and gps.time != gps.time0:
        if first_time == True:
            p1.removeItem(altitude_marker)
            altitude_marker = pg.ScatterPlotItem(x=[0.0], y=[gps.alt0],
                                           pen='b', brush='b', size=50)
            p1.addItem(altitude_marker)
            p2.plot(x=[gps.lon0,gps.lon],y=[gps.lat0,gps.lat],pen=(0,0,255))
            p3.plot(x=[gps.time0,gps.time],y=[gps.alt0,gps.alt],pen=(0,255,0))
            lon_p = gps.lon
            lat_p = gps.lat
            alt_p = gps.alt
            first_time = False
        else:
            p1.removeItem(altitude_marker)
            altitude_marker = pg.ScatterPlotItem(x=[0.0], y=[gps.alt],
                                           pen='b', brush='b', size=50)
            p1.addItem(altitude_marker)
            p2.plot(x=[lon_p,gps.lon],y=[lat_p,gps.lat],pen=(0,0,255))
            p3.plot(x=[time_p,gps.time],y=[alt_p,gps.alt],pen=(0,255,0))
        l5.setText(gps.alt)
        time_p = gps.time
        lat_p = gps.lat
        lon_p = gps.lon
        alt_p = gps.alt

    #p2.setData(gps.lat_history,gps.lon_history)
    #p3.setData(gps.time_history,gps.alt_history)
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
    updateGraphs()

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
