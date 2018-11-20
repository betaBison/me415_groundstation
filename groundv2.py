
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from gpslogv2 import Gpslogger
import threading
import pyqtgraph.opengl as gl

port = '/dev/ttyUSB0'



win = pg.GraphicsWindow()
pg.setConfigOption('foreground', 'k')
win.setBackground('w')
setBoundWarn = 0

win.setWindowTitle('Dewey and the Aeronauts Official Groundstation')

# FPV plot
p1 = win.addPlot(row=1,col=1,rowspan=9,title="FPV plot")
p1.setLabel('left',"Altitude",units='m')
p1.setXRange(-5.0,5.0)
p1.setYRange(20,40)
p1.hideButtons()
p1.hideAxis('bottom')
p1.plot([-10,10],[30,30],pen=pg.mkPen('g',width=3,style=QtCore.Qt.DotLine))
p1.plot([-10,10],[35,35],pen=pg.mkPen('r',width=3,style=QtCore.Qt.DotLine))
p1.plot([-10,10],[25,25],pen=pg.mkPen('r',width=3,style=QtCore.Qt.DotLine))
altitude_marker = pg.ScatterPlotItem(x=[0.0], y=[0.0],
                               pen='b', brush='b', size=50)
p1.addItem(altitude_marker)



# Ground Track Plot
p2 = win.addPlot(row=1,col=3,rowspan=3,title="Ground Track Plot")
g2x = [-111.63587334,-111.63552403]
g2y = [40.26764106, 40.26678857]
g2 = p2.plot(x=g2x,y=g2y,pen =None,symbol='o', symbolPen=None, symbolSize=10, symbolBrush=(255,0,0,255))
p2.setLabel('left',"Latitude")
p2.setLabel('bottom',"Longitude")

# Elevation Profile
p3 = win.addPlot(row=4,col=3,rowspan=3,title="Elevation Profile")
g3 = p3.plot(pen=(0,255,0))
p3.setLabel('left',"Altitude",units='m')
p3.setLabel('bottom',"Distance",units='m')

# 3D Trajectory
p4 = win.addPlot(row=7,col=3,rowspan=3,title="3D Trajectory")
'''
p4 = gl.GLViewWidget()
p4.opts['distance'] = 40
p4.show()
'''

# Warnings
l1 = win.addLabel("Altitude:",row=1,col=2)
l5 = win.addLabel("0.0",row=2,col=2)
l2 = win.addLabel("None",row=3,col=2)
l3 = win.addLabel("Boundary Warning:",row=4,col=2)
l4 = win.addLabel("None",row=5,col=2)
l6 = win.addLabel("Distance At Altitude",row=6,col=2)
l7 = win.addLabel("0.0",row=7,col=2)
l8 = win.addLabel(" ",row=8,col=2)
l9 = win.addLabel(" ",row=9,col=2)


# update all plots
first_time = True
time_p = 0.0
def updateGraphs():
    global p1,p2,p3,p4, first_time
    global time_p, lat_p, lon_p, alt_p
    global time_now, lat_now, lon_now, alt_now
    global altitude_marker
    if gps.initialized == True and gps.time != gps.time0:
        if first_time == True:
            p1.removeItem(altitude_marker)
            altitude_marker = pg.ScatterPlotItem(x=[0.0], y=[gps.alt0],
                                           pen='b', brush='b', size=50)
            p1.addItem(altitude_marker)
            time_now = gps.time
            lat_now = gps.lat
            lon_now = gps.lon
            alt_now = gps.alt
            p2.plot(x=[gps.lon0,lon_now],y=[gps.lat0,lat_now],pen=(0,0,255))
            time_p = time_now
            lon_p = lon_now
            lat_p = lat_now
            alt_p = alt_now
            first_time = False
        else:
            p1.removeItem(altitude_marker)
            altitude_marker = pg.ScatterPlotItem(x=[0.0], y=[gps.alt],
                                           pen='b', brush='b', size=50)
            p1.addItem(altitude_marker)
            time_now = gps.time
            lat_now = gps.lat
            lon_now = gps.lon
            alt_now = gps.alt
            p2.plot(x=[lon_p,lon_now],y=[lat_p,lat_now],pen=(0,0,255))
            p3.plot(x=[time_p,time_now],y=[alt_p,alt_now],pen=(0,255,0))
            time_p = time_now
            lon_p = lon_now
            lat_p = lat_now
            alt_p = alt_now
        l5.setText(alt_now)

    #p2.setData(gps.lat_history,gps.lon_history)
    #p3.setData(gps.time_history,gps.alt_history)
    #p4.setData(x=gps.lat_history,y=gps.lon_history,z=gps.alt_history)
    #update1()
    #update2()
    #update3()

def updateAltWarn(setAltWarn):
    global l2
    if setAltWarn == 0:
        l2.setText("None")
    if setAltWarn == 1:
        l2.setText("Approaching Ground, Climb!")
    if setAltWarn == 2:
        l2.setText("Above 300ft, Descend!")
    if setAltWarn == 3:
        l2.setText("Above 400ft, Descend!")
    else:
        pass

def updateBoundWarn(setBoundWarn):
    global l4
    if setBoundWarn == 0:
        l4.setText("None")
    if setBoundWarn == 1:
        l4.setText("Out of bounds! Return!")
    if setBoundWarn == 2:
        l4.setText("Close to boundary!")
    else:
        pass

def updateWpWarn(WpWarn):
    global l8
    if WpWarn == 0:
        l6.setText("Distance at Alt")
        l7.setText(gps.distance_alt)
    if WpWarn == 1:
        l6.setText("Head to Waypoint 1")
        l7.setText(" ")
    if WpWarn == 2:
        l6.setText("WP 1 reached. Go to 2")
    if WpWarn == 3:
        l6.setText("Wp 2 reached. Go home")

def updateBearing(current,desired):
    global l9
    text = "Current: %3.1f Desired: %3.1f" % (current,desired)
    l9.setText(text)

def updateGui():
    updateBoundWarn(gps.setBoundWarn)
    updateAltWarn(gps.setAltWarn)
    updateWpWarn(gps.waypoint)
    updateBearing(gps.currentBearing,gps.desiredBearing)
    updateGraphs()

timer = pg.QtCore.QTimer()
timer.timeout.connect(updateGui)
timer.start(50)

gps = Gpslogger(updateGraphs)
gpsthread = threading.Thread(target=gps.startgpslog, name="_proc", args=[port])
gpsthread.start()



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        gps.stop()
