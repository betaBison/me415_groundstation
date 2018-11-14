# import modules
import serial
import sys
import time
import warnings
from math import pi, isnan
import numpy as np
import libnmea_navsat_driver.driver
from pyqtgraph.Qt import QtCore, QtGui

class Gpslogger():

    def __init__(self, updateGraphs,setBoundWarn):

        self.initializevariables()
        #self.startgpslog()
        self.run = True
        self.updateGraphs = updateGraphs
        #self.updateAltWarn = updateAltWarn
        self.setBoundWarn = setBoundWarn

    def stop(self):
        self.run = False


# -------- Don't change anything above this line -----------

    def initializevariables(self):
        """
        FOR YOU: Define any constants or class variables you want in your function.
        This function is only called once at initialization.
        """

        # a counter
        self.counter = 0

        # define boundary ellipse
        self.bdy_center = [40.2672305, -111.635524]
        self.bdy_R = [98.353826748828595, 127.49653449395105]
        self.bdy_theta = -15*pi/180.0

        # define waypoint
        self.wp_start = [40.267641059243935, -111.63587333726502]
        self.wp_finish = [40.26678857, -111.63552403]
        self.wp_tolerance = 10.0  # m
        self.init_pts = 15 # points that achieves a gps fix (suggest 100)

        self.lat_init = np.zeros((self.init_pts))
        self.lon_init = np.zeros((self.init_pts))
        self.alt_init = np.zeros((self.init_pts))
        self.time0 = 0.0
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.alt0 = 0.0

        self.initialized = False

        self.time = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        self.time_history = []
        self.lat_history = []
        self.lon_history = []
        self.alt_history = []

    def usrfun(self, time, fix, NumSat, lat, lon, alt, speed, ground_course, covariance):
        """
        FOR YOU: Define any logic you need for processing GPS data here.
        This function is called every time a new GPS packet is received.
        """
        if fix == True:
            print(self.counter)
            #self.groundstation.update()
            # Initial GPS initialization
            if self.counter < self.init_pts:
                self.lat_init[self.counter] = lat
                self.lon_init[self.counter] = lon
                self.alt_init[self.counter] = alt

            elif self.counter == self.init_pts:
                self.lat0 =  np.mean(self.lat_init)
                self.lon0 = np.mean(self.lon_init)
                self.alt0 = np.mean(self.alt_init)
                print("GPS Initialization complete")
                print(self.lat0,self.lon0,self.alt0)
                self.lat = lat
                self.lon = lon
                self.alt = alt-self.alt0
                self.time0 = time
                self.time = time-self.time0
                self.initialized = True

            else:
                self.altitude_warning(alt)
                self.lat = lat
                self.lon = lon
                self.alt = alt-self.alt0
                self.time = time-self.time0
                self.time_history.append(time-self.time0)
                self.lat_history.append(lat)
                self.lon_history.append(lon)
                self.alt_history.append(alt)
                print("Current altitude =",alt-self.alt0)
                #self.updateGraphs()



            # ---- print GPS state -----
            if self.counter%10 == 0:  # reprint the titles every 10 iterations
                print("time,fix,NumSat,lat,lon,alt,speed,ground_course,covariance")

            print(time,fix,NumSat,lat,lon,alt,speed,ground_course,covariance)


            # ---- boundary checks ----

            ell = self.ellipse([lat, lon])
            if ell > 1.0:
                print("ALERT!  Out of bounds! Return immediately!")
                self.setBoundWarn = 1
            elif ell > 0.7:
                print("WARNING!  Close to boundary! Ready pilot.")
                self.setBoundWarn = 2

            # --- update counter ---
            self.counter += 1
        else:
            print("No GPS Fix")



 # -------- Don't change anything below this line -----------

    def altitude_warning(self,alt):
        if alt - self.alt0 < 15.0:
            #self.updateAltWarn("Warning. Approaching Ground Climb Immediately")
            print("Warning. Approaching Ground Climb Immediately")
        elif alt - self.alt0 > 91.44:
            #self.updateAltWarn("Warning altitude over 300ft. Please descend")
            print("Warning altitude over 300ft. Please descend")
        elif alt - self.alt0 > 122.0:
            #self.updateAltWarn("Above Altitude Ceiling Descend Immediately!")
            print("Above Altitude Ceiling Descend Immediately!")

    def distance(self, pt1, pt2):
        """pt = [lat, long]"""
        EARTH_RADIUS = 6371000.0
        distN = EARTH_RADIUS*(pt2[0] - pt1[0])*pi/180.0
        distE = EARTH_RADIUS*np.cos(pt1[0]*pi/180.0)*(pt2[1] - pt1[1])*pi/180.0
        return distE, distN, np.linalg.norm([distN, distE])


    def ellipse(self, pt):
        """check if point is outside of boundary ellipse (ell > 1)"""

        center = self.bdy_center
        R = self.bdy_R
        theta = self.bdy_theta

        dx, dy, _ = self.distance(center, pt)
        ct = np.cos(theta)
        st = np.sin(theta)
        ell = ((dx*ct + dy*st)/R[0])**2 + ((dx*st - dy*ct)/R[1])**2

        return ell


    def startgpslog(self,port):

        # parse arguments
        if len(port) < 2:
            print('Please specify the port (e.g. python gpslog.py /dev/ttyUSB0)')
            sys.exit(0)
        else:
            serial_port = port

        # set up port reading
        GPS = serial.Serial(port=serial_port, baudrate=57600, timeout=2)
        driver = libnmea_navsat_driver.driver.NMEADriver()

        # set filename and open csv file
        timestr = time.strftime("%Y%m%d-%H%M%S.csv")
        csvfile = open(timestr, "w")
        csvfile.write("time,fix,NumSat,latitude,longitude,altitude,speed,ground_course,covariance\n")

        # set time to zero
        t0 = time.time()

        try:
            while self.run:
                data = GPS.readline().strip()
                try:
                    out = driver.add_sentence(data.decode(), time.time()-t0)
                    if out != False:
                        self.usrfun(*out)
                        if not (isnan(out[3]) or isnan(out[4]) or isnan(out[5])):
                            csvfile.write(str(out[0])+","+str(out[1])+","+str(out[2])+","+ \
                                str(out[3])+","+str(out[4])+","+str(out[5])+","+ \
                                str(out[6])+","+str(out[7])+","+str(out[8])+"\n")
                except ValueError as e:
                    warnings.warn("Value error, likely due to missing fields in the NMEA message. Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag csvfile with the NMEA sentences that caused it." % e)
        except KeyboardInterrupt:
            GPS.close() #Close GPS serial port
            csvfile.close() # Close CSV file


if __name__ == '__main__':
    gpslogger()
