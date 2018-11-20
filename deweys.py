# -*- coding: utf-8 -*-
"""
Created on Mon Nov 12 17:30:34 2018

https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points

@author: deweypotts
"""


# ---- logic to be put into main code -----#

            self.newLat = 0.0
            self.newLon = 0.0
            self.oldLat = 0.0
            self.oldLon = 0.0

            newLat=lat
            newLon=lon

            if self.counter > 0:

                currentBearing=currentBearingIndic(oldLat, oldLon, newLat, newLon)

                if waypoint == 1:
                    latWaypoint=latWaypoint1
                    lonWaypoint=lonWaypoint1
                    if distBetweenGPS(newLat, newLon, latWaypoint, lonWaypoint) <10.0:
                        print("You have reached waypoint 1! Head to waypoint 2")
                        waypoint=2

                if waypoint == 2:
                    latWaypoint=latWaypoint2
                    lonWaypoint=lonWaypoint2
                    if distBetweenGPS(newLat, newLon, latWaypoint, lonWaypoint) <10.0:
                        print("You have reached waypoint 2! Head to waypoint 3")
                        waypoint=3

                desiredBearing=desiredBearingIndic(newLat, newLon, latWaypoint, lonWaypoint)

                print("Current heading & desired heading",currentBearing,desiredBearing )


# --- functions ----- #

from math import radians, sin, cos, sqrt, asin, atan2

def distBetweenGPS(lat1, lon1, lat2, lon2):

  R = 6372.8 # Earth radius in kilometers

  dLat = radians(lat2 - lat1)
  dLon = radians(lon2 - lon1)
  lat1 = radians(lat1)
  lat2 = radians(lat2)

  a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
  c = 2*asin(sqrt(a))
  distance= R*c*1000
  return distance



# input the last two GPS coordinates. Returns the heading based on those two coordinated and returns. I believe North is zero. Degrees increase positively clockwise from North.
def currentBearingIndic(lat1, lon1, lat2, lon2):

  #dLat = radians(lat2 - lat1)
  dLon = radians(lon2 - lon1)
  lat1 = radians(lat1)
  lat2 = radians(lat2)
  bearing = atan2(sin(dLon)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dLon))
  bearing = bearing*(180/3.1415)
  currentBearing = (bearing + 360) % 360
  return currentBearing

def desiredBearingIndic(lat, lon, latWaypoint, lonWaypoint): # inout current position (GPS) and the waypoint GPS

  #dLat = radians(lat2waypoint - lat1)
  dLon = radians(lonWaypoint - lon)
  lat = radians(lat)
  latWaypoint = radians(latWaypoint)
  bearing = atan2(sin(dLon)*cos(latWaypoint), cos(lat)*sin(latWaypoint)-sin(lat)*cos(latWaypoint)*cos(dLon))
  bearing = bearing*(180/3.1415)
  desiredBearing = (bearing + 360) % 360 

  return desiredBearing




 #   bearing = atan2(sin(long2-long1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(long2-long1))
 #   bearing = degrees(bearing)
 #   bearing = (bearing + 360) % 360

 # haversine(40.26764106, -111.63587334, 40.26678857, -111.63552403)
