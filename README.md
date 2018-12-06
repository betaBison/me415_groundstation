


# Run the groundv2.py file
The script groundv2.py starts a thread running gpslog2.py that reads serial GPS data (NMEA messages) and writes the data to a time-stamped csv file. Data is output only if the gps has a satellite fix. You can stop the file by closing the graph display.


# Example Usage (from command line)
```
python3 groundv2.py
```
# Variables to change
Replace the 'port' variable at the top of the groundv2.py file if necessary. Default is 'dev/ttyUSB0'. See Appendix A to find your port (pulled from https://github.com/byuflowlab/415gps)
Change 'self.init_pts' with how many points you want to average for your initial home position


# Troubleshooting
File runs, but nothing prints to the command line: the transmitter and receiver are not connected
Prints ("NO GPS FIX"): a gps fix has not yet been acquired


# Appendix A: Knowing your part
You will need to replace '/dev/ttyUSB0' with the path to your port.

## Finding Your Serial Port
### Windows
* Open Device Manager, and expand the Ports (COM & LPT) list.
* Note the number on the USB Serial Port.
* Run script with 
  ```
  python gpslog.py COM*
  ```
  where * is the number of your serial port

### Macintosh
* Open a terminal and type: ls /dev/*.
* Note the port number listed for /dev/tty.usbmodem* or /dev/tty.usbserial*.
* Run the script with
  ```
  python gpslog.py /dev/tty.usbmodem*
  ```
  or 
  ```
  python gpslog.py /dev/tty.usbserial*
  ```
  where * is the number of your serial port

### Linux
* Open a terminal and type: ls /dev/tty*.
* Note the port number listed for /dev/ttyUSB* or /dev/ttyACM*.
* Run the script with
  ```
  python gpslog.py /dev/ttyUSB*
  ```
  where * is the number of your serial port
