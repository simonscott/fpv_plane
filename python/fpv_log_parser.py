#!/usr/bin/python

import wx
import os
import time
import sys
import re
import matplotlib.pyplot as plotter


# Open log file
f = open(sys.argv[1], 'r')

# Lists for storing data
pkt_cntr = []
bat_12v_v = []
bat_12v_i = []
bat_6v_v = []
bat_6v_i = []
rssi = []
lat = []
lon = []
alt = []
spd = []
bearing = []

# Iterate through records
line = f.readline()

while(len(line)>1):

    # Parse packet counter
    pkt_cntr.append( int(line[16:]) );

    # Parse 12v battery info
    line = f.readline()
    s = re.search(': (\d+\.\d+)V', line).group(1)
    bat_12v_v.append( float(s) )
    s = re.search('(\d+\.\d+)A', line).group(1)
    bat_12v_i.append( float(s) )

    # Parse 6v battery info
    line = f.readline()
    s = re.search(': (\d+\.\d+)V', line).group(1)
    bat_6v_v.append( float(s) )
    s = re.search('(\d+)mA', line).group(1)
    bat_6v_i.append( float(s) )

    # Parse rssi
    line = f.readline()

    # Parse lat 38.29675deg
    line = f.readline()
    s = re.search('([+-]?\d+\.\d+)deg', line).group(1)
    lat.append( float(s) )

    # Parse lon -122.4626deg
    line = f.readline()
    s = re.search('([+-]?\d+\.\d+)deg', line).group(1)
    lon.append( float(s) )

    # Parse alt
    line = f.readline()
    s = re.search('(\d+)m', line).group(1)
    alt.append( int(s) )

    # Parse speed:
    line = f.readline()
    s = re.search('(\d+)km', line).group(1)
    spd.append( int(s) )

    # Parse bearing
    line = f.readline()

    # For next packet
    line = f.readline()

# Close file
f.close()

# Graph the output
plotter.plot(bat_12v_v, 'b')
plotter.plot(bat_12v_i, 'r')
plotter.plot(spd, 'g')
plotter.plot(alt, 'k')
plotter.show()

# Write coords to file
fout = open('coords_out.txt', 'w')
step_sz = 10

for coords in zip(lat, lon)[0::step_sz]:
    fout.write(str(coords[0]) + ', ' + str(coords[1]) + '\n')

fout.close()


