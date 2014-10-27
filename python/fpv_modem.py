#!/usr/bin/python

"""
TO DO:
* Calculate battery capacity used
* Figure out when to enable warnings
* Fix altitude inaccuracy
* Add handler to reset bat cap when button pressed
"""

import wx
import os
import time
import FPVModemWinV2
from FPVModemDecoder import *
import FPVModemDefns 

# Configure our batteries
batConfig = FPVModemDefns.FPVBatteryConfig
batConfig.bat12_max_volt = 12.6;
batConfig.bat12_min_volt = 10.2;
batConfig.bat12_cap = 4000;
batConfig.bat6_max_volt = 7.2;
batConfig.bat6_min_volt = 6.2;
batConfig.bat6_cap = 1600;

# Create a wx application
app = wx.App(False)

# Create the window object
modem_win = FPVModemWinV2.FPVModemWinV2(None, "FPV Modem v1.0", batConfig)

# Create wrappers for the GUI update functions
def updateDisplay(control_rssi, alt, dist, speed, dir_home, pkts_lost,
                    telem_rssi, bat12_volt, bat12_cap, bat12_curr, bat6_volt,
                    bat6_cap, bat6_curr, home_set) :
    wx.CallAfter(modem_win.updateDisplay, control_rssi, alt, dist, speed, dir_home, pkts_lost,
                    telem_rssi, bat12_volt, bat12_cap, bat12_curr, bat6_volt,
                    bat6_cap, bat6_curr, home_set)

def updateStatus(status) : wx.CallAfter(modem_win.updateStatus, status)

# Create the modem decoder
modem_decoder = FPVModemDecoder(updateDisplay, updateStatus)

# Pass the get/set home functions to the GUI
modem_win.set_home_func(modem_decoder.set_home_pos)

# Pass the file close function to the GUI
modem_win.set_close_log_func(modem_decoder.close_log)

# The main GUI event handler loop
app.MainLoop()
