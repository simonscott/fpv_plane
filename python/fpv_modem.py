#!/usr/bin/python

import wx
import os
import time
import FPVModemWinV2
from FPVModemDecoder import * 

# Create a wx application
app = wx.App(False)

# Create the window object
modem_win = FPVModemWinV2.FPVModemWinV2(None, "FPV Modem v1.0")

# Create wrappers for the GUI update functions
def updateDisplay(rssi, alt, dist, speed, dir_home, pkts_lost, home_set) : wx.CallAfter(modem_win.updateDisplay, rssi, alt, dist, speed, dir_home, pkts_lost, home_set)
def updateStatus(status) : wx.CallAfter(modem_win.updateStatus, status)

# Create the modem decoder
modem_decoder = FPVModemDecoder(updateDisplay, updateStatus)

# Pass the get/set home functions to the GUI
modem_win.set_home_func(modem_decoder.set_home_pos)

# Pass the file close function to the GUI
modem_win.set_close_log_func(modem_decoder.close_log)

# The main GUI event handler loop
app.MainLoop()
