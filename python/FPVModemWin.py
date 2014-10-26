import wx
import os
from Compass import *

"""
Constant Definitions
"""
STATE_NORMAL = 1
STATE_NO_SIG = 2
STATE_NO_LOCK = 3


class FPVModemWin(wx.Frame):

    """
    Constructor
    """
    def __init__(self, parent, title):

        wx.Frame.__init__(self, parent, title=title, size=(-1,-1))

        # Pointers to functions in the decoder that run when events occur
        self.set_home = None
        self.close_log = None

        # Create the compass
        self.compass = Compass(self, 60)

        # Create the labels
        self.status_value = wx.StaticText(self, label="XX XXX XXXX")
        self.rssi_label = wx.StaticText(self, label="RSSI")
        self.rssi_value = wx.StaticText(self, label="000%")
        self.alt_label = wx.StaticText(self, label="Altitude")
        self.alt_value = wx.StaticText(self, label="000m")
        self.dist_label = wx.StaticText(self, label="Distance")
        self.dist_value = wx.StaticText(self, label="0000m")
        self.speed_label = wx.StaticText(self, label="Speed")
        self.speed_value = wx.StaticText(self, label="000 km/h")
        self.lost_pkt_label = wx.StaticText(self, label="Packets lost: 000")

        # Create the button to set the Home position
        self.start_stop_flight_but = wx.Button(self, label='Set Home')
        self.start_stop_flight_but.Bind(wx.EVT_BUTTON, self.button_click_handler)

        # Set the font sizes
        value_font = wx.Font(20, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        label_font = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        small_font = wx.Font(14, wx.DEFAULT, wx.NORMAL, wx.NORMAL)
        self.status_value.SetFont(value_font)
        self.rssi_label.SetFont(label_font)
        self.rssi_value.SetFont(value_font)
        self.alt_label.SetFont(label_font)
        self.alt_value.SetFont(value_font)
        self.dist_label.SetFont(label_font)
        self.dist_value.SetFont(value_font)
        self.speed_label.SetFont(label_font)
        self.speed_value.SetFont(value_font)
        self.lost_pkt_label.SetFont(small_font)
        self.start_stop_flight_but.SetFont(small_font)

        # Use some sizers to see layout options
        self.sizer = wx.GridSizer(6, 2, vgap=10, hgap=20)
        self.sizer.Add(self.compass, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.status_value, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.rssi_label, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.rssi_value, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.alt_label, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.alt_value, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.dist_label, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.dist_value, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.speed_label, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.speed_value, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.start_stop_flight_but, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)
        self.sizer.Add(self.lost_pkt_label, 1, wx.ALIGN_CENTER_HORIZONTAL | wx.ALIGN_CENTER_VERTICAL)

        # Add event listeners
        self.Bind(wx.EVT_CLOSE, self.closeEvent)

        #Layout sizers
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)
        self.Show()


    """
    Updates the display with new data values
    Parameters:
        state - 
        rssi - int from 0 to 100
        alt - integer
        dist - float
        speed - integer
        pkts_lost - integer
        home_set - boolean
    """
    def updateDisplay(self, rssi, alt, dist, speed, dir_home, pkts_lost, home_set):

        self.rssi_value.SetLabel(str(rssi) + "%")
        self.alt_value.SetLabel(str(alt) + "m")
        self.dist_value.SetLabel(str(dist) + "m")
        self.speed_value.SetLabel(str(speed) + " km/h")
        self.compass.SetDirection(dir_home)
        self.lost_pkt_label.SetLabel("Packets lost: " + str(pkts_lost))

        if(home_set):
            self.start_stop_flight_but.SetLabel("Set Home")
        else:
            self.start_stop_flight_but.SetLabel("Waiting...")
    
    """
    Sets the modem status
    """
    def updateStatus(self, status):

        if status == STATE_NORMAL:
            self.status_value.SetLabel("")
        elif status == STATE_NO_SIG:
            self.status_value.SetLabel("NO SIGNAL")
        elif status == STATE_NO_LOCK:
            self.status_value.SetLabel("NO GPS LOCK")


    """
    Sets the function to call when user clicks "Set Home"
    """
    def set_close_log_func(self, close_log):

        self.close_log = close_log


    """
    Sets the function to call when user clicks "Set Home"
    """
    def set_home_func(self, set_home):

        self.set_home = set_home


    """
    Closes the log file if the user presses the X
    """
    def closeEvent(self, event):
        self.close_log()
        self.Destroy()


    """
    Runs when the button is pressed
    """
    def button_click_handler(self, event):
        self.set_home()



