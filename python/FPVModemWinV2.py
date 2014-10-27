import wx
import os
from Compass import *
from FuelBar import *
from WarningLight import *
from FPVModemDefns import *


class FPVModemWinV2(wx.Frame):

    """
    Constructor
    """
    def __init__(self, parent, title, bat_config):

        # Initialize the window and sizer
        wx.Frame.__init__(self, parent, title=title, size=(400,770))
        self.num_rows = 23
        self.num_cols = 3
        self.bat_config = bat_config
        sizer = wx.GridBagSizer(vgap = 4, hgap = 4)
    
        # Pointers to functions in the decoder that run when events occur
        self.set_home = None
        self.close_log = None

        # Create the GUI elements
        self.createGUI(sizer)

        # Add event listeners
        self.Bind(wx.EVT_CLOSE, self.closeEvent)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)
        self.set_home_but.Bind(wx.EVT_BUTTON, self.home_but_handler)

        # Set the sizer and make things fit
        for i in range(self.num_rows - 1):
            sizer.AddGrowableRow(i);
        for i in range(self.num_cols):
            sizer.AddGrowableCol(i);

        self.SetSizer(sizer)
        #sizer.Fit(self)
        self.Show()


    """
    The paint method. Used to draw the borders between components.
    """
    def OnPaint(self, event):

        # Erase the background
        dc = wx.BufferedPaintDC(self)
        backBrush = wx.Brush('#F0F0F0', wx.SOLID)
        dc.SetBackground(backBrush)
        dc.Clear()

        # Get the y values of the lines to draw
        window_width = self.GetClientSizeTuple()[0]
        top_compass = self.GetSizer().GetChildren()[4].GetPosition().y - 2
        top_rssi_label = self.GetSizer().GetChildren()[10].GetPosition().y - 10
        top_bat_label = self.GetSizer().GetChildren()[16].GetPosition().y - 4

        # Draw the lines
        dc.SetPen(wx.Pen(wx.BLACK, 1))
        dc.DrawLine(0, top_compass, window_width, top_compass)
        dc.DrawLine(0, top_rssi_label, window_width, top_rssi_label)
        dc.DrawLine(0, top_bat_label, window_width, top_bat_label)
      

    def OnEraseBackground(self, event):
        """ Handles the wx.EVT_ERASE_BACKGROUND event. """
        pass


    """
    Create the actual GUI elements
    """
    def createGUI(self, sizer):

        # Create the warning lights
        self.warn_light_telem = WarningLight(self, "NO TELEM SIGNAL")
        self.warn_light_rssi = WarningLight(self, "CONTROL RSSI LOW")
        self.warn_light_6v_bat = WarningLight(self, "12V BAT LOW")
        self.warn_light_12v_bat = WarningLight(self, "6V BAT LOW")

        sizer.Add(self.warn_light_telem, pos=(0, 0), span=(1, 3), flag=wx.EXPAND)
        sizer.Add(self.warn_light_rssi, pos=(1, 0), span=(1, 3), flag=wx.EXPAND)
        sizer.Add(self.warn_light_6v_bat, pos=(2, 0), span=(1, 3), flag=wx.EXPAND)
        sizer.Add(self.warn_light_12v_bat, pos=(3, 0), span=(1, 3), flag=wx.EXPAND)

        # Add the navigation info
        self.compass = Compass(self, 0)
        self.speed_label = wx.StaticText(self, label="Speed")
        self.speed_value = wx.StaticText(self, label="000 km/h")
        self.alt_label = wx.StaticText(self, label="Alt")
        self.alt_value = wx.StaticText(self, label="000 m")
        self.dist_value = wx.StaticText(self, label="0000 m")

        sizer.Add(self.compass, pos=(4, 1), span=(5, 1), flag=wx.EXPAND|wx.ALIGN_BOTTOM|wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.speed_label, pos=(4, 0), span=(3, 1), flag=wx.ALIGN_BOTTOM|wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.speed_value, pos=(7, 0), span=(4, 1), flag=wx.ALIGN_TOP|wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.alt_label, pos=(4, 2), span=(3, 1), flag=wx.ALIGN_BOTTOM|wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.alt_value, pos=(7, 2), span=(4, 1), flag=wx.ALIGN_TOP|wx.ALIGN_CENTER_HORIZONTAL)
        sizer.Add(self.dist_value, pos=(9, 1), span=(2, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)

        # Add the RSSI info
        self.telem_rssi_label = wx.StaticText(self, label="Telem\nRSSI")
        self.telem_rssi_value = wx.StaticText(self, label="000")
        self.lost_pkts_label = wx.StaticText(self, label="Lost\nPackets")
        self.lost_pkts_value = wx.StaticText(self, label="000")
        self.control_rssi_label = wx.StaticText(self, label="Control\nRSSI")
        self.control_rssi_bar = FuelBar(self, 0, 100, '%', 'v', 2)

        sizer.Add(self.telem_rssi_label,    pos=(11, 0), span=(2, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.telem_rssi_value,    pos=(13, 0), span=(3, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.lost_pkts_label,     pos=(11, 1), span=(2, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.lost_pkts_value,     pos=(13, 1), span=(3, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.control_rssi_label,  pos=(11, 2), span=(2, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.control_rssi_bar,    pos=(13, 2), span=(3, 1), border=30, flag=wx.LEFT|wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)

        # Add the battery meters
        self.bat_12v_label = wx.StaticText(self, label="12V Batt")
        self.bat_6v_label = wx.StaticText(self, label="6V Batt")
        self.volt_12v_bar = FuelBar(self, self.bat_config.bat12_min_volt, self.bat_config.bat12_max_volt, 'V', 'h', 2)
        self.volt_6v_bar = FuelBar(self, self.bat_config.bat6_min_volt, self.bat_config.bat6_max_volt, 'V', 'h', 2)
        self.cap_12v_bar = FuelBar(self, 0, self.bat_config.bat12_cap, 'mAh', 'h', 2)
        self.cap_6v_bar = FuelBar(self, 0, self.bat_config.bat6_cap, 'mAh', 'h', 2)
        self.curr_12v_value = wx.StaticText(self, label="00.0 A")
        self.curr_6v_value = wx.StaticText(self, label="000 mA")

        sizer.Add(self.bat_12v_label,   pos=(16, 0), span=(1, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.bat_6v_label,    pos=(16, 2), span=(1, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.volt_12v_bar,    pos=(17, 0), span=(2, 1), flag=wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.volt_6v_bar,     pos=(17, 2), span=(2, 1), flag=wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.cap_12v_bar,     pos=(19, 0), span=(2, 1), flag=wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.cap_6v_bar,      pos=(19, 2), span=(2, 1), flag=wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.curr_12v_value,  pos=(21, 0), span=(1, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.curr_6v_value,   pos=(21, 2), span=(1, 1), flag=wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        
        # Add the buttons
        self.set_home_but = wx.Button(self, label='Set Home')
        self.new_batt_but = wx.Button(self, label='New Battery')

        sizer.Add(self.set_home_but,    pos=(22, 0), span=(1, 1), flag=wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)
        sizer.Add(self.new_batt_but,    pos=(22, 2), span=(1, 1), flag=wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL|wx.ALIGN_CENTER_VERTICAL)

        # Set font sizes
        self.setLargeFonts([self.speed_value, self.alt_value, self.dist_value, self.telem_rssi_value, self.lost_pkts_value, \
                        self.curr_12v_value, self.curr_6v_value]);
        self.setMedFonts([self.speed_label, self.alt_label, self.telem_rssi_label, self.lost_pkts_label, self.control_rssi_label, \
                        self.bat_12v_label, self.bat_6v_label]);

    """
    Sets the font for a list of controls
    """
    def setLargeFonts(self, items):

        large_font = wx.Font(18, wx.DEFAULT, wx.NORMAL, wx.BOLD)

        for i in items:
            i.SetFont(large_font)

    """
    Sets the font for a list of controls
    """
    def setMedFonts(self, items):

        med_font = wx.Font(16, wx.DEFAULT, wx.NORMAL, wx.NORMAL)

        for i in items:
            i.SetFont(med_font)


    """
    Updates the display with new data values
    Parameters:
        alt             - integer, in meters
        dist            - integer, in meters
        speed           - integer, in km/h
        dir_home        - float, in degrees
        control_rssi    - int from 0 to 100
        pkts_lost       - integer
        telem_rssi      - integer from 0 to 100
        bat12_volt      - float, in volts
        bat12_cap       - int, in mAh
        bat12_curr      - float, in amps
        bat6_volt       - float, in volts
        bat6_cap        - int, in mAh
        bat6_curr       - int, in mA
        home_set        - boolean
    """
    def updateDisplay(self, control_rssi, alt, dist, speed, dir_home, pkts_lost,
                        telem_rssi, bat12_volt, bat12_cap, bat12_curr, bat6_volt,
                        bat6_cap, bat6_curr, home_set):

        # Update labels and bar graphs
        self.alt_value.SetLabel(str(alt) + " m")
        self.dist_value.SetLabel(str(dist) + " m")
        self.speed_value.SetLabel(str(speed) + " km/h")
        self.compass.SetDirection(dir_home)
        self.control_rssi_bar.SetLevel(control_rssi)
        self.lost_pkts_value.SetLabel(str(pkts_lost))
        self.telem_rssi_value.SetLabel(str(rssi))
        self.volt_12v_bar.SetLevel(bat12_volt)
        self.volt_6v_bar.SetLevel(bat6_volt)
        self.cap_12v_bar.SetLevel(bat12_cap)
        self.cap_6v_bar.SetLevel(bat12_cap)
        self.curr_12v_value.SetLabel('%.1f A' % str(bat12_curr))
        self.curr_6v_value.SetLabel(str(bat12_curr) + ' mA')

        # Update the button label
        if(home_set):
            self.set_home_but.SetLabel("Set Home")
        else:
            self.set_home_but.SetLabel("Waiting...")

        # Determine if any warning lights need to be lit
        # TODO: if bat_volt at min or cap < 20%, warn

        self.Refresh()
    
    """
    Sets the modem status
    """
    def updateStatus(self, status):

        if status == STATE_NORMAL:
            self.warn_light_telem.Off()
        elif status == STATE_NO_SIG:
            self.warn_light_telem.On()
        elif status == STATE_NO_LOCK:
            self.dist_value.SetLabel("NO GPS LOCK")

        self.Refresh()


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
    def home_but_handler(self, event):
        self.set_home()



