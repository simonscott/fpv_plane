import wx
import os
from WarningLight import *
import time

class MainWindow(wx.Frame):
    def __init__(self, parent, title):
        self.dirname=''

        # A "-1" in the size parameter instructs wxWidgets to use the default size.
        # In this case, we select 200px width and the default height.
        wx.Frame.__init__(self, parent, title=title, size=(200,-1))
        self.control = wx.TextCtrl(self, style=wx.TE_MULTILINE)

        self.warn = WarningLight(self, "CONTROL RSSI LOW", 150)

        # Use some sizers to see layout options
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.control, 1, wx.EXPAND)
        self.sizer.Add(self.warn, 1, wx.EXPAND)

        #Layout sizers
        self.SetSizer(self.sizer)
        self.SetAutoLayout(1)
        self.sizer.Fit(self)
        self.Show()


app = wx.App(False)
frame = MainWindow(None, "Sample editor")
frame.warn.On()
app.MainLoop()
