import wx
from math import sqrt, pi

class WarningLight(wx.PyControl):
    """
    A warning light, with custom text, that can be turned on and off.
    """

    """
    The default constructor
    """
    def __init__(self, parent, 
                label, min_width=150,
                id=wx.ID_ANY, pos=wx.DefaultPosition,
                size=wx.DefaultSize, style=wx.NO_BORDER | wx.FULL_REPAINT_ON_RESIZE, validator=wx.DefaultValidator,
                name="WarningLight"):

        wx.PyControl.__init__(self, parent, id, pos, size, style, validator, name)

        self.label = label
        self.min_width = min_width
        self.on = False

        self.InitializeBrushes()
        self.InheritAttributes()

        # Bind the events related to our control: first of all, we use a
        # combination of wx.BufferedPaintDC and an empty handler for
        # wx.EVT_ERASE_BACKGROUND (see later) to reduce flicker
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)


    def InitializeBrushes(self):
        """ Initializes the brushes and pens """
        self.red_brush = wx.Brush('#BB0000', wx.SOLID)


    def OnPaint(self, event):
        """ Handles the wx.EVT_PAINT event for Compass. """

        # If you want to reduce flicker, a good starting point is to
        # use wx.BufferedPaintDC.
        dc = wx.BufferedPaintDC(self)

        # Let's do the actual drawing in the Draw() method, passing the newly
        # initialized wx.BufferedPaintDC
        self.Draw(dc)


    def Draw(self, dc):
        """
        Actually performs the drawing operations.
        """

        # Get the actual client size of ourselves
        width, height = self.GetClientSize()

        if not width or not height:
            # Nothing to do, we still don't have dimensions!
            return

        # Set the text and background colours based on whether the lamp is on or off
        backColour = self.GetBackgroundColour()
        back_brush = wx.Brush(backColour, wx.SOLID)
        
        if self.on:
            dc.SetBackground(self.red_brush)
        else:
            dc.SetBackground(back_brush)

        dc.Clear()
        dc.SetTextForeground("#E0E0E0")

        # Draw the label
        dc.SetFont( wx.Font(20, wx.DEFAULT, wx.NORMAL, wx.BOLD) )
        dc.DrawLabel(str(self.label), (0, 0, width, height), alignment=wx.ALIGN_CENTRE|wx.ALIGN_CENTER_VERTICAL)


    def OnEraseBackground(self, event):
        """ Handles the wx.EVT_ERASE_BACKGROUND event for Compass. """
        pass


    def DoGetBestSize(self):
        """
        Overridden base class virtual.  Determines the best size of the control
        """
        width, height = self.GetClientSize()
        new_width = max(self.min_width, width)
        best = wx.Size(new_width, height)

        # Cache the best size so it doesn't need to be calculated again,
        # at least until some properties of the window change
        self.CacheBestSize(best)

        return best


    def On(self):
        """ Sets the warning lamp on """
        self.on = True


    def Off(self):
        """ Sets the warning lamp off """
        self.on = False


