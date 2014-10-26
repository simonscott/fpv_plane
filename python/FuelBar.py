import wx
from math import sqrt, pi

class FuelBar(wx.PyControl):
    """
    A fuel bar wx control. It displays how full something is with a bar that changes from red (empty) to green (full).
    """

    """
    The default constructor
    """
    def __init__(self, parent, 
                empty_value, full_value, units, orientation='h', fill_type=2, min_width=0,
                id=wx.ID_ANY, pos=wx.DefaultPosition,
                size=wx.DefaultSize, style=wx.NO_BORDER | wx.FULL_REPAINT_ON_RESIZE, validator=wx.DefaultValidator,
                name="FuelBar"):

        wx.PyControl.__init__(self, parent, id, pos, size, style, validator, name)

        # By default, we start full
        self.empty_value = empty_value
        self.full_value = full_value
        self.level = full_value
        self.units = units
        self.orientation = orientation
        self.fill_type = fill_type

        if min_width == 0:
            if self.orientation == 'v':
                self.min_width = 100
            else:
                self.min_width = 100
        else:
            self.min_width = min_width

        # Set the width:height aspect ratio
        self.aspect_ratio = 3

        self.InitializeBrushes()
        self.InheritAttributes()

        # Bind the events related to our control: first of all, we use a
        # combination of wx.BufferedPaintDC and an empty handler for
        # wx.EVT_ERASE_BACKGROUND (see later) to reduce flicker
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)


    def InitializeBrushes(self):
        """ Initializes the brushes and pens """

        self.black_pen = wx.Pen('black', 1)
        self.trans_pen = wx.Pen('black', 0, wx.TRANSPARENT)

        self.trans_brush = wx.Brush('black', wx.TRANSPARENT)
        self.black_brush = wx.Brush('black', wx.SOLID)

        backColour = self.GetBackgroundColour()
        self.bg_brush = wx.Brush(backColour, wx.SOLID)


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
        text_height = 32
        text_width = 80

        if not width or not height:
            # Nothing to do, we still don't have dimensions!
            return

        # Initialize the wx.BufferedPaintDC, assigning a background
        # colour and a foreground colour (to draw the text)
        backColour = self.GetBackgroundColour()
        backBrush = wx.Brush(backColour, wx.SOLID)
        dc.SetBackground(backBrush)
        dc.Clear()

        # Set the pens and brushes
        dc.SetPen(self.black_pen)
        dc.SetBrush(self.trans_brush)

        if self.orientation == 'h':

            # Compute dimensions and origin
            bar_outline_width = width - 4
            bar_outline_height = height - text_height
            bar_width = bar_outline_width - 2
            bar_height = bar_outline_height - 2
            bar_xo = 3
            bar_yo = 3

            # Draw the bar outline, and fill it
            dc.DrawRectangle(bar_xo - 1, bar_yo - 1, bar_outline_width, bar_outline_height)
            dc.SetPen(self.trans_pen)

            if self.fill_type == 1:
                dc.GradientFillLinear((bar_xo, bar_yo, bar_width/2, bar_height), 'RED', 'YELLOW')
                dc.GradientFillLinear((bar_xo + bar_width/2, bar_yo, bar_width - bar_width/2, bar_height), 'YELLOW', '#00FF00')
            else:
                dc.GradientFillLinear((bar_xo,                   bar_yo, bar_width/3,               bar_height), 'RED', 'ORANGE')
                dc.GradientFillLinear((bar_xo + bar_width/3,     bar_yo, bar_width/3,               bar_height), 'ORANGE', 'YELLOW')
                dc.GradientFillLinear((bar_xo + (bar_width/3)*2, bar_yo, bar_width-(bar_width/3)*2, bar_height), 'YELLOW', '#00FF00')

            # Cover up the part of the bar that is empty
            dc.SetPen(self.trans_pen)
            dc.SetBrush(self.bg_brush)

            fill_frac = (float)(self.level - self.empty_value) / (float)(self.full_value - self.empty_value)
            bar_empty_x = (int)(bar_xo + bar_width * fill_frac)
            bar_empty_width = bar_xo + bar_width - bar_empty_x
            dc.DrawRectangle(bar_empty_x, bar_yo, bar_empty_width, bar_height)

            # Draw the value text below the bar
            dc.SetTextForeground("#303030")
            dc.SetFont( wx.Font(14, wx.DEFAULT, wx.NORMAL, wx.NORMAL) )
            dc.DrawLabel(str(self.level) + ' ' + self.units, (0, height - text_height, width, text_height), alignment=wx.ALIGN_CENTRE|wx.ALIGN_BOTTOM)

        elif self.orientation == 'v':

            # Compute dimensions and origin
            bar_outline_width = width - text_width
            bar_outline_height = height - 4
            bar_width = bar_outline_width - 2
            bar_height = bar_outline_height - 2
            bar_xo = 3
            bar_yo = 3

            # Draw the bar outline, and fill it
            dc.DrawRectangle(bar_xo - 1, bar_yo - 1, bar_outline_width, bar_outline_height)
            dc.SetPen(self.trans_pen)

            if self.fill_type == 1:
                dc.GradientFillLinear((bar_xo, bar_yo, bar_width/2, bar_height), 'RED', 'YELLOW')
                dc.GradientFillLinear((bar_xo + bar_width/2, bar_yo, bar_width - bar_width/2, bar_height), 'YELLOW', '#00FF00')
            else:
                dc.GradientFillLinear((bar_xo, bar_yo,                    bar_width, bar_height/3), 'YELLOW', '#00FF00', nDirection=wx.NORTH)
                dc.GradientFillLinear((bar_xo, bar_yo + bar_height/3,     bar_width, bar_height/3), 'ORANGE', 'YELLOW', nDirection=wx.NORTH)
                dc.GradientFillLinear((bar_xo, bar_yo + (bar_height/3)*2, bar_width, bar_height - (bar_height/3)*2,), 'RED', 'ORANGE', nDirection=wx.NORTH)

            # Cover up the part of the bar that is empty
            dc.SetPen(self.trans_pen)
            dc.SetBrush(self.bg_brush)

            fill_frac = (float)(self.level - self.empty_value) / (float)(self.full_value - self.empty_value)
            bar_empty_y = bar_yo
            bar_empty_height = bar_height * (1 - fill_frac)
            dc.DrawRectangle(bar_xo, bar_empty_y, bar_width, bar_empty_height)

            # Draw the value text alongside the bar
            dc.SetTextForeground("#303030")
            dc.SetFont( wx.Font(14, wx.DEFAULT, wx.NORMAL, wx.NORMAL) )
            dc.DrawLabel(str(self.level) + ' ' + self.units, (width - text_width, 0, text_width, height), alignment=wx.ALIGN_CENTRE|wx.ALIGN_CENTER_VERTICAL)


    def OnEraseBackground(self, event):
        """ Handles the wx.EVT_ERASE_BACKGROUND event for Compass. """
        pass


    def DoGetBestSize(self):
        """
        Overridden base class virtual.  Determines the best size of the control
        """
        width, height = self.GetClientSize()
        new_width = max(self.min_width, width)
        new_height = max(self.min_width/self.aspect_ratio, height)
                
        best = wx.Size(new_width, new_height)

        # Cache the best size so it doesn't need to be calculated again,
        # at least until some properties of the window change
        self.CacheBestSize(best)

        return best


    def SetLevel(self, l):
        """ Sets how full the fuel bar is """

        self.level = l
        self.Refresh()



