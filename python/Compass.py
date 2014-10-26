import wx
from math import sqrt, pi

class Compass(wx.PyControl):
    """
    A compass wx control. Points in the specified direction.
    """

    def __init__(self, parent, min_radius=50, id=wx.ID_ANY, pos=wx.DefaultPosition,
                 size=wx.DefaultSize, style=wx.NO_BORDER | wx.FULL_REPAINT_ON_RESIZE, validator=wx.DefaultValidator,
                 name="Compass"):
        """
        Default class constructor.

        @param parent: Parent window. Must not be None.
        @param id: Compass identifier. A value of -1 indicates a default value.
        @param pos: Compass position. If the position (-1, -1) is specified
                    then a default position is chosen.
        @param size: Compass size. If the default size (-1, -1) is specified
                     then a default size is chosen.
        @param style: not used in this demo
        @param validator: Window validator.
        @param name: Window name.
        """

        wx.PyControl.__init__(self, parent, id, pos, size, style, validator, name)

        # By default, we start pointing north
        self.dir = 0
        self.min_radius = min_radius

        self.InitializeBrushes()
        self.InheritAttributes()

        # Bind the events related to our control: first of all, we use a
        # combination of wx.BufferedPaintDC and an empty handler for
        # wx.EVT_ERASE_BACKGROUND (see later) to reduce flicker
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)


    def ApplyParentThemeBackground(self, colour):
         self.SetBackgroundColour(colour) 

    def InitializeBrushes(self):
        """ Initializes the brushes and pens """

        self.black_pen = wx.Pen('black', 1)
        self.trans_brush = wx.Brush('black', wx.TRANSPARENT)
        self.black_brush = wx.Brush('black', wx.SOLID)


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

        # Compute dimensions and origin
        xo = width/2
        yo = height/2
        rad = min(width/2, height/2) - 4

        # Initialize the wx.BufferedPaintDC, assigning a background
        # colour and a foreground colour (to draw the text)
        backColour = self.GetBackgroundColour()
        backBrush = wx.Brush(backColour, wx.SOLID)
        dc.SetBackground(backBrush)
        dc.Clear()

        # Create a graphics context that can be rotated and shifted
        gc = wx.GraphicsContext.Create(dc)

        # Set the pens and brushes
        gc.SetPen(self.black_pen)
        gc.SetBrush(self.trans_brush)

        # Move and rotate the graphics context
        gc.Translate(xo, yo)
        gc.Rotate( (180.0 + self.dir) / 180.0 * pi)

        # Draw the circle around the compass
        gc.DrawEllipse(-rad, -rad, rad*2, rad*2)

        # Define width of arrow head and tail
        hwd = rad*2 * 0.75
        twd = hwd / 3

        # Draw the arrow facing south
        arrow_pts = [(0, rad), (hwd/2, 0), (twd/2, 0), (twd/2, -sqrt(rad**2 - (twd/2)**2)), (-twd/2, -sqrt(rad**2 - (twd/2)**2)), (-twd/2, 0), (-hwd/2, 0), (0, rad)]
        arrow_wxpts = [wx.Point2D(*p) for p in arrow_pts]
        gc.SetBrush(self.black_brush)
        gc.DrawLines(arrow_wxpts, wx.ODDEVEN_RULE)


    def OnEraseBackground(self, event):
        """ Handles the wx.EVT_ERASE_BACKGROUND event for Compass. """

        pass


    def SetDirection(self, d):
        """ Sets the compass direction and performs a drawing refresh """

        self.dir = d
        self.Refresh()


    def DoGetBestSize(self):
        """
        Overridden base class virtual.  Determines the best size of the control
        """
        width, height = self.GetClientSize()
        new_width = max(self.min_radius*2, width)
        new_height = max(self.min_radius*2, height)
                
        best = wx.Size(new_width, new_height)

        # Cache the best size so it doesn't need to be calculated again,
        # at least until some properties of the window change

        self.CacheBestSize(best)

        return best


