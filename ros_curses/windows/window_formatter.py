""" WindowFormatter Utility Class

This class provides an interface for formatting the display for a given
window. The design goal here is to provide a simplified interface around
the default curses.Window class to handle the expected format in this
project.
"""

# STL

# curses
import curses

class WindowFormatter:
    """ Window abstraction / wrapper to handle lower level formatting.
    """
    def __init__(self, height=0, width=0, origin_y=0, origin_x=0, active=False):
        # save state
        self._height = height
        self._width = width
        self._origin_y = origin_y
        self._origin_x = origin_x
        self._active = active

        # create new window at the given location
        self._window = curses.newwin(height, width, origin_y, origin_x)

        # set default window parameters
        self._window.keypad(True)
        self._window.nodelay(True)
        if active:
            self._window.box()
    
    def resize(self, height, width, origin_y, origin_x, active):
        """ Resize window and move to the new location.
        """
        changed = False

        # resize (if required)
        if (height != self._height or width != self._width):
            self._height, self._width = height, width
            self._window.resize(self._height, self._width)
            changed = True

        # move, if required
        if (origin_y != self._origin_y or origin_x != self._origin_x):
            self._origin_y, self._origin_x = origin_y, origin_x
            self._window.mvwin(self._origin_y, self._origin_x)
            changed = True

        # redraw or erase box, if required
        if self._active != active:
            self._active = active
            changed = True

        if changed:
            self._window.clear()
            if self._active:
                self._window.box()

    def size(self):
        """ Returns the size of available space in the window, i.e. excluding borders.
        """
        return (self._height - 2, self._width - 2)

    def write_line(self, row, string, highlight = False):
        """ Write the given string at the target row.

        Text requiring a greater width than is available is truncated. Text shorted
        than number of available columns is padded with whitespace.

        Args:
            row: The row to write this line, indexed from the first _available_ row.
            string: The string to display; should include all desired whitespace.
            highlight: Highlight the display text (excluding whitespace)

        Returns:
            idx: The next row index, for convenience
        """
        idx = row

        # handle wrapping indices (-1, -2)
        max_rows, max_cols = self.size()
        if row < 0:
            row = max_rows + row

        # don't do anything if this is outside our display bounds
        if 0 > row > max_rows:
            return idx

        # add a string to the given row
        self._window.addnstr(row + 1, 1, string, max_cols)

        # padd with whitespace, if necessary
        if len(string) < max_cols:
            self._window.addnstr(row + 1, 1 + len(string), " " * (max_cols - len(string)), max_cols)

        if highlight:
            col = next(idx for idx,c in enumerate(string) if not c.isspace())
            length = min(len(string), max_cols) - col
            self._window.chgat(row + 1, col + 1, length, curses.A_STANDOUT)
        
        return idx + 1

    def __getattr__(self, attr):
        """ Allow direct access to curses.Window class for undefined methods.
        """
        if hasattr(self._window, attr):
            return getattr(self._window, attr)
        raise AttributeError(f"No method or attribute '{attr}' defined.")
