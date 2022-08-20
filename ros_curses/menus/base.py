""" Base menu window class.
"""

# STL
import abc

class Menu(abc.ABC):
    """ Base menu class for a curses window.
    """

    def __init__(self, stdscr, ros, menus):
        # initialize common required objects
        self.stdcr = stdscr
        self.ros = ros
        self.menus = menus

    @abc.abstractmethod
    def update(self, y, x, height, width, keystroke=None):
        """ Rerender this window to fit the given dimensions.

        Args:
            y           : New Y origin coordinate.
            x           : New X origin coordinate.
            height      : New window height.
            width       : New window width.
            keystroke   : User input, if supplied.

        Returns:
            (menu, args): Optional tuple indicating new menu to display, with the given args.
        """
        ...
