""" Core curses display manager.

This class handles user input and selection of the appropriate
display for the current situation, as well as Curses boilerplate logic.
"""

# STL
import os
import typing

# curses
import curses

# custom
from .menus.base import Menu
from .menus.main import MainMenu

# global variables - hardcoded limits on screen size requirements
# @TODO make these more than placeholders
MIN_HEIGHT = 10
MIN_WIDTH = 10

class DisplayManager:
    """
    """

    # all available menus
    menus = {
        "main" : MainMenu
    }

    def __init__(self, stdscr, ros):
        # curses window interface
        self.stdscr = stdscr

        # ros parser interface
        self.ros = ros

        # current menu item in focus
        self.current_menu = None

        # curses initialization
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, "Initializing...")
        self.stdscr.refresh()

    def update(self):
        """ Externally triggered update; expected to be called periodically.
        """
        # clear screen
        self.stdscr.erase()

        # determine which display to show
        if not self._get_screen_metadata():
            self.stdscr.addstr(0, 0, self._display_too_small())
        elif not self.ros.connected():
            self.stdscr.addstr(0, 0, self._display_disconnect())
        else:
            if not self.current_menu:
                # first time initialization of 'main' menu
                self.current_menu = self.menus["main"](self.stdscr, self.ros, self.menus)
            self.current_menu.update(0, 0, self.height, self.width)

        # refresh the display
        self.stdscr.refresh()

    def _get_screen_metadata(self):
        """ Update class variables to do with screen size.

        Returns:
            True if screen is big enough for our purposes, else False
        """
        self.height, self.width = self.stdscr.getmaxyx()
        if self.height >= MIN_HEIGHT and self.width >= MIN_WIDTH:
            return True
        return False

    def _display_too_small(self):
        """ Display warning that the screen is too small for a useful display.
        """
        return "Screen too small!"

    def _display_disconnect(self):
        """ Display that there's no connection to an active ROS interface.
        """
        msg = "No ROS connection found :("

        # get a list of salient ROS environment variables
        evs = [key for key in os.environ if "ROS" in key]

        # display useful environment variables
        if len(evs) != 0:
            msg += "\nRelevant environment variables:"
            for i,ev in enumerate(evs):
                msg += "\n\t{}={}".format(ev, os.environ[ev])
        return msg
