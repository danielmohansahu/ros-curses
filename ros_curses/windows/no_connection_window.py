""" No Connection Window Display Class

This class just indicates that we don't have a connection to
an existing ROS Master.
"""

# STL
import os
import string

# curses
import curses

# ros_curses
from ros_curses.windows.window_formatter import WindowFormatter

# template for the main window display
MESSAGE = string.Template(
"""
  ROS Curses: A tool for command line debugging live ROS computational graphs.

  Waiting for an active ROS connection... if this is unexpected please check your network.

  Relevant Environment Variables:\n\t${vars}
""")

class NoConnectionWindow:
    def __init__(self, stdscr):
        # one time initialization
        self.stdscr = stdscr

    def render(self):
        # get some debugging information
        relevant_evs = {k:v for k,v in os.environ.items() if "ROS_" in k}

        # format EVs nicels
        evs_string = "\n\t".join([f"{k}:\t{v}" for k,v in relevant_evs.items()])

        # print formatted blurb
        self.stdscr.addstr(0, 0, MESSAGE.substitute(vars=evs_string))

        # get user input (and exit if requested)
        if self.stdscr.getch() == ord('q'):
            return None

        # return pointer to self        
        return self
