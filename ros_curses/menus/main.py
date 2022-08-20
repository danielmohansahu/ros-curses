""" Main Menu class implementation.
"""

# STL
from string import Template

# ros_curses
from .base import Menu

# template for the main window display
message = Template(
"""
 ROS Curses: A tool for command line debugging live ROS computational graphs.
 Press '?' for navigation information.

 See https://github.com/danielmohansahu/ros-curses for more info.

Available Utilities:
    ${menus}
""")

class MainMenu(Menu):
    """ Main Menu display; default display on startup.

    Contains information about available plugins, as well as basic instructions.
    """
    def __init__(self, stdscr, ros, menus):
        # initialize super class
        super().__init__(stdscr, ros, menus)

    def update(self, y, x, height, width, keystroke = None):
        """ Update main menu window display.
        """
        # display
        self.stdcr.addstr(y, x, message.substitute(menus="\n\t".join(self.menus.keys())))