""" Display Formatter

This class is in charge of formatting input text / data into the "best"
representation available for the given bounds.
"""

# STL
import typing

# curses
import curses

class Formatter:
    def __init__(self, stdscr):
        self.stdscr = stdscr

    def display(self, msg, height, width):
        """ Format the text to fit within the given bounds.
        """
        return msg
