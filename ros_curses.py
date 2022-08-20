#!/usr/bin/env python3
""" Curses CLI for ROS Computational Graph Debugging.
"""

# argument parsing
import time
import typing
import argparse

# curses
import curses

# ros_curses modules
from ros_curses.curses.display_manager import DisplayManager
from ros_curses.ros.cg_parser import CGParser

def main(stdscr):
    ros = CGParser()
    display = DisplayManager(stdscr, ros)

    # periodically poll
    try:
        while True:
            display.update()
            time.sleep(0.01)
    except KeyboardInterrupt:
        # exit silently
        return

if __name__ == "__main__":
    # curses initialization and graceful error handling
    curses.wrapper(main)