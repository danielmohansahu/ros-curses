#!/usr/bin/env python3
""" Curses CLI for ROS Computational Graph Debugging.
"""

# STL
import time

# curses
import curses

# ros_curses
from windows.nodes_window import NodesWindow






# core program execution
def main(stdscr):
    # set some non-default curses args
    curses.curs_set(0)  # normally hide the cursor

    # start with the default window type
    current_window = NodesWindow(stdscr).render()

    # periodically poll
    try:
        while True:
            current_window = current_window.render()
            
            # check if it's time to exit
            if current_window is None:
                break

            # otherwise sleep until next loop
            time.sleep(0.01)

    except KeyboardInterrupt:
        # exit silently
        return

if __name__ == "__main__":
    # curses initialization and graceful error handling
    curses.wrapper(main)
