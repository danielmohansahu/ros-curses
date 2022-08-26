""" ROS Nodes Window Display Class

The class defined herein defines the interface for a
window summarizing information about all ROS Nodes
currently detected in the computational graph.
"""

# STL

# curses
import curses

# ros_curses
from ros_curses.common.types import ROSNodeData, ROSNodesData
from ros_curses.windows.window_formatter import WindowFormatter

# hardcoded list of nodes for testing
NODES = ["node1","node2", "smol", "an_eCTRELMELY_lrg NODE name with !c 3290  weird chars."]

class NodesWindow:
    def __init__(self, stdscr):
        # one time initialization
        self.stdscr = stdscr

        # initialize class variables to None
        self._size = None
        self._displays = {
            "list" : WindowFormatter(),
            "info" : WindowFormatter()            
        }
        self._active_display = "list"
        self._highlighted_node_idx = 0

        # resize (sets class variables)
        self.resize()

    def resize(self):
        # split display based on available size

        # get current available size
        size = self.stdscr.getmaxyx()

        # check if we don't need to update
        if size == self._size:
            return

        # otherwise update window sizes (horizontal split)
        self._size = size
        rows, cols = self._size
        self._displays["list"].resize(rows, cols // 2, 0, 0)
        self._displays["info"].resize(rows, cols // 2, 0, cols // 2)

    def render(self):
        # render new window

        # resize, if necessary
        self.resize()
        
        # set topics
        self._displays["list"].write_line(0, "ROS Nodes:")
        for idx, node in enumerate(NODES):
            self._displays["list"].write_line(idx + 1, f"  {node}", idx == self._highlighted_node_idx)
                
        # get user input (and exit if requested)
        user_input = self._displays[self._active_display].getch()
        if user_input != curses.ERR:
            if user_input == ord('q'):
                return None
            # elif user_input == ord('\t'):
            #     self._active_display = "list" if self._active_display == "info" else "info"
            #     self._displays[self._active_display].addch(0, 0, user_input)
            elif user_input == curses.KEY_UP:
                self._highlighted_node_idx = (self._highlighted_node_idx - 1) % len(NODES)
            elif user_input == curses.KEY_DOWN:
                self._highlighted_node_idx = (self._highlighted_node_idx + 1) % len(NODES)
            else:
                # debugging
                self._displays[self._active_display].write_line(-1, str(user_input))


        # refresh displays
        self._displays["list"].refresh()
        self._displays["info"].refresh()

        return self
