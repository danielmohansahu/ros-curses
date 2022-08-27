""" ROS Nodes Window Display Class

The class defined herein defines the interface for a
window summarizing information about all ROS Nodes
currently detected in the computational graph.
"""

# STL
import enum

# curses
import curses

# ros_curses
from ros_curses.common.types import ROSNodeData, ROSNodesData
from ros_curses.windows.window_formatter import WindowFormatter


class NodesWindow:
    @enum.unique
    class Action(enum.Enum):
        """ Enumeration of available user actions.
        """
        NONE = enum.auto()      # no action requested
        EXIT = enum.auto()      # exit immediately
        HELP = enum.auto()      # display help panel
        DECREMENT = enum.auto() # decrement selection in active display
        INCREMENT = enum.auto() # increment selection in active display
        SWITCH = enum.auto()    # switch active display

    def __init__(self, stdscr, ros, active_display="list", active_node=None):
        # one time initialization
        self.stdscr = stdscr
        self.ros = ros

        # initialize available sub-windows
        self._displays = {
            "list" : WindowFormatter(),
            "info" : WindowFormatter()        
        }

        # set initial "state" variables - these track the user's current selected items
        self._active_display = active_display
        self._active_node = active_node
        self._active_info_item = None

        # resize (sets class variables)
        self.resize()

    def resize(self):
        # split display horizontally based on available size

        # otherwise update window sizes (horizontal split)
        rows,cols = self.stdscr.getmaxyx()
        self._displays["list"].resize(rows, cols // 2, 0, 0, self._active_display == "list")
        self._displays["info"].resize(rows, cols // 2, 0, cols // 2, self._active_display == "info")

    def render(self):
        """ Main polling API - rerender window.
        """
        # variable to determine if we need to completely redraw the window
        redraw = False

        # update ros variables and get current nodes
        self.ros.update()
        nodes = self.ros.nodes()

        # resize, if necessary
        self.resize()

        # process user input
        action = self._process_user_input()
        if action == self.Action.EXIT:
            return None
        elif action == self.Action.HELP:
            # @TODO
            ...
        elif action == self.Action.DECREMENT:
            # perform switching logic based on current display
            if self._active_display == "list":
                self._active_node = nodes.decrement(self._active_node)
            elif self._active_node in nodes:
                self._active_info_item = nodes[self._active_node].decrement(self._active_info_item)
        elif action == self.Action.INCREMENT:
            # perform switching logic based on current display
            if self._active_display == "list":
                self._active_node = nodes.increment(self._active_node)
            elif self._active_node in nodes:
                self._active_info_item = nodes[self._active_node].increment(self._active_info_item)        
        elif action == self.Action.SWITCH:
            self._active_display = "info" if self._active_display == "list" else "list"
            redraw = True

        # draw "list" window
        self._render_list_window(nodes)

        # draw "info" window
        self._render_info_window(nodes)

        # refresh and return
        for display in self._displays.values():
            display.refresh()
        return self

    def _process_user_input(self):
        # handle user input and return an appropriate action
        user_input = self._displays[self._active_display].getch()
        if user_input != curses.ERR:
            if user_input == ord('?'):
                return self.Action.HELP
            elif user_input == ord('\t'):
                return self.Action.SWITCH
            elif user_input == curses.KEY_UP:
                return self.Action.DECREMENT
            elif user_input == curses.KEY_DOWN:
                return self.Action.INCREMENT
            elif user_input == ord('q'):
                return self.Action.EXIT
            else:
                # debugging
                self._displays[self._active_display].write_line(-1, f"Got unknown key {chr(user_input)}")
        # do nothing
        return self.Action.NONE

    def _render_list_window(self, nodes):
        # this method contains all logic for drawing and formatting the "info" window

        # check that active node is valid, or set to first
        if self._active_node not in nodes:
            self._active_node = nodes.increment(self._active_node)

        # just display a simple list
        self._displays["list"].write_line(0, "ROS Nodes:")
        for idx, node in enumerate(nodes):
            self._displays["list"].write_line(idx + 1, f"  {node}", node == self._active_node)

    def _render_info_window(self, nodes):
        # this method contains all logic for drawing and formatting the "info" window

        # current line index
        idx = 0

        # handle edge case
        if self._active_node not in nodes:
            self._displays["info"].write_line(idx, f"No ROS Nodes detected...")
            return
        node_data = nodes[self._active_node]

        # display node summary information
        idx = self._displays["info"].write_line(idx, f"ROS Node '{node_data.name}':")
        idx = self._displays["info"].write_line(idx, f"  full name: {node_data.fullname}")
        idx = self._displays["info"].write_line(idx, f"  namespace: {node_data.namespace}")
        idx = self._displays["info"].write_line(idx, "")

        # display publications information
        idx = self._displays["info"].write_line(idx, f"Publications:")
        if len(node_data.topics_published_full) == 0:
            idx = self._displays["info"].write_line(idx, f"  None")
        else:
            for topic in node_data.topics_published_full:
                idx = self._displays["info"].write_line(idx, f"    {topic}")
        idx = self._displays["info"].write_line(idx, "")

        # display publications information
        idx = self._displays["info"].write_line(idx, f"Subscriptions:")
        if len(node_data.topics_subscribed_full) == 0:
            idx = self._displays["info"].write_line(idx, f"  None")
        else:
            for topic in node_data.topics_subscribed_full:
                idx = self._displays["info"].write_line(idx, f"    {topic}")
        idx = self._displays["info"].write_line(idx, "")

        # display services information
        idx = self._displays["info"].write_line(idx, f"Services:")
        if len(node_data.services_advertised_full) == 0:
            idx = self._displays["info"].write_line(idx, f"  None")
        else:
            for service in node_data.services_advertised_full:
                idx = self._displays["info"].write_line(idx, f"    {service}")
        idx = self._displays["info"].write_line(idx, "")

        # display parameter information
        idx = self._displays["info"].write_line(idx, f"Parameters in Namespace:")
        if len(node_data.parameters) == 0:
            idx = self._displays["info"].write_line(idx, f"  None")
        else:
            for param in node_data.parameters:
                idx = self._displays["info"].write_line(idx, f"    {param}")
