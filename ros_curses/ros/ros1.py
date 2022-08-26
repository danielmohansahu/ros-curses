""" ROS1 Interface Class

This class polls the ROS computational graph for salient information
and parses said info into a usable format for the rest of the system.
"""

# STL

# ROS
import rosgraph

# ros_curses
from ros_curses.common.types import ROSNodesData, ROSNodeData

class ROS1Interface:
    def __init__(self):
        # ROSMaster communication API
        self._master = rosgraph.Master("/rosnode")
    
    def connected(self):
        """ Returns True if there's an active ROS connection, else False
        """
        return rosgraph.is_master_online()
