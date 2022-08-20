""" ROS Computational Graph parser.

The class defined here handles querying, inspecting, and parsing data from the
ROS Computational Graph.
"""

# STL
import typing

# ROS
import rospy
import rosgraph

class CGParser:

    def __init__(self):
        pass

    def connected(self):
        """ Check if we have an active ROS connection.
        """
        return True
