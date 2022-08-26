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

        # initialize persistent data
        self._nodes = ROSNodesData()
    
    def connected(self):
        """ Returns True if there's an active ROS connection, else False
        """
        return rosgraph.is_master_online()

    def nodes(self):
        return self._nodes

    def update(self):
        """ Poll the rosgraph for updated ROS Primitive information.

        References:
            https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosgraph/src/rosgraph/masterapi.py
        """
        # return quietly if not connected; this is an acceptable state
        if not self.connected():
            return

        # update list of nodes and information
        #  note there's a lot of redundancy in our data structures
        publishers,subscribers,services = self._master.getSystemState()

        # process topic publishers
        for topic, pubs in publishers:
            for pub in pubs:
                if pub not in self._nodes:
                    self._nodes[pub] = ROSNodeData(pub)
                if topic not in self._nodes[pub].topics_published_full:
                    self._nodes[pub].topics_published_full.append(topic)

        # process topic subscribers
        for topic, subs in subscribers:
            for sub in subs:
                if sub not in self._nodes:
                    self._nodes[sub] = ROSNodeData(sub)
                if topic not in self._nodes[sub].topics_subscribed_full:
                    self._nodes[sub].topics_subscribed_full.append(topic)

        # process services
        for service, servers in services:
            for server in servers:
                if server not in self._nodes:
                    self._nodes[server] = ROSNodeData(server)
                if service not in self._nodes[server].services_advertised_full:
                    self._nodes[server].services_advertised_full.append(service)
