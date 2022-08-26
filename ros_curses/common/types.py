""" Common Datatypes
"""

# STL
from typing import List, Dict
from dataclasses import dataclass

@dataclass
class ROSNodeData:
    """ Encapsulation of salient information for a ROS Node
    """

    # node information
    name: str               # node name (without namespaces)
    fullname: str           # fully resolved node name
    namespace: str          # node namespace

    # topic connection information
    topics_subscribed_full: List[str]   # all topic subscriptions
    topics_published_full: List[str]    # all topic publications
    topics_subscribed: List[str]        # filtered topic subscriptions (ignoring dynamic reconfigure, etc.)
    topics_published: List[str]         # filtered topic publications (ignoring dynamic reconfigure, etc.)

    # service connection information
    services_advertised_full: List[str] # all advertised services
    services_advertised: List[str]      # filtered advertised services (ignoring dynamic reconfigure, etc.)

    # parameters
    params: List[str]       # parameters with a shared namespace; not perfect

@dataclass
class ROSNodesData:
    """ Encapsulation of salient information for all available ROS Nodes
    """

    # nodes information
    nodes: Dict[str, ROSNodeData]

