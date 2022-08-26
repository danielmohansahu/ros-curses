""" Common Datatypes
"""

# STL
from typing import List, Dict
import dataclasses

@dataclasses.dataclass
class ROSNodeData:
    """ Encapsulation of salient information for a ROS Node
    """

    # node information
    fullname: str           # fully resolved node name
    name: str = None        # node name (without namespaces)
    namespace: str = None   # node namespace

    # topic connection information
    topics_subscribed_full: List[str] = None  # all topic subscriptions
    topics_published_full: List[str] = None   # all topic publications
    topics_subscribed: List[str] = None       # filtered topic subscriptions (ignoring dynamic reconfigure, etc.)
    topics_published: List[str] = None        # filtered topic publications (ignoring dynamic reconfigure, etc.)

    # service connection information
    services_advertised_full: List[str] = None    # all advertised services
    services_advertised: List[str] = None         # filtered advertised services (ignoring dynamic reconfigure, etc.)

    # parameters
    params: List[str] = None  # parameters with a shared namespace; not perfect

    def __post_init__(self):
        """ Parse supplied initial data into components.
        """
        # update name metadata
        name = self.fullname.split("/")[-1]
        namespace = self.fullname.replace(name, "")

        # populate defaults with empty lists
        for field in dataclasses.fields(ROSNodeData):
            if getattr(self, field.name) is None:
                setattr(self, field.name, [])

@dataclasses.dataclass
class ROSNodesData:
    """ Encapsulation of salient information for all available ROS Nodes
    """

    # nodes information
    nodes: Dict[str, ROSNodeData] = None

    def __post_init__(self):
        # if no data was supplied, set to empty list
        self.nodes = [] if self.nodes is None else self.nodes

    # allow iteration over underlying data
    def __iter__(self):
        return self.nodes.__iter__()

    # access length of underlying data
    def __len__(self):
        return self.nodes.__len__()