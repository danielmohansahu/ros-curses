""" Common Datatypes
"""

# STL
from typing import List, Dict
import dataclasses
from collections import UserDict

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

class ROSNodesData(dict):
    """ Encapsulation of ROSNode information.

    @TODO decide if this should just be a normal dict...
    """