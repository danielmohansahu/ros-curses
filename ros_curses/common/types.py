""" Common Datatypes
"""

# STL
import itertools
from typing import List, Dict
import dataclasses
from collections import OrderedDict

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
    parameters: List[str] = None    # parameters with a shared namespace; not perfect

    def __post_init__(self):
        """ Parse supplied initial data into components.
        """
        # update name metadata
        self.name = self.fullname.split("/")[-1]
        self.namespace = self.fullname.replace(self.name, "")

        # populate defaults with empty lists
        for field in dataclasses.fields(ROSNodeData):
            if getattr(self, field.name) is None:
                setattr(self, field.name, [])

    def increment(self, item):
        # @TODO
        return item

    def decrement(self, item):
        # @TODO
        return item

class ROSNodesData(dict):
    """ Encapsulation of ROSNode information.
    """
    def increment(self, item):
        """ Convenience method to return the next item.
        """
        # handle edge cases gracefully - return first item or None
        if item not in self:
            return next(iter(self)) if len(self) != 0 else None
        
        # otherwise, iterate through our data structure and return next after match
        keys = list(self.keys())
        for i,key in enumerate(keys):
            if key == item:
                return keys[(i + 1) % len(self)]
        assert (False), "Reached unreachable code block."

    def decrement(self, item):
        """ Convenience method to return the previous item.
        """
        # handle edge cases gracefully - return first item or None
        if item not in self:
            return next(iter(self)) if len(self) != 0 else None
        
        # otherwise, iterate through our data structure and return next after match
        keys = list(self.keys())
        for i,key in enumerate(keys):
            if key == item:
                return keys[(i - 1) % len(self)]
        assert (False), "Reached unreachable code block."