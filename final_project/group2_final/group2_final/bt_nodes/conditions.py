# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: conditions.py - Defines Behavior Tree condition nodes for checking remaining zones and survivor detection status in the search-and-rescue mission. These nodes interact with the shared ZoneManager and DetectSurvivor action node to determine their status.

import py_trees
from py_trees.common import Status
from typing import Optional
from rclpy.node import Node

from group2_final.zone_manager import ZoneManager
from group2_final.bt_nodes.actions import DetectSurvivor


class ZonesRemaining(py_trees.behaviour.Behaviour):
    """
    Behavior Tree condition node that checks if there are remaining
    search zones to visit.

    Returns SUCCESS if there are unvisited zones, otherwise FAILURE.
    """

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """
        Initialize the ZonesRemaining node.

        Args:
            name (str): Name of the behavior tree node.
            zone_manager (ZoneManager): Shared zone manager instance.
        """
        super().__init__(name)
        self._zone_manager = zone_manager   # Reference to the shared ZoneManager for checking remaining zones
        self._node: Optional[Node] = None

    def setup(self, **kwargs) -> None:
        """
        Setup ROS 2 resources.

        Stores the node handle for logging.
        """
        self._node = kwargs["node"]

    def update(self) -> Status:
        """
        Check if there are remaining zones.

        Returns:
            Status.SUCCESS: If zones remain.
            Status.FAILURE: If all zones have been visited.
        """
        if self._zone_manager.has_remaining():   # Check if there are unvisited zones in the ZoneManager
            return Status.SUCCESS
        else:
            self._node.get_logger().info(   # Log that all zones have been visited and the robot is returning to base
                "All zones visited. Returning to base."
            )
            return Status.FAILURE


class IsSurvivorDetected(py_trees.behaviour.Behaviour):
    """
    Behavior Tree condition node that checks if a survivor was detected
    in the current zone.

    Reads the detection result from the DetectSurvivor action node.

    Returns SUCCESS if a survivor was found, otherwise FAILURE.
    """

    def __init__(self, name: str, detect_node: DetectSurvivor) -> None:
        """
        Initialize the IsSurvivorDetected node.

        Args:
            name (str): Name of the behavior tree node.
            detect_node (DetectSurvivor): Reference to the DetectSurvivor BT node.
        """
        super().__init__(name)
        self._detect_node = detect_node   # Reference to the DetectSurvivor node for checking detection results
        self._node: Optional[Node] = None

    def setup(self, **kwargs) -> None:
        """
        Setup ROS 2 resources.

        Stores the node handle for logging.
        """
        self._node = kwargs["node"]

    def update(self) -> Status:
        """
        Check if a survivor was detected.

        Returns:
            Status.SUCCESS: If a survivor was found.
            Status.FAILURE: If no survivor was found.
        """
        if self._detect_node.was_found():   # Check if the DetectSurvivor node has a positive detection result
            return Status.SUCCESS
        else:
            return Status.FAILURE