"""BT action nodes: DetectSurvivor, BroadcastSurvivorTF, NotifyBase."""

import py_trees
import rclpy
from rclpy.node import Node
from group2_final_interfaces.srv import DetectSurvivor as DetectSurvivorSrv
from group2_final_interfaces.srv import ReportSurvivor as ReportSurvivorSrv
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from group2_final.zone_manager import ZoneManager


class DetectSurvivor(py_trees.behaviour.Behaviour):
    """BT action node that calls the detect_survivor service.

    Uses the async-poll pattern to avoid blocking the executor.
    Stores the detection result for use by IsSurvivorDetected and
    BroadcastSurvivorTF via was_found() and survivor_pose().

    Args:
        name: Display name for the BT node.
        zone_manager: Shared ZoneManager instance.
    """

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initialize DetectSurvivor BT node."""
        super().__init__(name=name)