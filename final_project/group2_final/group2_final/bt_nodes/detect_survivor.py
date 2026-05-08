# Name: Yossaphat Kulvatunyou & Nam Facchetti
# Module: detect_survivor.py - BT action node that calls the detect_survivor service

import py_trees
from rclpy.node import Node
from group2_final_interfaces.srv import DetectSurvivor as DetectSurvivorSrv
from group2_final.zone_manager import ZoneManager


class DetectSurvivor(py_trees.behaviour.Behaviour):
    """BT action node that calls the detect_survivor service.

    Uses the async-poll pattern to avoid blocking the executor.
    Stores the detection result for use by IsSurvivorDetected and
    BroadcastSurvivorTF via was_found() and survivor_pose().

    """

    SERVICE_NAME = 'detect_survivor'
    SERVICE_TIMEOUT_SEC = 5.0

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """Initialize DetectSurvivor BT node.

        Args:
            name: Display name for the BT node.
            zone_manager: Shared ZoneManager instance.
        """
        super().__init__(name=name)
        self._zone_manager = zone_manager
        self._node: Node | None = None
        self._client = None
        self._future = None
        self._was_found = False
        self._survivor_xy = None
        self._current_survivor_id = ""


    def setup(self, **kwargs):
        """Create the detect_survivor service client.

        Args:
            **kwargs: Must contain 'node' (rclpy.node.Node).
        """
        # Called once before the first tick.
        # Acquire ROS 2 resources here (node, publishers, etc.)
        self._node = kwargs['node']
        self._client = self._node.create_client(DetectSurvivorSrv, 'detect_survivor')

    def initialise(self)-> None:
        """Reset state each time this node becomes active."""

        # Called each time the node become active again
        # Reset internal state here so past result dont get carry over
        self._future = None
        self._was_found = False
        self._survivor_xy = None
        

    def update(self) -> py_trees.common.Status:
        """Poll the detect_survivor service; return RUNNING/SUCCESS/FAILURE.

        Returns:
            RUNNING while waiting, SUCCESS when done, FAILURE on error.
        """
        # Called every tick while this node is active.
        # Return SUCCESS, FAILURE, or RUNNING.

        # 1st tick (no call made yet) : submit the request
        if self._future is None:
            if not self._client.service_is_ready(): # server running ??
                self._node.get_logger().warn("detect_survivor service not ready, retrying...")
                return py_trees.common.Status.RUNNING
            else: 
                zone_id = self._zone_manager.current_zone()['id']
                request = DetectSurvivorSrv.Request()
                request.zone_id = zone_id
                self._future = self._client.call_async(request)
                self._node.get_logger().info(f"DetectSurvivor: called service for '{zone_id}'")
                return py_trees.common.Status.RUNNING # running and waiting for response
        
        # 2nd tick : poll until server response and done
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        
        # final tick : response arrive , then read result
        response = self._future.result()
        self._future = None # reset storing val
        
        if response is None: #sth went wrong return failure
            return py_trees.common.Status.FAILURE
        
        self._was_found = response.found
        if response.found:
            self._survivor_xy = (response.survivor_x, response.survivor_y)
        else:
            self._survivor_xy = None  # no survivor = None, not (0.0, 0.0)

        if self._survivor_xy:
            sx, sy = self._survivor_xy
            self._node.get_logger().info(
                f"DetectSurvivor: found={self._was_found} at ({sx:.2f}, {sy:.2f})")
        else:
            self._node.get_logger().info(f"DetectSurvivor: found={self._was_found}")
        return py_trees.common.Status.SUCCESS

        
    def was_found(self) -> bool:
        """Return True if the last detection found a survivor.

        Returns:
            True if a survivor was detected.
        """
        return self._was_found

    def survivor_pose(self) -> tuple[float, float] | None:
        """ 
        Window into stored result here

        Returns:
            Tuple of (x, y) survivor coord.
        """
        return (self._survivor_xy)
    
        
    def set_survivor_id(self, survivor_id: str) -> None:
        """Store the TF frame name assigned to current survivor.

        Args:
            survivor_id: The TF frame name e.g. 'survivor_1'.
        """
        self._current_survivor_id = survivor_id

    def survivor_id(self) -> str:
        """Return the TF frame name assigned to current survivor.

        Returns:
            The survivor ID string e.g. 'survivor_1'.
        """
        return self._current_survivor_id
        
    def terminate(self, new_status: py_trees.common.Status) -> None:
        """No cleanup needed for this node."""
        pass