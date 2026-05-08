# Name: Yossaphat Kulvatunyou & Nam Facchetti
# Module: notify_base.py - BT action node that calls the report_survivor service.

import py_trees
from group2_final_interfaces.srv import ReportSurvivor as ReportSurvivorSrv
from group2_final.bt_nodes.detect_survivor import DetectSurvivor

from geometry_msgs.msg import PointStamped


class NotifyBase(py_trees.behaviour.Behaviour):
    """Uses the async-poll pattern to avoid blocking the executor.
    Reads survivor ID and pose from detect_node and reports to the
    simulated command center via the report_survivor service.

    Args:
        name: Display name for the BT node.
        detect_node: Reference to the DetectSurvivor BT node.
    """
    SERVICE_NAME = 'report_survivor'
    SERVICE_TIMEOUT_SEC = 5.0

    def __init__(self, name: str, detect_node: DetectSurvivor ) -> None:
        """Initialize NotifyBase BT node.
        
        Args:
            name: Display name for the BT node.
            detect_node: Reference to the DetectSurvivor BT node.
        """
        super().__init__(name=name)
        self._detect_node = detect_node
        self._node = None
        self._client = None
        self._future = None
       
    def setup(self, **kwargs)-> None:
        """Create the report_survivor service client.

        Args:
            **kwargs: Must contain 'node' (rclpy.node.Node).
        """
        # Called once before the first tick.
        # Acquire ROS 2 resources here (node, publishers, etc.)
        self._node = kwargs['node']
        self._client = self._node.create_client(ReportSurvivorSrv, 'report_survivor')
        pass

    def initialise(self)-> None:
        """Reset future each time this node becomes active."""
        # Called each time the node become active again
        self._future = None


    def update(self) -> py_trees.common.Status:
        """Send survivor report

        Returns:
            RUNNING while waiting, SUCCESS if acknowledged, FAILURE otherwise.
        """
        # Called every tick while this node is active.

        # 1st tick (no call made yet) : submit the request
        if self._future is None:
            if not self._client.service_is_ready(): # server running ??
                self._node.get_logger().warn("report_survivor service not ready, retrying...")
                return py_trees.common.Status.RUNNING
            
            pose = self._detect_node.survivor_pose()
            survivor_id = self._detect_node.survivor_id()

            if pose is None or not survivor_id:
                return py_trees.common.Status.FAILURE
            
            x, y = pose
            location = PointStamped()
            location.header.frame_id = "map"    # coordinates are in map frame
            location.header.stamp = self._node.get_clock().now().to_msg()

            location.point.x = x 
            location.point.y = y
            location.point.z = 0.0
            
            request = ReportSurvivorSrv.Request()
            request.survivor_id = survivor_id
            request.location = location

            self._future = self._client.call_async(request)
            self._node.get_logger().info(f"NotifyBase: reporting '{request.survivor_id}' at ({x:.2f}, {y:.2f})")
            return py_trees.common.Status.RUNNING # running and waiting for response
        
        # 2nd tick : poll until server response and done
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        
        # final tick : response arrive , then read result
        response = self._future.result()
        self._future = None # reset storing val
        
        if response is None: #sth went wrong return failure
            return py_trees.common.Status.FAILURE
        
        if not response.acknowledged: # server said "not acknowledged"
            return py_trees.common.Status.FAILURE

        self._node.get_logger().info("NotifyBase: command center acknowledged")
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status: py_trees.common.Status) -> None:
        """No cleanup needed for this node."""
        pass