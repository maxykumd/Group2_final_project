# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: actions.py - Defines Behavior Tree action nodes for navigating to zones, advancing zones, and handling survivor detection in the search-and-rescue mission. These nodes interact with the shared ZoneManager and ROS 2 action servers/services to perform their tasks.

import math
from typing import Optional

import py_trees
from py_trees.common import Status

from rclpy.node import Node

from group2_final_interfaces.srv import DetectSurvivor as DetectSurvivorSrv
from group2_final_interfaces.srv import ReportSurvivor as ReportSurvivorSrv

from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from tf2_ros import StaticTransformBroadcaster

from group2_final.zone_manager import ZoneManager
from action_msgs.msg import GoalStatus


class NavigateToZone(py_trees.behaviour.Behaviour):
    """
    Behavior Tree action node that sends a Nav2 NavigateToPose goal
    to the current search zone.

    This node uses an asynchronous ActionClient to avoid blocking the
    behavior tree tick loop. It returns RUNNING while navigation is in
    progress, SUCCESS when the robot reaches the goal, and FAILURE if
    navigation fails.

    Attributes:
        _zone_manager (ZoneManager): Shared mission state manager.
        _node (Node): ROS 2 node (injected via setup).
        _client (ActionClient): Nav2 NavigateToPose action client.
        _goal_future (Optional[Future]): Future for goal request.
        _result_future (Optional[Future]): Future for result.
        _goal_handle: Handle to the active goal.
        _done (bool): Whether navigation is complete.
        _success (bool): Whether navigation succeeded.
    """

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """
        Initialize the NavigateToZone node.

        Args:
            name (str): Name of the behavior tree node.
            zone_manager (ZoneManager): Shared zone manager instance.
        """
        super().__init__(name)
        self._zone_manager = zone_manager  # Reference to the shared ZoneManager for accessing the current zone

        self._node: Optional[Node] = None  # ROS node will be injected in setup()
        self._client: Optional[ActionClient] = (
            None  # Nav2 ActionClient will be created in setup()
        )

        self._goal_future = None  # Future for the goal request
        self._result_future = None  # Future for the result of the goal
        self._goal_handle = None  # Handle for the active goal

        self._done: bool = (
            False  # Whether navigation has completed (success or failure)
        )
        self._success: bool = False  # Whether navigation succeeded (reached goal)
        self._retry_count: int = 0
        self._max_retries: int = 5

    def setup(self, **kwargs) -> None:
        """
        Setup ROS 2 resources (called once before ticking begins).

        Creates the Nav2 ActionClient.
        """
        self._node = kwargs[
            "node"
        ]  # Get the ROS node from the setup kwargs (injected by the BT framework)

        self._client = ActionClient(self._node, NavigateToPose, "navigate_to_pose")

        self._node.get_logger().info("NavigateToZone node setup complete.")

    def initialise(self) -> None:
        """
        Called when the node transitions from idle to active.

        Sends a new navigation goal to the current zone.
        """
        self._done = False
        self._success = False
        self._goal_future = None
        self._result_future = None
        self._goal_handle = None

        zone = self._zone_manager.current_zone()
        total = self._zone_manager.total_zones()
        current_idx = self._zone_manager.current_index() + 1
        self._node.get_logger().info(
            f"--- Zone {current_idx}/{total}: {zone['id']} "
            f"({zone['x']:.2f}, {zone['y']:.2f}) ---"
        )
        self._node.get_logger().info(f"Navigating to {zone['id']}... ")

        if not self._client.wait_for_server(
            timeout_sec=2.0
        ):  # Wait for Nav2 action server (non-blocking safe here)
            self._node.get_logger().error("NavigateToPose action server not available!")
            self._done = True
            self._success = False
            return

        goal_msg = (
            NavigateToPose.Goal()
        )  # Populate the goal message with the target pose from the zone manager
        goal_msg.pose = self._build_pose_stamped(  # Helper function to convert zone coordinates to PoseStamped
            zone["x"], zone["y"], zone["yaw"]
        )

        self._goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=None
        )

    def update(self) -> Status:
        """
        Called on every behavior tree tick.

        Returns:
            Status: RUNNING, SUCCESS, or FAILURE depending on navigation state.
        """
        if self._goal_future is None:  # If goal hasn't been sent yet (edge case)
            return Status.RUNNING

        if (
            self._goal_handle is None
        ):  # Check for if the goal has been sent and accepted but we don't have a handle yet
            if not self._goal_future.done():
                return Status.RUNNING

            self._goal_handle = self._goal_future.result()

            if not self._goal_handle.accepted:  # Check if the goal was accepted by Nav2
                self._node.get_logger().error("Navigation goal rejected!")
                self._done = True
                self._success = False
                return Status.FAILURE

            self._result_future = (
                self._goal_handle.get_result_async()
            )  # Request result asynchronously, goal accepted, now wait for the result
            return Status.RUNNING

        if (
            self._result_future is not None
        ):  # Check if we have a result future to wait on (should be set if goal was accepted)
            if not self._result_future.done():
                return Status.RUNNING

            result = self._result_future.result()  # Get the result of the navigation goal and check the status code to determine success or failure
            status = result.status  # Nav2 status code for the navigation result

            if status == GoalStatus.STATUS_SUCCEEDED:  # SUCCEEDED
                zone = (
                    self._zone_manager.current_zone()
                )  # Get the current zone info for logging
                self._node.get_logger().info(f"Reached {zone['id']}.")
                self._success = True  # Mark navigation as successful and done
                self._done = True  # Mark as done to prevent further processing
                return Status.SUCCESS
            else:
                self._node.get_logger().error(
                    f"Navigation failed with status: {status}"
                )
                self._success = False  # Mark navigation as failed and done
                self._done = True  # Mark as done to prevent further processing
                return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        """
        Called when the behavior is interrupted or finishes.

        Cancels the navigation goal if still active.

        Args:
            new_status (Status): The new status after termination.
        """
        if (
            self._goal_handle is not None  # Active goal handle
            and not self._done  # Navigation is not already marked as done (success or failure)
        ):
            self._node.get_logger().warn("Cancelling active navigation goal...")
            self._goal_handle.cancel_goal_async()  # Cancel the goal asynchronously to avoid blocking the BT tick loop

    def _build_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        Construct a PoseStamped message for Nav2.

        Args:
            x (float): X position in meters.
            y (float): Y position in meters.
            yaw (float): Orientation in radians.

        Returns:
            PoseStamped: Goal pose in the map frame.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self._node.get_clock().now().to_msg()

        pose.pose.position.x = (
            x  # Set the position from the zone manager's current zone coordinates
        )
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # Assuming flat ground, z is 0

        qz = math.sin(yaw / 2.0)  # Convert yaw to quaternion (assuming roll=pitch=0)
        qw = math.cos(yaw / 2.0)  # Quaternion for yaw rotation only

        pose.pose.orientation.z = (
            qz  # Set the orientation from the zone manager's current zone yaw
        )
        pose.pose.orientation.w = (
            qw  # Set the orientation w component for the quaternion
        )

        return pose


class NavigateToBase(py_trees.behaviour.Behaviour):
    """
    Behavior Tree action node that sends a Nav2 NavigateToPose goal
    to the base station.

    This node mirrors NavigateToZone but targets the base pose instead.
    It uses an asynchronous ActionClient to avoid blocking the BT tick loop.

    Returns:
        RUNNING while navigating,
        SUCCESS when the robot reaches the base,
        FAILURE if navigation fails.
    """

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """
        Initialize the NavigateToBase node.

        Args:
            name (str): Name of the BT node.
            zone_manager (ZoneManager): Shared zone manager instance.
        """
        super().__init__(name)
        self._zone_manager = zone_manager  # Reference to the shared ZoneManager for accessing the base station pose

        self._node: Optional[Node] = None  # ROS node will be injected in setup()
        self._client: Optional[ActionClient] = (
            None  # Nav2 ActionClient will be created in setup()
        )

        self._goal_future = None  # Future for the goal request
        self._result_future = None  # Future for the result of the goal
        self._goal_handle = None  # Handle for the active goal

        self._done: bool = (
            False  # Whether navigation has completed (success or failure)
        )
        self._success: bool = False  # Whether navigation succeeded (reached base)
        self._retry_count = 0

    def setup(self, **kwargs) -> None:
        """
        Setup ROS 2 resources (called once before ticking begins).

        Creates the Nav2 ActionClient.
        """
        self._node = kwargs[
            "node"
        ]  # Get the ROS node from the setup kwargs (injected by the BT framework)

        self._client = (
            ActionClient(  # Create the ActionClient for NavigateToPose using the node
                self._node, NavigateToPose, "navigate_to_pose"
            )
        )

        self._node.get_logger().info("NavigateToBase node setup complete.")

    def initialise(self) -> None:
        """
        Called when the node transitions from idle to active.

        Sends a navigation goal to the base station.
        """
        self._done = False  # Reset done and success flags for this new activation
        self._success = False  # Reset success flag
        self._goal_future = (
            None  # Reset goal and result futures and handle for this new activation
        )
        self._result_future = None  # Reset result future
        self._goal_handle = None  # Reset goal handle

        base = (
            self._zone_manager.base_pose()
        )  # Get the base station pose from the zone manager

        self._node.get_logger().info("Navigating to base station...")

        if not self._client.wait_for_server(
            timeout_sec=2.0
        ):  # Wait for Nav2 action server (non-blocking safe here, only runs once on initialise)
            self._node.get_logger().error("NavigateToPose action server not available!")
            self._done = (
                True  # Mark as done to prevent further attempts, set success to False
            )
            self._success = False  # Set success to False since we can't navigate without the action server
            return

        goal_msg = (
            NavigateToPose.Goal()
        )  # Create the goal message and populate it with the base station pose
        goal_msg.pose = self._build_pose_stamped(  # Helper function to convert base station coordinates to PoseStamped
            base["x"], base["y"], base["yaw"]
        )

        self._goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=None
        )

    def update(self) -> Status:
        """
        Called on every BT tick.

        Returns:
            Status: RUNNING, SUCCESS, or FAILURE.
        """
        if self._goal_future is None:  # If goal hasn't been sent yet
            return Status.RUNNING

        if (
            self._goal_handle is None
        ):  # Check if the goal has been sent and accepted but we don't have a handle yet
            if not self._goal_future.done():
                return Status.RUNNING

            self._goal_handle = (
                self._goal_future.result()
            )  # Get the goal handle from the future result

            if not self._goal_handle.accepted:  # Check if the goal was accepted by Nav2, if not log error and mark as done with failure
                self._node.get_logger().error("Base navigation goal rejected!")
                self._done = True  # Mark as done to prevent further attempts, set success to False
                self._success = (
                    False  # Set success to False since the goal was rejected
                )
                return Status.FAILURE

            self._result_future = (
                self._goal_handle.get_result_async()
            )  # Request result asynchronously, goal accepted, now wait for the result
            return Status.RUNNING

        if (
            self._result_future is not None
        ):  # Check if we have a result future to wait on (should be set if goal was accepted)
            if not self._result_future.done():
                return Status.RUNNING

            result = self._result_future.result()  # Get the result of the navigation goal and check the status code to determine success or failure
            status = result.status  # Nav2 status code for the navigation result

            if status == GoalStatus.STATUS_SUCCEEDED:  # SUCCESS
                self._node.get_logger().info("Reached base station.")
                self._success = True  # Mark navigation as successful and done
                self._done = True  # Mark as done to prevent further processing
                return Status.SUCCESS
            else:
                self._node.get_logger().error(
                    f"Base navigation failed with status: {status}"
                )
                self._success = False  # Mark navigation as failed and done
                self._done = True  # Mark as done to prevent further processing
                return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        """
        Called when the behavior finishes or is interrupted.

        Cancels the goal if still active.

        Args:
            new_status (Status): The new status.
        """
        if (
            self._goal_handle is not None and not self._done
        ):  # Active goal handle exists
            self._node.get_logger().warn("Cancelling base navigation goal...")
            self._goal_handle.cancel_goal_async()  # Cancel the goal asynchronously to avoid blocking the BT tick loop

    def _build_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        Build a PoseStamped for Nav2.

        Args:
            x (float): X position.
            y (float): Y position.
            yaw (float): Orientation (radians).

        Returns:
            PoseStamped: Goal pose in map frame.
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self._node.get_clock().now().to_msg()

        pose.pose.position.x = x  # Set the position from the base station coordinates
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # Assuming flat ground, z is 0

        qz = math.sin(yaw / 2.0)  # Convert yaw to quaternion (assuming roll=pitch=0)
        qw = math.cos(yaw / 2.0)  # Quaternion for yaw rotation only

        pose.pose.orientation.z = qz  # Set the orientation from the base station yaw
        pose.pose.orientation.w = (
            qw  # Set the orientation w component for the quaternion
        )

        return pose


class DetectSurvivor(py_trees.behaviour.Behaviour):
    """BT action node that calls the detect_survivor service.

    Uses the async-poll pattern to avoid blocking the executor.
    Stores the detection result for use by IsSurvivorDetected and
    BroadcastSurvivorTF via was_found() and survivor_pose().

    """

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
        self._node = kwargs["node"]
        self._client = self._node.create_client(DetectSurvivorSrv, "detect_survivor")

    def initialise(self) -> None:
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
            if not self._client.service_is_ready():  # server running ??
                self._node.get_logger().warn(
                    "detect_survivor service not ready, retrying..."
                )
                return py_trees.common.Status.RUNNING
            else:
                zone_id = self._zone_manager.current_zone()["id"]
                self._node.get_logger().info(
                    f"Calling detect_survivor for {zone_id}..."
                )

                request = DetectSurvivorSrv.Request()
                request.zone_id = zone_id
                self._future = self._client.call_async(request)
                return (
                    py_trees.common.Status.RUNNING
                )  # running and waiting for response

        # 2nd tick : poll until server response and done
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        # final tick : response arrive , then read result
        response = self._future.result()
        self._future = None  # reset storing val

        if response is None:  # sth went wrong return failure
            return py_trees.common.Status.FAILURE

        self._was_found = response.found
        if response.found:
            self._survivor_xy = (response.survivor_x, response.survivor_y)
        else:
            self._survivor_xy = None  # no survivor = None, not (0.0, 0.0)

        if self._survivor_xy:
            sx, sy = self._survivor_xy
            self._node.get_logger().info(
                f"DetectSurvivor: found={self._was_found} at ({sx:.2f}, {sy:.2f})"
            )
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
        Read into stored result here

        Returns:
            Tuple of (x, y) survivor coord.
        """
        return self._survivor_xy

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


class BroadcastSurvivorTF(py_trees.behaviour.Behaviour):
    """BT action node that broadcasts a static TF frame for a survivor."""

    def __init__(
        self, name: str, detect_node: DetectSurvivor, zone_manager: ZoneManager
    ) -> None:
        """Initialize BroadcastSurvivorTF node

        Args:
            name: Display name for the BT node.
            detect_node: Reference to the DetectSurvivor BT node.
            zone_manager: Shared ZoneManager instance.
        """

        super().__init__(name=name)
        self._detect_node = detect_node
        self._zone_manager = zone_manager
        self._node = None
        self._broadcaster = None
        self._last_survivor_id = ""

    def last_survivor_id(self) -> str:
        """Return most recently allocated survivor frame ID.

        Returns:
            The child_frame_id of the last broadcast. Empty string if none yet.
        """
        return self._last_survivor_id

    def setup(self, **kwargs):
        """Create the StaticTransformBroadcaster.

        Args:
            **kwargs: Must contain 'node' (rclpy.node.Node).
        """
        self._node = kwargs["node"]
        self._broadcaster = StaticTransformBroadcaster(kwargs["node"])

    def initialise(self):
        """Nothing to reset here, this node reads and broadcasts instantly."""
        pass

    def update(self) -> py_trees.common.Status:
        """Broadcast the survivor TF frame and return SUCCESS.

        Reads survivor pose from detect_node, generates a survivor ID,
        broadcasts the static transform, and stores the ID back onto
        detect_node for NotifyBase to read.

        Returns:
            SUCCESS after broadcasting.
        """

        pose = self._detect_node.survivor_pose()
        if pose is None:  # faliure if no surivor pose available
            return py_trees.common.Status.FAILURE

        sx, sy = pose
        survivor_id = self._zone_manager.next_survivor_id()  # get survivor id

        self._detect_node.set_survivor_id(
            survivor_id
        )  # store id so notifybase can read

        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = "map"  # parent frame
        t.child_frame_id = survivor_id  # new frame name
        t.transform.translation.x = sx  # survivor x
        t.transform.translation.y = sy  # survivor y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # identity rotation

        self._broadcaster.sendTransform(t)
        self._last_survivor_id = survivor_id
        self._node.get_logger().info("Survivor detected at zone!")
        self._node.get_logger().info(
            f"Broadcasting TF frame: {survivor_id} at ({sx:.2f}, {sy:.2f}) in map frame."
        )

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """No cleanup needed for this node."""
        pass


class NotifyBase(py_trees.behaviour.Behaviour):
    """Uses the async-poll pattern to avoid blocking the executor.
    Reads survivor ID and pose from detect_node and reports to the
    simulated command center via the report_survivor service.

    Args:
        name: Display name for the BT node.
        detect_node: Reference to the DetectSurvivor BT node.
    """

    def __init__(self, name: str, detect_node: DetectSurvivor) -> None:
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

    def setup(self, **kwargs) -> None:
        """Create the report_survivor service client.

        Args:
            **kwargs: Must contain 'node' (rclpy.node.Node).
        """
        # Called once before the first tick.
        # Acquire ROS 2 resources here (node, publishers, etc.)
        self._node = kwargs["node"]
        self._client = self._node.create_client(ReportSurvivorSrv, "report_survivor")
        pass

    def initialise(self) -> None:
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
            if not self._client.service_is_ready():  # server running ??
                self._node.get_logger().warn(
                    "report_survivor service not ready, retrying..."
                )
                return py_trees.common.Status.RUNNING

            pose = self._detect_node.survivor_pose()
            survivor_id = self._detect_node.survivor_id()

            if pose is None or not survivor_id:
                return py_trees.common.Status.FAILURE

            x, y = pose
            location = PointStamped()
            location.header.frame_id = "map"  # coordinates are in map frame
            location.header.stamp = self._node.get_clock().now().to_msg()

            location.point.x = x
            location.point.y = y
            location.point.z = 0.0

            request = ReportSurvivorSrv.Request()
            request.survivor_id = survivor_id
            request.location = location

            self._future = self._client.call_async(request)
            self._node.get_logger().info(f"Reporting {survivor_id} to base...")
            return py_trees.common.Status.RUNNING  # running and waiting for response

        # 2nd tick : poll until server response and done
        if not self._future.done():
            return py_trees.common.Status.RUNNING

        # final tick : response arrive , then read result
        response = self._future.result()
        self._future = None  # reset storing val

        if response is None:  # sth went wrong return failure
            return py_trees.common.Status.FAILURE

        if not response.acknowledged:  # server said "not acknowledged"
            return py_trees.common.Status.FAILURE

        self._node.get_logger().info(
            f"Base acknowledged {self._detect_node.survivor_id()}."
        )

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """No cleanup needed for this node."""
        pass


class AdvanceZone(py_trees.behaviour.Behaviour):
    """
    Behavior Tree action node that advances to the next search zone.

    This node interacts with the shared ZoneManager to move the mission
    to the next zone in the patrol sequence. It always returns SUCCESS
    immediately after advancing.

    Attributes:
        _zone_manager (ZoneManager): Shared mission state manager.
        _node (Node): ROS 2 node (injected via setup).
    """

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """
        Initialize the AdvanceZone node.

        Args:
            name (str): Name of the behavior tree node.
            zone_manager (ZoneManager): Shared zone manager instance.
        """
        super().__init__(name)
        self._zone_manager = zone_manager  # Reference to the shared ZoneManager for advancing to the next zone
        self._node: Optional[Node] = None

    def setup(self, **kwargs) -> None:
        """
        Setup ROS 2 resources.

        Stores the node handle for logging.
        """
        self._node = kwargs["node"]

    def update(self) -> Status:
        """
        Advance to the next zone.

        Returns:
            Status.SUCCESS: Always succeeds after advancing.
        """
        current_zone = (
            self._zone_manager.current_zone()
        )  # Get the current zone info for logging before advancing

        self._node.get_logger().info(  # Log the zone we just finished before advancing to the next one
            f"Finished {current_zone['id']}. Advancing to next zone."
        )

        self._zone_manager.advance()  # Advance to the next zone in the ZoneManager's patrol sequence

        return Status.SUCCESS


class LogNoDetection(py_trees.behaviour.Behaviour):
    """
    Behavior Tree action node that logs when no survivor is found
    in the current search zone.

    This node is executed as the fallback branch of the HandleDetection
    selector when IsSurvivorDetected returns FAILURE. It simply logs
    the result and returns SUCCESS so the patrol sequence can continue.

    Attributes:
        _zone_manager (ZoneManager): Shared mission state manager.
        _node (Node): ROS 2 node (injected via setup).
    """

    def __init__(self, name: str, zone_manager: ZoneManager) -> None:
        """
        Initialize the LogNoDetection node.

        Args:
            name (str): Name of the behavior tree node.
            zone_manager (ZoneManager): Shared zone manager instance.
        """
        super().__init__(name)
        self._zone_manager = zone_manager  # Reference to the shared ZoneManager for accessing the current zone information for logging
        self._node: Optional[Node] = None

    def setup(self, **kwargs) -> None:
        """
        Setup ROS 2 resources.

        Stores the node handle for logging.
        """
        self._node = kwargs["node"]

    def update(self) -> Status:
        """
        Log that no survivor was found in the current zone.

        Returns:
            Status.SUCCESS: Always succeeds after logging.
        """
        zone = (
            self._zone_manager.current_zone()
        )  # Get the current zone info for logging

        self._node.get_logger().info(f"No survivor found at {zone['id']}.")

        return Status.SUCCESS
