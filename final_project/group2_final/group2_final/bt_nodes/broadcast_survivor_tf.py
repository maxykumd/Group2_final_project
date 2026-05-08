# Name: Yossaphat Kulvatunyou & Nam Facchetti
# Module: broadcast_survivor_tf.py - BT action node that broadcasts a static TF frame for a survivor.


import py_trees
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from group2_final.bt_nodes.detect_survivor import DetectSurvivor
from group2_final.zone_manager import ZoneManager

class BroadcastSurvivorTF(py_trees.behaviour.Behaviour):
    """BT action node that broadcasts a static TF frame for a survivor.
    
    """

    def __init__(self, name:str, detect_node: DetectSurvivor, zone_manager: ZoneManager) -> None:
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
        self._node = kwargs['node']
        self._broadcaster = StaticTransformBroadcaster(kwargs['node'])


    def initialise(self):
        """Nothing to reset — this node reads and broadcasts instantly."""
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
        if pose is None: # faliure if no surivor pose available
            return py_trees.common.Status.FAILURE

        sx, sy = pose
        survivor_id = self._zone_manager.next_survivor_id()# get survivor id

        self._detect_node.set_survivor_id(survivor_id) # store id so notifybase can read

        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = "map"  # parent frame
        t.child_frame_id = survivor_id # new frame name
        t.transform.translation.x = sx  # survivor x
        t.transform.translation.y = sy  # survivor y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0  # identity rotation
        
        self._broadcaster.sendTransform(t)
        self._last_survivor_id = survivor_id
        self._node.get_logger().info(f"BroadcastSurvivorTF: published '{survivor_id}' at ({sx:.3f}, {sy:.3f})")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status: py_trees.common.Status) -> None:
        """No cleanup needed for this node."""
        pass
