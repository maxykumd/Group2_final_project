# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: main_search_and_rescue.py - Entry point: assemble and run the BT.

import rclpy
import py_trees
import py_trees_ros
import time

from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

from group2_final.zone_manager import ZoneManager
from group2_final.bt_nodes.conditions import ZonesRemaining, IsSurvivorDetected
from group2_final.bt_nodes.actions import ( NavigateToZone, NavigateToBase, DetectSurvivor, BroadcastSurvivorTF, NotifyBase, AdvanceZone, LogNoDetection,)


def build_tree(zone_manager: ZoneManager) -> py_trees.trees.BehaviourTree:
    """Assemble and combine all node to get the full search-and-rescue behaviour tree.

    Args:
        zone_manager: Shared ZoneManager instance for all the nodes.

    Returns:
        The assembled BehaviourTree (not yet ticking).
    """
    # step1 : Create Leaf node 
    zones_remaining   = ZonesRemaining("ZonesRemaining?", zone_manager)
    navigate_to_zone  = NavigateToZone("NavigateToZone", zone_manager)
    detect_survivor   = DetectSurvivor("DetectSurvivor", zone_manager)
    is_detected       = IsSurvivorDetected("IsSurvivorDetected?", detect_survivor)
    broadcast_tf      = BroadcastSurvivorTF("BroadcastSurvivorTF", detect_survivor, zone_manager)
    notify_base       = NotifyBase("NotifyBase", detect_survivor)
    log_no_detection  = LogNoDetection("LogNoDetection", zone_manager)
    advance_zone      = AdvanceZone("AdvanceZone", zone_manager)
    navigate_to_base  = NavigateToBase("NavigateToBase", zone_manager)

    
    # SurvivorFound Sequence : is_detected -> broadcast_tf -> notify_base 
    survivor_found_seq = py_trees.composites.Sequence(
        name="SurvivorFound",
        memory=True,
        children=[is_detected, broadcast_tf, notify_base], # check in order
    )


    # HandleDetection Selector : SurvivorFound -> LogNodDetection
    handle_detection = py_trees.composites.Selector(
        name="HandleDetection",
        memory=False,
        children=[survivor_found_seq, log_no_detection],
    )

    # Patrol Sequence : ZoneRemaining -> NavigateToZone -> DetectSurvivor -> HandleDetection -> AdvanceZone
    patrol_seq = py_trees.composites.Sequence(
        name="Patrol",
        memory=True,
        children=[zones_remaining, navigate_to_zone, detect_survivor,
                  handle_detection, advance_zone],
    )


    # NavigateToBase OneShot
    navigate_to_base_once = py_trees.decorators.OneShot(
        child=navigate_to_base,
        name="NavigateToBase",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )

    # Root Sequence — patrol until done, THEN go home
    root = py_trees.composites.Selector(
        name="Mission",
        memory=False,
        children=[patrol_seq, navigate_to_base_once],
    )

    return root, navigate_to_base  


def main(args: list[str] | None = None) -> None:
    """ROS 2 entry point: load parameters, build tree, spin.

    Args:
        args: Forwarded to rclpy.init (defaults to sys.argv).
    """
    rclpy.init(args=args)

    # Temporary node to read parameteres
    read_param_node = Node(
        "search_and_rescue",
        automatically_declare_parameters_from_overrides=True,
        allow_undeclared_parameters=True,
    )

    # Read tick rate
    if not read_param_node.has_parameter("tick_rate_hz"):
        read_param_node.declare_parameter("tick_rate_hz", 2.0)
    tick_rate_hz = read_param_node.get_parameter("tick_rate_hz").value
    period_ms = int(1000.0 / tick_rate_hz)

    # Read zone order
    zone_order = (read_param_node.get_parameter("zone_order").get_parameter_value().string_array_value)


    # Build zone list from yaml file
    zones = []
    for zone_id in zone_order:
        zones.append({
            "id":  zone_id,
            "x":   read_param_node.get_parameter(f"zones.{zone_id}.x").value,
            "y":   read_param_node.get_parameter(f"zones.{zone_id}.y").value,
            "yaw": read_param_node.get_parameter(f"zones.{zone_id}.yaw").value,
        })

    # Read base pose
    base_station = {
        "x":   read_param_node.get_parameter("base_station.x").value,
        "y":   read_param_node.get_parameter("base_station.y").value,
        "yaw": read_param_node.get_parameter("base_station.yaw").value,
    }

    read_param_node.get_logger().info(f"Loaded {len(zones)} search zones from parameters.")
    read_param_node.get_logger().info(
        f"Base station at ({base_station['x']:.2f}, "
        f"{base_station['y']:.2f}, yaw={base_station['yaw']:.2f})." 
    )

    
    # Build ZoneManager
    zone_manager = ZoneManager(zones=zones, base_station=base_station)

    # feed AMCL with spawn position with BasicNavigator
    navigator = BasicNavigator(node_name="basic_navigator")
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.w = 1.0  # yaw = 0
    initial_pose.pose.orientation.z = 0.0
    navigator.setInitialPose(initial_pose)
    time.sleep(3.0)
    navigator.waitUntilNav2Active()
    read_param_node.get_logger().info("Nav2 is active.")
    read_param_node.destroy_node()

    # Assemble tree 
    root, navigate_to_base = build_tree(zone_manager)

    # Wrap in py_trees_ros BehaviourTree 
    tree = py_trees_ros.trees.BehaviourTree(root=root,unicode_tree_debug=False,)

    try:
        tree.setup(node_name="search_and_rescue", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError:
        read_param_node.get_logger().error("BT setup timed out.")
        rclpy.shutdown()
        return

    read_param_node.get_logger().info(f"Nav2 is active. Starting BT at {tick_rate_hz} Hz.")

    # Tick the tree 
    tree.tick_tock(period_ms=period_ms)

    try:
        while rclpy.ok():
            rclpy.spin_once(tree.node, timeout_sec=0.5)
            if (navigate_to_base.status == py_trees.common.Status.SUCCESS
                    and not zone_manager.has_remaining()):
                tree.node.get_logger().info("Mission complete.")
                break
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()