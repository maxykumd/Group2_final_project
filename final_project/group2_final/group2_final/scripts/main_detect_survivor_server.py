# Name: Yossaphat Kulvatunyou & Nam Facchetti
# Module: main_detect_survivor_server.py


import rclpy
from group2_final.service_servers.detect_survivor_server import DetectSurvivorServer 

def main(args=None) -> None:
    """Entry point for the detect_survivor_server_exe executable."""
    rclpy.init(args=args)
    node = DetectSurvivorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()