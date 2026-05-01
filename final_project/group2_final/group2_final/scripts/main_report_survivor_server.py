# Name: Yossaphat Kulvatunyou & Nam Facchetti
# Module: report_survivor_server.py


import rclpy
from group2_final.service_servers.report_survivor_server import ReportSurvivorServer 

def main(args=None) -> None:
    """Entry point for the report_survivor_server_exe executable."""
    rclpy.init(args=args)
    node = ReportSurvivorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()