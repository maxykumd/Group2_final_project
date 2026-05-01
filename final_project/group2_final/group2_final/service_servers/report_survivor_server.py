"""Simulated command-center report service server."""

from rclpy.node import Node
from group2_final_interfaces.srv import ReportSurvivor


class ReportSurvivorServer(Node):
    """ROS 2 node that simulates a command-center report service.

    Receives survivor reports from the behavior tree and logs them.
    Does not forward messages anywhere — the 'report' is purely a
    log entry. Rejects (warns) if coordinates are not in the map frame.
    """

    def __init__(self) -> None:
        """Initialize the ReportSurvivorServer node."""
        super().__init__("report_survivor_server")

        self._srv = self.create_service(
            ReportSurvivor,
            "report_survivor",
            self._handle_request
        )
        self.get_logger().info("ReportSurvivorServer ready.")
    
    
    def _handle_request(self,request: ReportSurvivor.Request, response: ReportSurvivor.Response) -> ReportSurvivor.Response:
        """Handle a ReportSurvivor service request.

        Logs the survivor_id, frame_id, and coordinates. Warns if
        the frame_id is not 'map'.

        Args:
            request: Contains survivor_id and PointStamped location.
            response: Populated with acknowledged=True.

        Returns:
            The populated response.
        """

        survivor_id = request.survivor_id
        frame = request.location.header.frame_id
        x = request.location.point.x
        y = request.location.point.y

        if frame != "map":
            self.get_logger().warn(
                f"Expected frame 'map' but received '{frame}'. ")
        else:
            self.get_logger().info(
                f"Report received: survivor_{survivor_id} at (-x={x:.3f}, y={y:.3f}). Acknowledged."
)

        response.acknowledged = True
        return response
