"""Simulated survivor detection service server."""

from rclpy.node import Node
from group2_final_interfaces.srv import DetectSurvivor
import rclpy

# zones A and C have survivors
SURVIVORS: dict[str, tuple[float, float]] = {
    "zone_a": (-2.5, 3.2),
    "zone_c": (4.1, -2.5),
}


class DetectSurvivorServer(Node):
    """ROS 2 node that simulates a survivor-detection service.

    Maintains a hardcoded dictionary mapping zone IDs to survivor
    coordinates. Returns found=True with coordinates if the zone has
    a survivor, found=False otherwise.
    """

    def __init__(self) -> None:
        super().__init__("detect_survivor_server")

        self._srv = self.create_service(
            DetectSurvivor, "detect_survivor", self._handle_request
        )
        self.get_logger().info("DetectSurvivor service ready.")

    def _handle_request(
        self,
        request: DetectSurvivor.Request,
        response: DetectSurvivor.Response,
    ) -> DetectSurvivor.Response:
        """Handle a DetectSurvivor service request.

        Args:
            request: contains the zone_id string to look up.
            response: populated with found status and coordinates.

        Returns:
            The populated response.
        """

        zone_id = request.zone_id

        if zone_id in SURVIVORS:
            x, y = SURVIVORS[zone_id]
            response.found = True
            response.survivor_x = x
            response.survivor_y = y
            self.get_logger().info(
                f"Detection requested for {zone_id}: FOUND at ({x:.2f}, {y:.2f})"
            )
        else:
            response.found = False
            response.survivor_x = 0.0
            response.survivor_y = 0.0
            self.get_logger().info(f"Detection requested for {zone_id}: NOT FOUND")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectSurvivorServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
