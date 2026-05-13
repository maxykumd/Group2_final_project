# Name: Nam Facchetti & Yossaphat Kulvatunyou
# Module: zone_manager.py - Manages the mission state for the search-and-rescue task, including zone tracking and survivor ID generation.

from typing import List, Dict


class ZoneManager:
    """
    Manages the mission state for the search-and-rescue task.

    This class keeps track of:
    - The ordered list of search zones
    - The current zone index
    - The base station pose
    - Unique survivor IDs for detected survivors

    It is shared across all Behavior Tree nodes via constructor injection,
    acting as the single source of truth for mission progression.
    """

    def __init__(self, zones: List[Dict], base_station: Dict) -> None:
        """
        Initialize the ZoneManager with zones and base station pose.

        Args:
            zones (List[Dict]): List of zone dictionaries, each containing:
                {
                    "id": str,
                    "x": float,
                    "y": float,
                    "yaw": float
                }
            base_station (Dict): Dictionary containing base pose:
                {
                    "x": float,
                    "y": float,
                    "yaw": float
                }
        """
        if not zones:  # Ensure that there is at least one zone to manage
            raise ValueError("ZoneManager requires at least one zone.")

        self._zones: List[Dict] = (
            zones  # List of zones to be searched, each with an ID and pose information
        )
        self._base_station: Dict = (
            base_station  # Pose of the base station for return after mission completion
        )

        self._current_index: int = (
            0  # Index of the current active zone in the zones list
        )

        self._survivor_count: int = 0  # Counter for generating unique survivor IDs

    def current_zone(self) -> Dict:
        """
        Get the current active zone.

        Returns:
            Dict: The current zone dictionary with keys:
                "id", "x", "y", "yaw"
        """
        if (
            not self.has_remaining()
        ):  # Guard against accessing a zone when there are no remaining zones
            raise IndexError("No remaining zones. Cannot access current zone.")

        return self._zones[
            self._current_index
        ]  # Return the current zone based on the internal index

    def has_remaining(self) -> bool:
        """
        Check if there are unvisited zones remaining.

        Returns:
            bool: True if there are remaining zones, False otherwise.
        """
        return self._current_index < len(self._zones)

    def advance(self) -> None:
        """
        Advance to the next zone in the list.

        This method increments the internal index. It does not return anything.

        Note:
            The Behavior Tree logic ensures this is only called when valid,
            so no explicit bounds check is required here.
        """
        self._current_index += 1

    def base_pose(self) -> Dict:
        """
        Get the base station pose.

        Returns:
            Dict: Base station pose with keys:
                "x", "y", "yaw"
        """
        return self._base_station

    def next_survivor_id(self) -> str:
        """
        Generate a unique survivor ID.

        Returns:
            str: A unique identifier in the format "survivor_N"
        """
        self._survivor_count += (
            1  # Increment the survivor count for each detected survivor
        )
        return f"survivor_{self._survivor_count}"

    def total_zones(self) -> int:
        """
        Get the total number of zones.

        Returns:
            int: Total number of zones.
        """
        return len(self._zones)

    def current_index(self) -> int:
        """
        Get the current zone index (0-based).

        Returns:
            int: Current zone index.
        """
        return self._current_index


if __name__ == "__main__":
    zones = [
        {
            "id": "zone_a",
            "x": 0,
            "y": 0,
            "yaw": 0,
        },  # Define the first zone with its ID and pose
        {
            "id": "zone_b",
            "x": 1,
            "y": 1,
            "yaw": 0,
        },  # Define the second zone with its ID and pose
    ]
    base = {"x": 0, "y": 0, "yaw": 0}  # Define the base station pose

    zm = ZoneManager(
        zones, base
    )  # Create an instance of ZoneManager with the defined zones and base station

    while (
        zm.has_remaining()
    ):  # Loop through the zones as long as there are unvisited zones remaining
        print(zm.current_zone())
        zm.advance()  # Advance to the next zone after printing the current zone information

    print(zm.next_survivor_id())  # Generate and print the survivor ID
    print(zm.next_survivor_id())
