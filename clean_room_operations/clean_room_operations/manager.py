''' 
#######################################################
Room Cleaning Manager
#######################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
# ======================================================
# --- Imports ---
# ======================================================
# Standard imports
import yaml

# Custom imports
from clean_room_operations.planner_core.mapping.map_processor import MapProcessor
from clean_room_operations.planner_core.planning.room_cleaning_path_planner import RoomPathPlanner
from clean_room_operations.planner_core.visualizing.rviz_markers import RVizMarkers

from clean_room_operations.utils.map_utils import MapUtils

# ======================================================
# --- Class ---
# ======================================================
class CleaningManager:
    """This class manages cleaning operations and logic for room cleaning."""
    def __init__(self, rooms_config_path, map_yaml_path):
        # Load room config
        with open(rooms_config_path, 'r') as f:
            data = yaml.safe_load(f)
        self.room_config = data["rooms"]

        # Load map utilities
        self.map_utils = MapUtils(map_yaml_path)

        # Load map image
        self.map_processor = MapProcessor(self.map_utils.map_image_path)


    def cleaning_path(self, room_name, current_pose):
        """Return ordered waypoints for cleaning the given room."""

        params = self.room_config[room_name]
        corners = params["corners"]
        mesh_res = params["mesh_resolution"]
        inflation = params["inflation_radius"]

        # Create planner
        planner = RoomPathPlanner(
            map_processor=self.map_processor,
            corners=corners,
            mesh_resolution=mesh_res,
            inflation_radius=inflation
        )

        # Generate waypoints
        waypoints = planner.generate_waypoints()

        # Determine closest waypoint to robot
        start_idx = self._find_closest_waypoint(waypoints, current_pose)

        # Compute optimal path
        ordered_waypoints = planner.compute_full_path(start_index=start_idx)

        return ordered_waypoints
    

    def _find_closest_waypoint(self, waypoints, pose):
        """Find the closest waypoint to the robot's current pose."""
        if pose is None:
            return 0

        px = pose.position.x
        py = pose.position.y

        dists = [(i, (wx - px)**2 + (wy - py)**2)
                 for i, (wx, wy) in enumerate(waypoints)]

        return min(dists, key=lambda x: x[1])[0]