''' 
###########################################################
Room Cleaning Path Planner Wrapper Class
###########################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
#=======================================
# --- Imports ---
#=======================================
from clean_room.planner_core.mapping.waypoint_mesher import WaypointMesh
from clean_room.planner_core.planning.tsp_solver_greedy import GreedyTSP
import numpy as np

#=======================================
# --- Class ---
#=======================================
class RoomPathPlanner:
    def __init__(self, map_processor, corners, mesh_resolution=10, inflation_radius=5):
        """
        Args:
            map_processor: MapProcessor instance
            room_bounds: (x_min, x_max, y_min, y_max)
            mesh_resolution: waypoint spacing in pixels
            inflation_radius: Radius that obstacles will be inflated by in pixels
        """
        self.mp = map_processor                 
        self.corners = corners 
        self.mesh_resolution = mesh_resolution
        self.inflation_radius = inflation_radius

        # Initialize variables for instance
        self.room = None
        self.free_mask = None
        self.waypoints = None
        self.tsp_order = None


    def extract_room(self):
        '''
        This function calls on MapProcessor class room extraction function to isolate room of interest from full map file based on x and y pixel coordinate maximums and minimums.
        '''
        # Full map
        full_map = self.mp.map

        # Create polygon mask
        mask = self.mp.polygon_mask(self.corners, full_map.shape)

        # Extract only pixels inside polygon
        self.room = np.where(mask, full_map, self.mp.occupied_val)

        return self.room


    def compute_free_mask(self):
        ''' This function creates a MapProcessor class instance and extracts all free area in inputted map array, it then removes a buffer region around obstacles from free mask. '''
        if self.room is None:   # If user forgets to create room map instance this forces it to be extracted
            self.extract_room()
            
        free = self.mp.free_space_mask(self.room)                               # Create a mask of all free spaces
        inflated = self.mp.inflate_obstacles(self.room, self.inflation_radius)  # Create a mask of inflated obstacles 
        self.free_mask = np.logical_and(free, ~inflated)                        # Remove intersecting cells from the free mask and the inflated mask
        
        return self.free_mask
    
    
    def generate_waypoints(self):
        ''' This function creates a WaypointMesh class instance and generates the mesh of waypoints for the free space in the room. '''
        if self.free_mask is None:      # If user forgets to isolate free areas in map instance this will ensure data is clean
            self.compute_free_mask()
        
        print("Creating mesh...")    
        mesher = WaypointMesh(self.mesh_resolution)                         # Create a meshing instance with given resolution
        self.waypoints = mesher.generate_mesh(self.room, self.free_mask)    # Call on meshing instance to generate waypoints
        print("Meshing complete.")    
        
        return self.waypoints


    def compute_tsp_path(self, start_index=0):
        """ This function solves for the optimal path through all waypoints using a TSP Greedy solver instance, the output is a list of waypoints in the order of the path. """
        if self.waypoints is None:      # If user forgets to generate waypoints this will ensure they are created
            self.generate_waypoints()
            
        tsp = GreedyTSP(self.waypoints)                         # Create a greedy solver instance
        print("Starting pathing...")
        self.tsp_order = tsp.solve_two_opt(start_index=start_index)     # Call on solver instance to solve optimal path
        print("Pathing complete.")
        return self.tsp_order


    def compute_full_path(self, start_index=0):
        """ This function calls on the tsp path solver function and iterates through the stored waypoints to create a list of waypoint coordinates in the order of the path. """
        "Returns ordered waypoint coordinates"
        order = self.compute_tsp_path(start_index)
        return [self.waypoints[i] for i in order]
    
    
    


