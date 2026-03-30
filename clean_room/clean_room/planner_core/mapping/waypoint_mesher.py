###########################################################
# ----- PGM Map File Processor ------
###########################################################
''' 
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
#=======================================
# --- Class ---
#=======================================
class WaypointMesh:
    
    """ This class takes 2D occupancy grid information and generates a list of waypoints in the free spaces within at a specified resolution. """
    
    def __init__(self, resolution=10):
        self.resolution = resolution
        """
        resolution = spacing between waypoints in pixels
        """
        
    def generate_mesh(self, room_array, free_mask):
        """ This function generates a 2D mesh of waypoints using the occupancy array and a boolean mask of all free spaces within.
        
        Args:
            room_array: 2D numpy array of the room
            free_mask: boolean mask of free pixels
        
        Returns:
            waypoints: List of waypoint coordinates
        """
        
        h, w = room_array.shape
        waypoints = []

        for y in range(0, h, self.resolution):
            for x in range(0, w, self.resolution):
                if free_mask[y, x]:
                    waypoints.append((x, y))

        return waypoints