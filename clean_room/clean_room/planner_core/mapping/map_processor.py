'''
###########################################################
# ----- PGM Map File Processor ------
###########################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
#=======================================
# --- Imports ---
#=======================================
import numpy as np
import imageio.v2 as imageio
from scipy.ndimage import binary_dilation
from matplotlib.path import Path

#=======================================
# --- Class ---
#=======================================
class MapProcessor:
    """ This class extracts and processes occupancy grid information from .pgm files into numpy arrays. """
    def __init__(self, pgm_path):
        self.map = imageio.imread(pgm_path).astype(np.uint8)
        self.free_val = 250
        self.unknown_val = 205
        self.occupied_val = 0

    def extract_room(self, x_min, x_max, y_min, y_max):
        """ This FUNCTION extracts a rectangular sub portion of the map file through inputted x and y coordinate minimum and maximums.
        
        Args:
            x_min (integer) : Minimum x-coordinate value (pixels)
            x_max (integer) : Maximum x-coordinate value (pixels)
            y_min (integer) : Minimum y-coordinate value (pixels)
            y_max (integer) : Maximum y-coordinate value (pixels)
        
        Returns:
            map: Updated map variable of current class instance containing only the region of interest.
        """
        return self.map[y_min:y_max, x_min:x_max]

    def free_space_mask(self, room_array):
        """ This function creates a boolean mask of all free space within an occupancy array."""
        return room_array >= self.free_val
    
    def obstacle_mask(self, room_array):
        """ This function creates a boolean mask of all occupied space within an occupancy array."""
        return room_array == self.occupied_val
    
    def inflate_obstacles(self, room_array, inflation_radius=5):
        """ This function inflates the found occupied cells in the occupancy array by an inflation radius value."""
        
        obstacles = self.obstacle_mask(room_array)                          # Create boolean mask of occupied cells
        struct = np.ones((inflation_radius*2+1,inflation_radius*2+1))       # Create structure of the inflation
        inflated = binary_dilation(obstacles, structure=struct)             # Inflate obstacle mask using the created structure
        
        return inflated
    
    def polygon_mask(self, polygon, shape):
        """ This function returns a boolean mask of the defined polygon."""

        poly = Path(polygon)

        # Create grid of pixel coordinates
        y, x = np.mgrid[:shape[0], :shape[1]]
        coords = np.vstack((x.flatten(), y.flatten())).T

        mask = poly.contains_points(coords)
        return mask.reshape(shape)