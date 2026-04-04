''' 
##################################################################
# ----- Room Cleaning Planner Map Related Utility Functions -----
##################################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
# ======================================================
# ----- Imports -----
# ======================================================
import yaml
import os

# ======================================================
# ----- Class -----
# ======================================================
class MapUtils:
    def __init__(self, map_yaml_path):
        with open(map_yaml_path, 'r') as f:
            self.metadata = yaml.safe_load(f)

        self.resolution = self.metadata['resolution']
        self.origin_x, self.origin_y, _ = self.metadata['origin']
        self.map_height = 210

        # Map image path
        self.map_image_path = os.path.join(
            os.path.dirname(map_yaml_path),
            self.metadata['image']
        )
    
    # -------------------------------------------------------------------
    # --- World coordinates to pixel coordinates conversion ---
    # -------------------------------------------------------------------
    def world_to_pixel(self, x, y):
        """ This function converts world coordinates (x,y) to pixel coordinates (px,py)."""
        px = int((x - self.origin_x) / self.resolution)
        
        py_flipped = int((y - self.origin_y) / self.resolution)
        py = int(self.map_height - py_flipped)
        return px, py
    
    # -------------------------------------------------------------------
    # --- Pixel coordinates to world coordinates conversion ---
    # -------------------------------------------------------------------
    def pixel_to_world(self, px, py):
        """ This function converts pixel coordinates (px,py) to world coordinates (x,y)."""
        py_flip = self.map_height - py
        x = (px + 0.5) * self.resolution + self.origin_x
        y = (py_flip + 0.5) * self.resolution + self.origin_y
        return x, y  