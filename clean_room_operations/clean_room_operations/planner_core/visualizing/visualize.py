###########################################################
# ----- Visualization Functions for Room Cleaning Planner ------
###########################################################
''' 
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''

#=======================================
# --- Imports ---
#=======================================
import matplotlib.pyplot as plt
import numpy as np


#=======================================
# --- Functions ---
#=======================================

# --------------------------------------
# Room and path visualization
# --------------------------------------
def visualize_room_and_path(room_array, waypoints, tsp_path):
    """
    This function creates a visualization of the room with waypoints and the solved path overlayed.
    
    Args:
        room_array: 2D numpy array of the room
        waypoints: list of (x, y) tuples
        tsp_path: ordered list of (x, y) tuples
    """

    plt.figure(figsize=(10, 10))

    # Show the room map
    plt.imshow(room_array, cmap='gray', origin='upper')

    # Plot waypoints
    if waypoints:
        xs, ys = zip(*waypoints)
        plt.scatter(xs, ys, s=10, c='blue', label='Waypoints')

    # Plot TSP path
    if tsp_path:
        px, py = zip(*tsp_path)
        plt.plot(px, py, c='lime', linewidth=2, label='TSP Path')
        plt.scatter(px[0], py[0], c='red', s=50, label='Start')

    plt.title("Room Map with Waypoints and TSP Path")
    plt.legend()
    plt.gca().invert_yaxis()  # Match image coordinate system
    plt.show()
    
    
# --------------------------------------
# Inflated Room Visualization
# --------------------------------------    
def visualize_inflation(room_array, inflated_mask):
    """ This function visualizes the room and overlays the inflated obstacles."""
    plt.figure(figsize=(8, 8))
    plt.imshow(room_array, cmap='gray')
    plt.imshow(inflated_mask, cmap='Reds', alpha=0.4)
    plt.title("Obstacle Inflation Overlay")
    plt.show()    