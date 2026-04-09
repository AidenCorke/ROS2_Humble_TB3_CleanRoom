''' 
###########################################################
TSP Greedy Solver
###########################################################
Author      : Aiden Corke [101358475]
Course      : MCG5138 - Mobile Robotics
Professor   : Amirhossein Monjazeb 
'''
#=======================================
# --- Imports ---
#=======================================
from scipy.spatial.distance import cdist
import numpy as np

#=======================================
# --- Class ---
#=======================================
class GreedyTSP:   
    def __init__(self, points):
        self.points = np.array(points)
        self.n = len(points)
        self.dist_matrix = cdist(self.points, self.points)  # Generate distance between each point


    def solve_greedy(self, start_index=0):
        ''' 
        This function solves the optimal path through a set of waypoints using a Greedy algorithm. 
        
        Each waypoint is assigned a cost value based on distance and as the solver reaches each node it will choose the cheapest option available to it.
        
        Args:
            start_index (integer): Start point, defaults to the first waypoint (0).
            
        Returns:
            visited (List): List of waypoints in order of travel.     
        '''
        visited = [start_index]                         # List of visited waypoints (order is the path)
        remaining = set(range(self.n))                  # List of all remaining waypoints
        remaining.remove(start_index)                   # Remove starting waypoint
        
        current = start_index
        while remaining:    # Active while there are unvisited waypoints
            next_waypoint = min(remaining, key=lambda j: self.dist_matrix[current, j])   # Identifies next waypoint in path by finding waypoint with lowest cost
            visited.append(next_waypoint)       # Add next waypoint to path
            remaining.remove(next_waypoint)     # Remove next waypoint from unvisited waypoint list
            current = next_waypoint             # Set current waypoint to the next waypoint

        return visited
    
    def solve_two_opt(self, start_index=0):
        """This function calls on a greedy solver then runs the found path through a two-opt solver to optimize it."""
        greedy_order = self.solve_greedy(start_index)
        improved_order = self.two_opt(greedy_order)
        return improved_order


    def path_length(self, path, turn_weight=0.5):
        """ This function computes the total path length"""
        total = 0
        
        ## Distance calculation ##
        for i in range((len(path)-1)):
            total += self.dist_matrix[path[i], path[i+1]]
        
        ## Turning Penalty ##    
        for i in range(1, (len(path)-1)):
            A = self.points[path[i-1]]      # Previous waypoint
            B = self.points[path[i]]        # Current waypoint
            C = self.points[path[i+1]]      # Next waypoint
            
            v1 = B - A  # Vector from current waypoint to previous waypoint
            v2 = C - B  # Vector from next waypoint to current waypoint

            if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:  # Skip over any normals that are zero length
                continue
                    
            # Normalize
            n1 = v1 / np.linalg.norm(v1)
            n2 = v2 / np.linalg.norm(v2)
            
            # Angle
            dot = np.clip(np.dot(n1,n2),-1.0, 1.0)  # Dot product
            angle = np.arccos(dot)                  # Angle calc
            
            total += turn_weight * angle            # Apply weighting to angle (higher angle creates higher penalty)
            
        return total
    
    def two_opt(self, path, max_iterations=100):
        """ This function trys to optimize pathing between waypoints by identifying crossover points and checking that it creates a shorter path."""
        
        improved = True
        iteration = 0
        opt_path = path
        opt_distance = self.path_length(opt_path)
        
        while improved and iteration < max_iterations:         # Loops while improved variable is True and under iteration limit
            improved = False    # Set improved to false so the loop breaks if no optimal path is found
            iteration += 1
            print(f"Iteration: {iteration}")
            for i in range(1, len(opt_path) - 2):        
                for j in range(i+1, len(opt_path) - 1): 
                    if j-i == 1:    # Skips adjacent edges
                        continue    
                    
                    # Compute cost difference using delta evaluation
                    A = opt_path[i - 1]
                    B = opt_path[i]
                    C = opt_path[j]
                    D = opt_path[j + 1]
                    
                    old_cost = (self.dist_matrix[A, B] + self.dist_matrix[C, D])
                    new_cost = (self.dist_matrix[A, C] + self.dist_matrix[B, D])
                    
                    if new_cost >= old_cost:    # Skips over any calc that doesnt result in an improvement
                        continue
                    
                    # Build the new path
                    new_path = (
                        opt_path[:j]            # Everything before point i
                        + opt_path[i:j][::-1]   # Reversed segment from i -> j
                        + opt_path[j:]          # Everythiny after point j 
                    )
                    
                    new_distance = self.path_length(new_path)
                    
                    if new_distance < opt_distance - 1e-6:
                        opt_distance = new_path
                        opt_distance = new_distance
                        improved = True
                        break                           # Break for j loop
                    
                if improved:    
                    break   # Break for i loop
        
        return opt_path