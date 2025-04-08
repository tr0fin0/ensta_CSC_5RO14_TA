#!/usr/bin/env python3
import numpy as np
import heapq



EXPANSION_RADIUS = 15

class AStarPlanner:
    """
    A class that encapsulates obstacle expansion and the A* algorithm.
    """

    class Node:
        """
        A node used for A* pathfinding.

        Attributes:
            position (tuple): The (row, col) position in the grid.
            g (float): Cost from the start node.
            h (float): Heuristic cost to the goal.
            f (float): Total cost (g + h).
            parent (AStarPlanner.Node): Parent node in the path.
        """
        def __init__(self, position, g=0, h=0, parent=None):
            self.position = position  # (row, col) in the grid
            self.g = g                # cost from start
            self.h = h                # heuristic cost to goal
            self.f = g + h            # total cost
            self.parent = parent

        def __lt__(self, other):
            """
            Less-than comparison for priority queue ordering.
            """
            return self.f < other.f


    @staticmethod
    def expand_obstacles(grid, expansion_radius):
        """
        Expand obstacles in the occupancy grid by a given expansion radius.

        Every cell that is an obstacle (value == 100) will cause its neighbors to be marked as obstacles.

        Parameters:
            grid (np.ndarray): 2D numpy array representing the occupancy grid.
            expansion_radius (int): Number of cells to expand around each obstacle.

        Returns:
            np.ndarray: A new grid with expanded obstacles.
        """
        expanded = grid.copy()
        rows, cols = grid.shape
        obstacles = np.where(grid == 100)

        for r, c in zip(obstacles[0], obstacles[1]):
            for i in range(max(0, r - expansion_radius), min(rows, r + expansion_radius + 1)):
                for j in range(max(0, c - expansion_radius), min(cols, c + expansion_radius + 1)):
                    expanded[i, j] = 100

        return expanded


    @staticmethod
    def heuristic(a, b):
        """
        Compute the Euclidean distance between two points as the heuristic.

        Parameters:
            a (tuple): The first point (row, col).
            b (tuple): The second point (row, col).

        Returns:
            float: The Euclidean distance between a and b.
        """
        return np.linalg.norm(np.array(a) - np.array(b))


    @classmethod
    def astar(cls, grid, start, goal, expansion_radius=EXPANSION_RADIUS):
        """
        Perform A* search on the occupancy grid from start to goal.
        Obstacles are expanded by expansion_radius cells.
        
        Parameters:
            grid: 2D numpy array of the occupancy grid.
            start: Tuple (row, col) for the starting point.
            goal: Tuple (row, col) for the goal.
            expansion_radius: Radius (in cells) to expand obstacles.
        
        Returns:
            path: A list of (row, col) tuples representing the path.
            expanded_grid: The grid after obstacle expansion.
        """
        grid_expanded = cls.expand_obstacles(grid, expansion_radius)
        rows, cols = grid.shape
        open_list = []
        closed_set = set()
        
        start_node = cls.Node(start, g=0, h=cls.heuristic(start, goal))
        heapq.heappush(open_list, start_node)
        
        while open_list:
            current = heapq.heappop(open_list)
            if current.position == goal:
                # Reconstruct path by following parent pointers.
                path = []
                while current:
                    path.append(current.position)
                    current = current.parent
                return path[::-1], grid_expanded
            
            closed_set.add(current.position)
            
            # Consider 8-connected neighbors.
            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue

                    neighbor_pos = (current.position[0] + dr, current.position[1] + dc)
                    if (neighbor_pos[0] < 0 or neighbor_pos[0] >= rows or
                        neighbor_pos[1] < 0 or neighbor_pos[1] >= cols):
                        continue

                    if grid_expanded[neighbor_pos] == 100:
                        continue

                    if neighbor_pos in closed_set:
                        continue

                    tentative_g = current.g + np.linalg.norm(np.array([dr, dc]))
                    neighbor_node = cls.Node(neighbor_pos, tentative_g, cls.heuristic(neighbor_pos, goal), current)
                    
                    # If already in open_list with lower g, skip.
                    skip = False
                    for open_node in open_list:
                        if neighbor_pos == open_node.position and tentative_g >= open_node.g:
                            skip = True
                            break
                    if skip:
                        continue
                    
                    heapq.heappush(open_list, neighbor_node)
        
        # No path found
        return None, grid_expanded
