#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
import heapq
import time

from geometry_msgs.msg import Twist
import math


EXPANSION_RADIUS = 15

class AStarPlanner:
    """
    A class that encapsulates obstacle expansion and the A* algorithm.
    """
    
    class Node:
        """A node for A* pathfinding."""
        def __init__(self, position, g=0, h=0, parent=None):
            self.position = position  # (row, col) in the grid
            self.g = g                # cost from start
            self.h = h                # heuristic cost to goal
            self.f = g + h            # total cost
            self.parent = parent

        def __lt__(self, other):
            return self.f < other.f

    @staticmethod
    def expand_obstacles(grid, expansion_radius):
        """
        Expand obstacles in the occupancy grid by the specified expansion_radius.
        Every cell that is an obstacle (value == 100) expands to its neighbors.
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
        """Compute Euclidean distance as heuristic."""
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



class PIDController:
    """A simple PID controller."""
    def __init__(self, kp, ki, kd, max_output=float('inf'), min_output=-float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0
        self.previous_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.min_output, min(output, self.max_output))
        self.previous_error = error
        return output




class MapVisualizer:
    """
    A class that handles ROS occupancy grid subscription and visualization.
    All plotting is done with axes in meters.
    """
    def __init__(self):
        rospy.init_node('map_and_path_planner', anonymous=True)
        
        # Initialize variables
        self.map_data = None
        self.start_world = None   # (x, y) in meters (initial start)
        self.current_pose = None  # (x, y, yaw) in meters and radians (updated continuously)
        self.goal_world = None
        self.start = None
        self.goal = None
        
        # Subscribers and Publishers
        rospy.Subscriber('/map_2', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        rospy.loginfo("Waiting for map data, start, and goal...")
        while (self.map_data is None or self.start is None or self.goal is None) and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        self.process_map()
    

    def pose_callback(self, msg):
        """Update current robot pose (position and orientation)."""
        # Position
        self.current_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            self.quaternion_to_yaw(msg.pose.orientation)
        )
        # Convert to grid for initial planning (if needed)
        if self.map_data is not None and self.start_world is None:
            self.start_world = (self.current_pose[0], self.current_pose[1])
            self.convert_start_to_grid()

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert geometry_msgs/Quaternion to yaw (radians)."""
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y**2 + q.z**2))

    def start_callback(self, msg):
        """Store start pose in world coordinates and convert to grid if map is available."""
        self.start_world = (msg.pose.position.x, msg.pose.position.y)
        if self.map_data is not None:
            self.convert_start_to_grid()
    
    def goal_callback(self, msg):
        """Store goal pose in world coordinates and convert to grid if map is available."""
        self.goal_world = (msg.pose.position.x, msg.pose.position.y)
        if self.map_data is not None:
            self.convert_goal_to_grid()
    
    def map_callback(self, msg):
        """Store map data and convert existing start/goal if available."""
        self.map_data = msg
        if self.start_world is not None:
            self.convert_start_to_grid()
        if self.goal_world is not None:
            self.convert_goal_to_grid()
    
    def convert_start_to_grid(self):
        """Convert stored world start coordinates to grid coordinates."""
        x, y = self.start_world
        origin = self.map_data.info.origin.position
        res = self.map_data.info.resolution
        grid_col = int((x - origin.x) / res)
        grid_row = int((y - origin.y) / res)
        # Check bounds
        if 0 <= grid_row < self.map_data.info.height and 0 <= grid_col < self.map_data.info.width:
            self.start = (grid_row, grid_col)
            rospy.loginfo("Start position updated to grid: (%d, %d)", grid_row, grid_col)
        else:
            rospy.logwarn("Start pose out of grid bounds: (%d, %d)", grid_row, grid_col)
            self.start = None
    
    def convert_goal_to_grid(self):
        """Convert stored world goal coordinates to grid coordinates."""
        x, y = self.goal_world
        origin = self.map_data.info.origin.position
        res = self.map_data.info.resolution
        grid_col = int((x - origin.x) / res)
        grid_row = int((y - origin.y) / res)
        # Check bounds
        if 0 <= grid_row < self.map_data.info.height and 0 <= grid_col < self.map_data.info.width:
            self.goal = (grid_row, grid_col)
            rospy.loginfo("Goal position updated to grid: (%d, %d)", grid_row, grid_col)
        else:
            rospy.logwarn("Goal pose out of grid bounds: (%d, %d)", grid_row, grid_col)
            self.goal = None
    
    def process_map(self):
        """
        Convert the occupancy data to a numpy array, plot the initial grid,
        run A* with obstacle expansion, and plot the final grid with path.
        """
        grid = np.array(self.map_data.data, dtype=np.int8)
        grid = grid.reshape((self.map_data.info.height, self.map_data.info.width))
        self.plot_initial_grid(grid)
        
        result = AStarPlanner.astar(grid, self.start, self.goal, expansion_radius=EXPANSION_RADIUS)
        if result[0] is None:
            rospy.logwarn("No path found.")
            return
        path, expanded_grid = result
        
        # Downsample the path to 12 points (start + 10 intermediates + goal)
        if len(path) > 2:
            num_points = 20
            indices = np.round(np.linspace(0, len(path)-1, num=num_points)).astype(int)
            indices = sorted(np.unique(indices))  # Remove duplicates and sort
            downsampled_path = [path[i] for i in indices]
        else:
            downsampled_path = path
        
        # Convert path to world coordinates
        self.path_world = []
        origin = self.map_data.info.origin.position
        res = self.map_data.info.resolution
        for (row, col) in downsampled_path:
            x = origin.x + col * res
            y = origin.y + row * res
            self.path_world.append((x, y))
        self.current_waypoint_index = 0
        
        # Plot and start control loop
        self.plot_final_grid(expanded_grid, downsampled_path)
        self.follow_path()

    def follow_path(self):
        """PID control loop to follow the path."""
        pid_angular = PIDController(kp=0.75, ki=0.0, kd=0.1, max_output=1.5)
        pid_linear = PIDController(kp=0.50, ki=0.0, kd=0.05, max_output=0.8)
        rate = rospy.Rate(10)
        goal_tolerance = 0.1  # meters
        
        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.path_world):
            if self.current_pose is None:
                rate.sleep()
                continue
            
            # Current target waypoint
            target = self.path_world[self.current_waypoint_index]
            current_x, current_y, current_yaw = self.current_pose
            
            # Calculate errors
            dx = target[0] - current_x
            dy = target[1] - current_y
            distance = math.hypot(dx, dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = self.normalize_angle(angle_to_target - current_yaw)
            
            # Check if waypoint is reached
            if distance < goal_tolerance:
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.path_world):
                    rospy.loginfo(f"Reached waypoint {self.current_waypoint_index}/{len(self.path_world)}")
                else:
                    rospy.loginfo("Goal reached!")
                    self.cmd_vel_pub.publish(Twist())  # Stop
                    break
            
            # PID control
            dt = 0.1  # 10 Hz rate
            angular_vel = pid_angular.compute(angle_error, dt)
            linear_vel = pid_linear.compute(min(distance, 0.5), dt)  # Cap distance error
            
            # Publish velocities
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def plot_initial_grid(self, grid):
        """Plot the initial occupancy grid."""
        color_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        color_grid[grid == 100] = [0, 0, 0]         # Black for obstacles
        color_grid[grid == 0]   = [255, 255, 255]     # White for free space
        color_grid[grid == -1]  = [127, 127, 127]     # Gray for unknown
        
        fig, ax = plt.subplots()
        # extent in meters: [0, width*resolution, 0, height*resolution]
        extent = [0, self.map_data.info.width * self.map_data.info.resolution,
                  0, self.map_data.info.height * self.map_data.info.resolution]
        ax.imshow(color_grid, origin='lower', extent=extent)
        ax.set_title("Initial Occupancy Grid")
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.grid(True)
        
        plt.tight_layout()
        plt.savefig("initial_occupancy_grid.png")
        plt.show()
    
    def plot_final_grid(self, expanded_grid, path):
        """Plot the expanded occupancy grid with the computed A* path."""
        color_grid = np.zeros((expanded_grid.shape[0], expanded_grid.shape[1], 3), dtype=np.uint8)
        color_grid[expanded_grid == 100] = [0, 0, 0]         # Black for obstacles
        color_grid[expanded_grid != 100] = [255, 255, 255]     # White for free/unknown
        
        fig, ax = plt.subplots()
        extent = [0, self.map_data.info.width * self.map_data.info.resolution,
                  0, self.map_data.info.height * self.map_data.info.resolution]
        ax.imshow(color_grid, origin='lower', extent=extent)
        
        # Convert grid coordinates to meters for path plotting
        path = np.array(path)
        xs = path[:, 1] * self.map_data.info.resolution
        ys = path[:, 0] * self.map_data.info.resolution
        ax.plot(xs, ys, color='red', linewidth=2, label='A* Path')
        
        # Mark start and goal points
        ax.plot(self.start[1] * self.map_data.info.resolution, self.start[0] * self.map_data.info.resolution,
                marker='o', color='green', markersize=8, label='Start')
        ax.plot(self.goal[1] * self.map_data.info.resolution, self.goal[0] * self.map_data.info.resolution,
                marker='o', color='blue', markersize=8, label='Goal')
        
        ax.set_title("Expanded Occupancy Grid with A* Path")
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.grid(True)
        ax.legend(loc='upper right')
        
        plt.tight_layout()
        plt.savefig("expanded_occupancy_grid_with_path.png")
        plt.show()
    
    def display_grid_from_file(self, file_path):
        """
        Load an occupancy grid from a .npy file and display it.
        
        Parameters:
            file_path (str): Path to the .npy file.
        """
        grid = np.load(file_path)
        color_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        color_grid[grid == 100] = [0, 0, 0]         # Black for obstacles
        color_grid[grid == 0]   = [255, 255, 255]     # White for free space
        color_grid[grid == -1]  = [127, 127, 127]     # Gray for unknown
        
        plt.figure()
        plt.imshow(color_grid, origin='lower')
        plt.title("Loaded Occupancy Grid")
        plt.xlabel("Y [m]")
        plt.ylabel("Y [m]")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    try:
        visualizer = MapVisualizer()
        # Optionally, display a grid saved in a .npy file.
        # visualizer.display_grid_from_file("expanded_occupancy_grid.npy")
    except rospy.ROSInterruptException:
        pass
