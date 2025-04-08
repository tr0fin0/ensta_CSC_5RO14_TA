#!/usr/bin/env python3
from astar_planner import AStarPlanner, EXPANSION_RADIUS
from pid_controller import PIDController

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
import math


COLOR_OBSTACLE = [0, 0, 0]
COLOR_FREE = [255, 255, 255]
COLOR_UNKNOWN = [127, 127, 127]

VALUE_OBSTACLE = 100
VALUE_FREE = 0
VALUE_UNKNOWN = -1


DT = 0.1                # 10 Hz rate
EXPANSION_RADIUS = 18   # Expansion radius for A* algorithm
GOAL_TOLERANCE = 0.1    # meters
KP_ANGULAR = 0.75
KP_LINEAR = 0.50
KI_ANGULAR = 0.0
KI_LINEAR = 0.0
KD_ANGULAR = 0.1
KD_LINEAR = 0.05
MAX_ANGULAR_VEL = 1.5
MAX_LINEAR_VEL = 0.8


ASTAR_PATH_POINT_COUNT = 20




class MapVisualizer:
    """
    A class that handles ROS occupancy grid subscription and visualization.
    """

    def __init__(self):
        """
        Initialize the ROS node, subscribers, and publishers.
        """
        rospy.init_node('map_and_path_planner', anonymous=True)

        self.map_data = None
        self.start_world = None   # (x, y) in meters
        self.current_pose = None  # (x, y, yaw) in meters and radians (updated continuously)
        self.goal_world = None
        self.start = None
        self.goal = None

        rospy.Subscriber('/map_2', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/natnet_ros/base_link/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)

        rospy.loginfo("Waiting for map data, start, and goal...")
        while (
            self.map_data is None or self.start is None or self.goal is None
        ) and not rospy.is_shutdown():
            rospy.sleep(0.1)

        self.process_map()


    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi].
        """
        return math.atan2(math.sin(angle), math.cos(angle))
    

    @staticmethod
    def quaternion_to_yaw(q):
        """
        Convert a quaternion to yaw (in radians).

        Parameters:
            q: Quaternion message.

        Returns:
            float: Yaw angle in radians.
        """
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y**2 + q.z**2)
        )
    
    
    def _convert_world_to_grid(self, world_coords):
        """
        Convert world coordinates to grid coordinates.

        Parameters:
            world_coords (tuple): (x, y) in world coordinates.

        Returns:
            tuple or None: (row, col) in grid coordinates, or None if out of bounds.
        """
        x, y = world_coords
        origin = self.map_data.info.origin.position
        resolution = self.map_data.info.resolution

        grid_col = int((x - origin.x) / resolution)
        grid_row = int((y - origin.y) / resolution)

        if (
            0 <= grid_row < self.map_data.info.height and
            0 <= grid_col < self.map_data.info.width
        ):
            return (grid_row, grid_col)
        else:
            return None


    def convert_start_to_grid(self):
        """
        Convert the stored world start coordinates to grid coordinates.
        """
        grid_coords = self._convert_world_to_grid(self.start_world)

        if grid_coords:
            self.start = grid_coords
            rospy.loginfo(f"Position updated: ({grid_coords[0]:03d}, {grid_coords[1]:03d}), start")
        else:
            rospy.logwarn("Pose out of grid bounds, start")
            self.start = None


    def convert_goal_to_grid(self):
        """
        Convert stored world goal coordinates to grid coordinates.
        """
        grid_coords = self._convert_world_to_grid(self.goal_world)

        if grid_coords:
            self.goal = grid_coords
            rospy.loginfo(f"Position updated: ({grid_coords[0]:03d}, {grid_coords[1]:03d}), goal")
        else:
            rospy.logwarn("Pose out of grid bounds, goal")
            self.goal = None


    def display_grid_from_file(self, file_path):
        """
        Load an occupancy grid from a .npy file and display it.
        
        Parameters:
            file_path (str): Path to the .npy file.
        """
        grid = np.load(file_path)
        color_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        color_grid[grid == VALUE_OBSTACLE]  = COLOR_OBSTACLE
        color_grid[grid == VALUE_FREE]      = COLOR_FREE
        color_grid[grid == VALUE_UNKNOWN]    = COLOR_UNKNOWN
        
        plt.figure()
        plt.imshow(color_grid, origin='lower')
        plt.title("Loaded Occupancy Grid")
        plt.xlabel("Y [m]")
        plt.ylabel("Y [m]")
        plt.grid(True)
        plt.tight_layout()
        plt.show()


    def follow_path(self):
        """
        PID control loop to follow the path.
        """
        pid_angular = PIDController(kp=KP_ANGULAR, ki=KI_ANGULAR, kd=KD_ANGULAR, max_output=MAX_ANGULAR_VEL)
        pid_linear = PIDController(kp=KP_LINEAR, ki=KI_LINEAR, kd=KD_LINEAR, max_output=MAX_LINEAR_VEL)
        rate = rospy.Rate(10)
        
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
            if distance < GOAL_TOLERANCE:
                self.current_waypoint_index += 1

                if self.current_waypoint_index < len(self.path_world):
                    rospy.loginfo(f"PID Path: reached waypoint {self.current_waypoint_index}/{len(self.path_world)}")
                else:
                    rospy.loginfo("PID Path: goal reached!")
                    self.cmd_vel_pub.publish(Twist())  # Stop
                    break
            
            # PID control
            angular_vel = pid_angular.compute(angle_error, DT)
            linear_vel = pid_linear.compute(min(distance, 0.5), DT) # limit distance error
            
            # Publish velocities
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist)
            rate.sleep()


    def goal_callback(self, msg):
        """
        Callback to update the goal pose in world coordinates.

        Parameters:
            msg (PoseStamped): The goal pose message.
        """
        self.goal_world = (msg.pose.position.x, msg.pose.position.y)

        if self.map_data is not None:
            self.convert_goal_to_grid()


    def map_callback(self, msg):
        """
        Callback to store map data and update start/goal grid coordinates if available.

        Parameters:
            msg (OccupancyGrid): The map data message.
        """
        self.map_data = msg

        if self.start_world is not None:
            self.convert_start_to_grid()

        if self.goal_world is not None:
            self.convert_goal_to_grid()


    def pose_callback(self, msg):
        """
        Callback to update the current robot pose.

        Parameters:
            msg (PoseStamped): The current pose message.
        """
        self.current_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            self.quaternion_to_yaw(msg.pose.orientation)
        )
  
        # Convert to grid for initial planning (if needed)
        if self.map_data is not None and self.start_world is None:
            self.start_world = (self.current_pose[0], self.current_pose[1])
            self.convert_start_to_grid()


    def start_callback(self, msg):
        """
        Store start pose in world coordinates and convert to grid if map is available.
        """
        self.start_world = (msg.pose.position.x, msg.pose.position.y)

        if self.map_data is not None:
            self.convert_start_to_grid()

    
    def plot_final_grid(self, expanded_grid, path):
        """
        Plot the expanded occupancy grid with the computed A* path.
        """
        color_grid = np.zeros((expanded_grid.shape[0], expanded_grid.shape[1], 3), dtype=np.uint8)
        color_grid[expanded_grid == VALUE_OBSTACLE] = COLOR_OBSTACLE
        color_grid[expanded_grid != VALUE_OBSTACLE] = COLOR_FREE
        
        _, ax = plt.subplots()
        extent = [
            0, self.map_data.info.width * self.map_data.info.resolution,
            0, self.map_data.info.height * self.map_data.info.resolution
        ]
        ax.imshow(color_grid, origin='lower', extent=extent)
        
        # Convert grid coordinates to meters for path plotting
        path = np.array(path)
        xs = path[:, 1] * self.map_data.info.resolution
        ys = path[:, 0] * self.map_data.info.resolution
        ax.plot(xs, ys, color='red', linewidth=2, label='A* Path')
        
        # Mark start and goal points
        ax.plot(
            self.start[1] * self.map_data.info.resolution, self.start[0] * self.map_data.info.resolution,
            marker='o', color='green', markersize=8, label='start'
        )
        ax.plot(
            self.goal[1] * self.map_data.info.resolution, self.goal[0] * self.map_data.info.resolution,
            marker='o', color='blue', markersize=8, label='goal'
        )
        
        ax.set_title("Expanded Occupancy Grid with A* Path")
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.grid(True)
        ax.legend(loc='upper right')
        
        plt.tight_layout()
        plt.savefig("expanded_occupancy_grid_with_path.png")
        plt.show()


    def plot_initial_grid(self, grid):
        """Plot the initial occupancy grid."""
        color_grid = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
        color_grid[grid == VALUE_OBSTACLE] = COLOR_OBSTACLE
        color_grid[grid == VALUE_FREE]   = COLOR_FREE
        color_grid[grid == VALUE_UNKNOWN]  = COLOR_UNKNOWN
        
        _, ax = plt.subplots()
        extent = [
            0, self.map_data.info.width * self.map_data.info.resolution,
            0, self.map_data.info.height * self.map_data.info.resolution
        ]
        ax.imshow(color_grid, origin='lower', extent=extent)
        ax.set_title("Initial Occupancy Grid")
        ax.set_xlabel("Y [m]")
        ax.set_ylabel("X [m]")
        ax.grid(True)
        
        plt.tight_layout()
        plt.savefig("initial_occupancy_grid.png")
        plt.show()

    
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
        

        if len(path) > 2:
            indices = np.round(np.linspace(0, len(path)-1, num=ASTAR_PATH_POINT_COUNT)).astype(int)
            indices = sorted(np.unique(indices))
            down_sampled_path = [path[i] for i in indices]
        else:
            down_sampled_path = path
        
        # Convert path to world coordinates
        self.path_world = []
        origin = self.map_data.info.origin.position
        resolution = self.map_data.info.resolution

        for (row, col) in down_sampled_path:
            x = origin.x + col * resolution
            y = origin.y + row * resolution
            self.path_world.append((x, y))
 
        self.current_waypoint_index = 0
        
        # Plot and start control loop
        self.plot_final_grid(expanded_grid, down_sampled_path)
        self.follow_path()



if __name__ == '__main__':
    try:
        visualizer = MapVisualizer()
    except rospy.ROSInterruptException:
        pass
