import argparse
import configparser
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.logging import set_logger_level, LoggingSeverity
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import heapq
import math

#my one
np.set_printoptions(2, suppress=True, threshold=np.inf) # Print numpy arrays to specified d.p., suppress scientific notation and do not truncate
set_logger_level("obstaclecourse", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  




_PACKAGE_DIR = os.path.dirname(os.path.realpath(__file__))

#helpers copied from CA2's provided code
# --- Coordinate conversion --------------------------------------------------
# A grid index (i, j) represents a CELL, not a point. That cell's world
# coordinate is its CENTER, e.g. cell [0, 0] is centred half a resolution-step
# away from the grid's origin corner, not exactly on it. This matches how the
# Gazebo world and the real maze are physically laid out (goal tape/markers
# sit in the middle of a cell, not on its boundary line).
def grid_to_world(i:int, j:int, origin:tuple, resolution:float) -> tuple:
    '''Convert grid index (i, j) to the world (x, y) coordinate of that cell's centre.'''
    return (origin[0] + (i + 0.5) * resolution, origin[1] + (j + 0.5) * resolution)

def world_to_grid(x:float, y:float, origin:tuple, resolution:float) -> tuple:
    '''Convert a world (x, y) coordinate to the grid index (i, j) of the cell containing it.'''
    return (int(np.floor((x - origin[0]) / resolution)), int(np.floor((y - origin[1]) / resolution)))


def wrap_deg(angle:float) -> float:
    '''Wrap an angle in degrees into [-180, 180).'''
    return (angle + 180.0) % 360.0 - 180.0

def optitrack_to_maze(x:float, y:float, heading_deg:float, frame:dict) -> tuple:
    '''Optitrack-frame pose (x, y, heading in degrees) -> CA2 maze-frame pose (see optitrack_variables.config).'''
    dx, dy = x - frame["origin_x"], y - frame["origin_y"]
    r = math.radians(frame["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap_deg(heading_deg - frame["rotation_deg"] - frame["heading_offset_deg"]))

def maze_to_course(x:float, y:float, heading_deg:float, course:dict) -> tuple:
    '''CA2 maze-frame pose -> final-project course frame (origin/rotation of the course in the maze frame, [course] in the config).'''
    dx, dy = x - course["origin_x"], y - course["origin_y"]
    r = math.radians(course["rotation_deg"])
    c, s = math.cos(r), math.sin(r)
    return (c * dx + s * dy, -s * dx + c * dy, wrap_deg(heading_deg - course["rotation_deg"]))

def load_frame(robot_number:int) -> tuple:
    '''Read optitrack_variables.config: returns (frame, course) for this robot.'''
    parser = configparser.ConfigParser()
    path = os.path.join(_PACKAGE_DIR, "optitrack_variables.config")
    if not parser.read(path):
        raise FileNotFoundError(f"Could not read {path}")
    frame = {k: parser.getfloat("frame", k) for k in ("origin_x", "origin_y", "rotation_deg")}
    raw = parser.get("frame", "heading_offset_deg", fallback="auto").strip().lower()
    # Rigid bodies are created facing +y of the maze frame, so Motive's zero heading is maze +y (90 deg)
    frame["heading_offset_deg"] = wrap_deg(-(frame["rotation_deg"] + 90.0)) if raw == "auto" else float(raw)
    if parser.has_section("heading_offsets") and parser.has_option("heading_offsets", str(robot_number)):
        frame["heading_offset_deg"] = parser.getfloat("heading_offsets", str(robot_number))
    course = {k: parser.getfloat("course", k, fallback=0.0) for k in ("origin_x", "origin_y", "rotation_deg")}
    return frame, course

class LidarNode(Node):
    def __init__(self):
        """Node constructor"""
        super().__init__("LidarNode")
        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to lidar
        self.is_simulation = None

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360: 
            self.is_simulation = True
        else:
            self.is_simulation = False

class ObstacleCourseNode(Node):
    '''Node to navigate obstacle course, using pose from either gazebo odometer or optitrack and lidar scan data'''
    def __init__(self, is_simulation:bool=True, calibrate:bool=False):
        super().__init__('obstaclecourse')
        self.get_logger().info("Starting ObstacleCourseNode")

        self.is_simulation = is_simulation
        self.calibrate = calibrate # Real robot: only print poses, never drive
        self.raw_pose = None # Pose exactly as Optitrack reported it (real robot only)
        self.frame = self.course = None
        self._ticks = 0
        self.lidar_rays = 0

        if self.is_simulation:
            self.origin = (-0.5, -3.3) #this anchors the occupancy grid, its alw the "bottom left" corner of the occupancy grid
            self.global_goal = (4.7,-3.5) 
            self.robot_length = 0.22
            self.robot_width = 0.24
            self.lookahead_distance = 0.05
            self.pursuit_speed = 0.3 #can tune based on curvature of lookahead arc
            self.goal_tolerance = 0.05
            self.occupancy_grid_resolution = 0.01
            self.door_x = 3.4
            self.reached_right_wall = False

        else:
            self.origin = (-1.5, -1.5) #must figure out where the lowest x and lowest y of maze frame is, this will be the origin
            self.global_goal = (0.102, 0.758) #must figure out where the global goal is in maze frame
            self.robot_length = 0.3
            self.robot_width = 0.2
            self.lookahead_distance = 0.05
            self.pursuit_speed = 0.3 #can tune based on curvature of lookahead arc
            self.goal_tolerance = 0.05
            self.occupancy_grid_resolution = 0.005
            self.door_x = None

        
        

        if is_simulation:
            self.max_translate_velocity = 1.4
        else:
            self.max_translate_velocity = 0.3 # Please keep this in place; 0.3m/s is more than fast enough 

        self.sub_scan = self.create_subscription(LaserScan, "scan", self.sub_scan_callback, 2) # Subscribe to LiDAR scan data

        # Subscribe to the dynamic_pose topic from Gazebo that publishes ground-truth pose data
        if self.is_simulation:
            self.subscription = self.create_subscription(Odometry, 'odom', self.odometer_callback, 2)
        else:
            qos_profile = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
            # On the Bingda robots ROS_DOMAIN_ID is the robot's number, so the Optitrack topic is picked automatically
            domain = os.environ.get("ROS_DOMAIN_ID", "")
            self.robot_number = int(domain) if domain.isdigit() and 1 <= int(domain) <= 99 else 3
            self.frame, self.course = load_frame(self.robot_number)
            self.get_logger().info(f"Reading pose from /vrpn_mocap/bingda_{self.robot_number:03d}/pose")

            self.map_sub = self.create_subscription( 
                PoseStamped,
                f'/vrpn_mocap/bingda_{self.robot_number:03d}/pose',
                self.optitrack_callback, 
                qos_profile
                )
            
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # Publish to cmd_vel node       
        self.timer = self.create_timer(0.05, self.timer_callback)  # Runs at 20Hz. Can be changed.

        self.pose = None

        self.map_size_x = 6.02                                                    # metres the grid must cover in x direction
        self.grid_n_x = int(round(self.map_size_x / self.occupancy_grid_resolution))  # 301 cells in x-direction at 2 cm, 602 at 1 cm
        self.map_size_y = 4.5                                                     # metres the grid must cover in x direction
        self.grid_n_y = int(round(self.map_size_y / self.occupancy_grid_resolution))
        self.occupancy_grid = np.full((self.grid_n_x, self.grid_n_y), -1, dtype=np.int8) #-1 represents unexplored. we initialise the occupancy grid with all cells unexplored
        # self.occupancy_grid = np.full((301,301),-1,dtype = np.int8) 
        self.path = None
        self.started = False
        self.target = None
        self.last_scan = None
        self.counter = 0

        self.passed_door = False

        

    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        #360 scan rays in simulation
        if len(msg.ranges) <= 360: #this one for sim bro, only 360 rays
            self.last_scan = np.array(msg.ranges)
            
        else: #this one for the real world(what is really real?)
            self.last_scan = np.array(msg.ranges)[::2] 
            print(len(msg.ranges))

    def yaw_from_quaternion(self, q):
        '''Returns yaw angle (in rad) for orientation based on given quaternion input q'''
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def optitrack_callback(self, msg:PoseStamped):
        '''Callback to calculate 2D pose info from Optitrack node. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        x, y = msg.pose.position.x, msg.pose.position.y
        heading = np.rad2deg(self.yaw_from_quaternion(msg.pose.orientation))
        self.raw_pose = np.array((x,y,heading))
        x, y, heading = optitrack_to_maze(x, y, heading, self.frame) # CA2 maze frame, see optitrack_variables.config
        self.pose = np.array((x,y,heading))
        return self.pose

    def to_course(self, pose=None):
        '''Convert a pose [x, y, heading] (default: self.pose) from the CA2 maze frame to the final-project course frame. Optional; see [course] in optitrack_variables.config'''
        pose = self.pose if pose is None else pose
        return np.array(maze_to_course(pose[0], pose[1], pose[2], self.course)) if self.course else np.array(pose)

    def odometer_callback(self, msg):
        '''Callback to calculate 2D pose info from Gazebo odomoter. Pose info includes x and y coordinates, as well as heading in degrees.
        This callback will run everytime the rclpy executor spins'''
        latest_pose_msg = msg.pose.pose 
        heading = np.rad2deg(self.yaw_from_quaternion(latest_pose_msg.orientation))
        self.pose = np.array((latest_pose_msg.position.x, latest_pose_msg.position.y, heading))
        return self.pose

    def move_2D(self, x:float=0.0, y:float=0.0, turn:float=0.0):
        '''Publishes a Twist message to ROS to move a robot. Inputs are x and y linear velocities, as well as turn (z-axis yaw) angular velocity.'''
        twist_msg = Twist()
        x = np.clip(x, -self.max_translate_velocity, self.max_translate_velocity)
        y = np.clip(y, -self.max_translate_velocity, self.max_translate_velocity)
        turn = np.clip(turn, -self.max_translate_velocity*2, self.max_translate_velocity*2)
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = float(x), float(y), 0.0
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = 0.0, 0.0, float(turn)
        self.publisher_.publish(twist_msg)

    def timer_callback(self):
        """Controller loop. Insert path planning and PID control logic here"""

        #create a score function with weights; weighs the depth(how far the lidar ray detects the obstacles to be), width etc
        #create two blocks: one for sim one for real run
        
        if self.pose is None:
            print("No pose detected")
            return # Does not run if no pose received from Odom or Optitrack

        if self.calibrate and self.raw_pose is not None:
            self._ticks += 1
            if self._ticks % 10 == 0: # twice a second
                ox, oy, oh = self.raw_pose
                self.get_logger().info(f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  maze ({self.pose[0]:+.3f}, {self.pose[1]:+.3f}, {self.pose[2]:+7.1f} deg)")
            return # Calibration mode: look, don't drive
        # print(f"x:{self.pose[0]}, y:{self.pose[1]}")
        # self.move_2D(0.5,0,0)
        ###### INSERT CODE HERE ######
        self.counter+=1
        #test pose
        # if True:
        #     print(self.pose[0],self.pose[1])
        #     return
        
        if self.is_simulation: #for sim city 4
        #use lidar scan to update the occupancy grid map
            if self.last_scan is None:
                return
            #use bresenham to register free cells first:0, then whatever coordaintes the lidar scan returns, convert it to grid cells,
            #then update them as occupied cells: 99
            #bresenham line algo used for free cell calculation

            heading = np.deg2rad(self.pose[2])
            cx = self.pose[0] + 0.0725 * np.cos(heading)    # robot's center. the published pose is offset from robot's physical center
            cy = self.pose[1] + 0.0725 * np.sin(heading)
            lidar_x = self.pose[0] + 0.114 * np.cos(heading)
            lidar_y = self.pose[1] + 0.114 * np.sin(heading)
            li, lj = world_to_grid(lidar_x, lidar_y,self.origin,self.occupancy_grid_resolution)                       # start cell: the LiDAR

            for scan_idx in range(len(self.last_scan)):
                hypotenus = self.last_scan[scan_idx]
                if not np.isfinite(hypotenus) or hypotenus < 0.05:
                    continue
                theta = heading + scan_idx * (6.28 / 359)
                hi, hj = world_to_grid(lidar_x + hypotenus * np.cos(theta),
                                lidar_y + hypotenus * np.sin(theta),self.origin, self.occupancy_grid_resolution)  # end cell: where the ray hit

                cells = bresenham(li, lj, hi, hj)
                for (i, j) in cells[:-1]:                            # everything before the hit: free
                    if 0 <= i < self.grid_n_x and 0 <= j < self.grid_n_y:
                        #if self.occupancy_grid[i,j] != 99: #what if i comment out this line so that our robot only trusts the latest lidar?
                        self.occupancy_grid[i, j] = 0
                if 0 <= hi < self.grid_n_x and 0 <= hj < self.grid_n_y:                  # the hit itself: occupied
                    self.occupancy_grid[hi, hj] = 99
            #now we done with building the occupancy grid
            occ = (self.occupancy_grid == 99)                     # rebuild from the raw grid every time
            #should the below 3 arrays be variables or object attributes? for eg: self.buffered instead
            buffered = inflate_square(occ, round((self.robot_length/2)/self.occupancy_grid_resolution),round((self.robot_width/2)/self.occupancy_grid_resolution))      # 8 cells x 2 cm = 0.16 m, buffered is the grid to use for traversal
            frontier = frontier_cells(buffered, self.occupancy_grid)
            

            #what if we replan A* on every other callback?
            if self.passed_door:
                distance_to_goal = np.hypot(self.global_goal[0]-cx,self.global_goal[1]-cy)
                if distance_to_goal<=self.goal_tolerance:
                    self.move_2D(0,0,0)
                    print("reached goal yay")

                #go for the right wall straight until 2cm away
                if self.last_scan[270]>=0.15 and False:
                    self.move_2D(0,-0.5,0)
                else:
                    front_left_quadrant = self.last_scan[:91]
                    front_right_quadrant = self.last_scan[270:]
                    min_distance = 0.25
                    if np.all(front_left_quadrant>min_distance) and np.all(front_right_quadrant>min_distance):
                        self.move_2D(0.5,0,0)
                    else:
                        print("something blocking front left n front right")
                        self.move_2D(0,0,0)

            else:
                if self.counter%2:
                    if cx>=self.door_x: #will this change in the real course??????????
                        print(f"passed door,{self.pose[0]}")
                        self.passed_door = True
                        self.target = select_target2(buffered,self.occupancy_grid,self.global_goal,self.occupancy_grid_resolution,
                                                    self.origin)
                        self.path = A_star(buffered,world_to_grid(cx,cy,self.origin,self.occupancy_grid_resolution),
                                    (self.target[0], self.target[1]))
                        if not self.path:
                            self.get_logger().info('no path found bro')
                    else:
                        # self.target = select_target(frontier, self.global_goal,self.origin,self.occupancy_grid_resolution)
                        self.target = select_target2(buffered,self.occupancy_grid,self.global_goal,self.occupancy_grid_resolution,
                                                     self.origin)
                        self.path = A_star(buffered,world_to_grid(cx,cy,self.origin,self.occupancy_grid_resolution),
                                    (self.target[0], self.target[1]))
                        if not self.path:
                            self.get_logger().info('no path found bro')
                
                #now i have an A* path that is replanned whenever half the previous plan has finished
                #now i need the pid controller

                if self.path:
                    vx, vy = pure_pursuit(self.path, cx, cy, self.origin,self.occupancy_grid_resolution,
                                        self.lookahead_distance,self.pursuit_speed,self.goal_tolerance)
                    self.move_2D(vx, vy, 0.0)      # no heading control, just strafing it up
                else:
                    self.move_2D(0.0, 0.0, 0.0)    # no path to follow





        else: #for real world
            if self.last_scan is None:
                return
            #use bresenham to register free cells first:0, then whatever coordaintes the lidar scan returns, convert it to grid cells,
            #then update them as occupied cells: 99
            #bresenham line algo used for free cell calculation
            
            heading = np.deg2rad(self.pose[2])
            #i think need to find out the offset, if any, between the real robot's published pose and its physical center, so robot doesnt scrape shit
            lidar_x = self.pose[0] + 0.045 * np.cos(heading)
            lidar_y = self.pose[1] + 0.045 * np.sin(heading)
            cx = self.pose[0] + 0.0725 * np.cos(heading)    # robot's center. the published pose is offset from robot's physical center
            cy = self.pose[1] + 0.0725 * np.sin(heading)
            li, lj = world_to_grid(lidar_x, lidar_y,self.origin,self.occupancy_grid_resolution)                       # start cell: the LiDAR
            
            for scan_idx in range(len(self.last_scan)):
                hypotenus = self.last_scan[scan_idx]
                if not np.isfinite(hypotenus) or hypotenus < 0.05:
                    continue
                theta = heading + scan_idx * 2 * np.pi / len(self.last_scan)
                hi, hj = world_to_grid(lidar_x + hypotenus * np.cos(theta),
                                lidar_y + hypotenus * np.sin(theta),self.origin, self.occupancy_grid_resolution)  # end cell: where the ray hit
            
                cells = bresenham(li, lj, hi, hj)
                for (i, j) in cells[:-1]:                            # everything before the hit: free
                    if 0 <= i < self.grid_n_x and 0 <= j < self.grid_n_y:
                        #if self.occupancy_grid[i,j] != 99: 
                        self.occupancy_grid[i, j] = 0
                if 0 <= hi < self.grid_n_x and 0 <= hj < self.grid_n_y:                  # the hit itself: occupied
                    self.occupancy_grid[hi, hj] = 99
            #now we done with building the occupancy grid
            occ = (self.occupancy_grid == 99)                     # rebuild from the raw grid every time
            #should the below 3 arrays be variables or object attributes? for eg: self.buffered instead
            buffered = inflate_square(occ, round((self.robot_length/2)/self.occupancy_grid_resolution),round((self.robot_width/2)/self.occupancy_grid_resolution))      # 8 cells x 2 cm = 0.16 m, buffered is the grid to use for traversal
            frontier = frontier_cells(buffered, self.occupancy_grid)
            #what if we replan A* on every other callback?
            if self.counter%2:
                if cx>=2.9:
                    print("passed door")
                    self.target = select_target2(buffered,self.occupancy_grid,self.global_goal,self.occupancy_grid_resolution,
                                                 self.origin)
                    self.path = A_star(buffered,world_to_grid(cx,cy,self.origin,self.occupancy_grid_resolution),
                                (self.target[0], self.target[1]))
                    if not self.path:
                        self.get_logger().info('no path found bro')
                else:
                    self.target = select_target(frontier, self.global_goal,self.origin,self.occupancy_grid_resolution)
                    self.path = A_star(buffered,world_to_grid(cx,cy,self.origin,self.occupancy_grid_resolution),
                                (self.target[0], self.target[1]))
                    if not self.path:
                        self.get_logger().info('no path found bro')
            
            
            
            
            #i think now i have an A* path that is replanned whenever half the previous plan has finished
            #now i need the pid controller
            
            if self.path:
                vx, vy = pure_pursuit(self.path, cx, cy, self.origin,self.occupancy_grid_resolution,self.lookahead_distance,
                                      self.pursuit_speed,self.goal_tolerance)
                self.move_2D(vx, vy, 0.0)      # no heading control, just strafing it up
            else:
                self.move_2D(0.0, 0.0, 0.0)    # no path to follow
            
        

def select_target2(buffered, raw, global_goal,occupancy_grid_resolution,origin):
#fuck i need a grid that shows unoccupied cells as 0 but the obstacles must be expanded as well. i have buffered that shows every occupied 
#cell as True, and i have the self.original occupancy grid unexpanded that shows the free cells as 0. those cells that are 0 and not a buffer are free
#known cells
    free = ~buffered
    known = (raw==0)
    free_known = free & known
    gi, gj = global_goal
    #convert global goal world coordinates into grid coords
    gi, gj = world_to_grid(gi,gj,origin,occupancy_grid_resolution)
    fi, fj = np.nonzero(free_known)
    d = np.hypot(gi-fi,gj-fj)
    k = np.argmin(d)
    target = (fi[k],fj[k]) #the goal A* plans a path towards
    return target

    




def select_target(frontier,global_goal,origin,occupancy_grid_resolution):
    gi, gj = global_goal
    #convert global goal world coordinates into grid coords
    gi, gj = world_to_grid(gi,gj,origin,occupancy_grid_resolution)
    fi, fj = np.nonzero(frontier)
    d = np.hypot(gi-fi,gj-fj)
    k = np.argmin(d)
    target = (fi[k],fj[k]) #the goal A* plans a path towards
    return target

def halfway_reached(path, robot_cell):
    '''True once the path cell closest to the robot is at or past the middle of the path.'''
    pts = np.array(path)                                            # shape (n, 2): grid (i, j) of each path cell
    d = np.hypot(pts[:, 0] - robot_cell[0], pts[:, 1] - robot_cell[1])
    return np.argmin(d) >= len(path) // 2

def frontier_cells(buffered,raw):
    #this helper creates an array that contains the coordinates of all the frontier cells in our occupancy grid
    original = (raw == 0)
    free = ~buffered & original #all cells that are free and safe to travel to are True in free

    #now create an array that holds only the unknown cells and then make their 8-connected neighbours also unknown so later we can & with the free cells to get frontier cells
    unknown = (raw == -1)
    output = unknown.copy()
    
    output[1:, :]  |= unknown[:-1, :]     #   copy shifted down by k
    output[:-1, :] |= unknown[1:, :]      #   copy shifted up by k

    tmp = output.copy()
    output[:, 1:]  |= tmp[:, :-1]
    output[:, :-1] |= tmp[:, 1:]

    frontier = output & free
    return frontier

def inflate_square(occ, length,width):
    '''occ: bool array, True = occupied. Returns occ grown by r cells in every direction (a square).'''
    out = occ.copy()
    for k in range(1, length + 1):          # pass 1: grow along axis 0
        out[k:, :]  |= occ[:-k, :]     #   copy shifted down by k
        out[:-k, :] |= occ[k:, :]      #   copy shifted up by k
    tmp = out.copy()
    for k in range(1, width + 1):          # pass 2: grow that result along axis 1
        out[:, k:]  |= tmp[:, :-k]
        out[:, :-k] |= tmp[:, k:]
    return out
                    

        ###### INSERT CODE HERE ######

#thank goodness for bresenham
def bresenham(i0, j0, i1, j1):
    '''All grid cells on the line from (i0, j0) to (i1, j1), both ends included.'''
    cells = []
    di = abs(i1 - i0)            # how far to go in i
    dj = -abs(j1 - j0)           # how far to go in j (stored negative, makes the maths symmetric)
    si = 1 if i0 < i1 else -1    # step direction in i
    sj = 1 if j0 < j1 else -1    # step direction in j
    err = di + dj                # how far the current cell is from the true line
    i, j = i0, j0
    while True:
        cells.append((i, j))
        if i == i1 and j == j1:
            break
        e2 = 2 * err
        if e2 >= dj:             # the line has moved far enough in i: step in i
            err += dj
            i += si
        if e2 <= di:             # the line has moved far enough in j: step in j
            err += di
            j += sj
    return cells


#helper functions for A*
def calculate_h(start, dest):
    start_row, start_col = start
    target_row, target_col = dest #assuming dest is a tuple of 2 values
    #calculate euclidean distance from current location to dest (admissible heuristic)
    return ((target_col-start_col)**2 +(target_row-start_row)**2)**0.5
# def is_valid(map_array,row,col):
#     r,c = len(map_array), len(map_array[0])
#     return 0<=row<r and 0<=col<c
def is_goal(row,col,dest):
    return row==dest[0] and col == dest[1]
# def is_blocked(map_array,row,col): #99 = wall
#     return map_array[row,col]

#8 connected grid. returns "no feasible path if no path"
def A_star(map_array, startcell, goalcell):
    open_list = []
    closed = set()
    rows, cols = map_array.shape
    grid = map_array.tolist()                                 # plain Python list: fast to read one cell at a time
    g = [[math.inf] * cols for _ in range(rows)]              # cost from source (needs: import math)
    parent_i = [[-1] * cols for _ in range(rows)]             # parent's row; -1 = no parent (marks the start)
    parent_j = [[-1] * cols for _ in range(rows)]             # parent's column
    si, sj = startcell
    g[si][sj] = 0
    heapq.heappush(open_list, (calculate_h(startcell, goalcell), si, sj))
    while open_list:
        _, i, j = heapq.heappop(open_list)
        if (i, j) in closed:
            continue
        if is_goal(i, j, goalcell):
            path = [(i, j)]
            while parent_i[i][j] != -1:
                i, j = parent_i[i][j], parent_j[i][j]
                path.append((i, j))
            path.reverse()
            return path
        closed.add((i, j))
        nb_list = [(i+1, j), (i, j+1), (i-1, j), (i, j-1), (i+1, j+1), (i-1, j+1), (i+1, j-1), (i-1, j-1)]
        for neighbour in nb_list:
            ni, nj = neighbour
            if 0 <= ni < rows and 0 <= nj < cols and not grid[ni][nj] and neighbour not in closed:
                diag = (nj!=j) and (ni!=i)
                new_g = g[i][j] + (1.414 if diag else 1)#diagonal travel cost is 1.414  
                if new_g < g[ni][nj]:
                    g[ni][nj] = new_g
                    parent_i[ni][nj] = i
                    parent_j[ni][nj] = j
                    heapq.heappush(open_list, (new_g + calculate_h(neighbour, goalcell), ni, nj))
    return []

def pure_pursuit(path, x, y, origin, resolution,
                 lookahead, speed,goal_tolerance):
    
    pts = np.array(path, dtype=float)                       # grid (i, j) of every path cell
    wx = origin[0] + (pts[:, 0] + 0.5) * resolution         # world x of cell center
    wy = origin[1] + (pts[:, 1] + 0.5) * resolution         # world y of cell centre
    d = np.hypot(wx - x, wy - y)                            # distance from the robot to every path point
    k = int(np.argmin(d))                                   # index of the closest path point
    ahead = np.nonzero(d[k:] >= lookahead)[0]               # path points from k onwards that are at least `lookahead` away
    t = k + int(ahead[0]) if ahead.size else len(path) - 1  # first of those, or the last point if none is that far
    dx, dy = wx[t] - x, wy[t] - y
    dist = math.hypot(dx, dy)
    if t == len(path) - 1 and dist < goal_tolerance:
        return 0.0, 0.0                                     # reached the end of the path
    return speed * dx / dist, speed * dy / dist             # returns speed, pointed at target
        
        




def main(args=None):
    arg_parser = argparse.ArgumentParser(description="RB2301 final project")
    arg_parser.add_argument('--calibrate', action='store_true', help="Real robot: only print the Optitrack pose and the maze-frame pose twice a second, never drive")
    cli_args, ros_args = arg_parser.parse_known_args(args=args)
    print("Starting obstacle course")
    rclpy.init(args=ros_args)

    # Use lidar scan length to determine if in simulation or not
    lidar = LidarNode()
    while lidar.is_simulation is None:
        rclpy.spin_once(lidar)
    is_simulation = lidar.is_simulation
    lidar.destroy_node()

    # Start spinning the obstacle node and only stop once SystemExit error is raised within the node callback
    obstacle = ObstacleCourseNode(is_simulation, cli_args.calibrate)
    try:
        rclpy.spin(obstacle)
    except SystemExit:
        print("Shutting down")
    obstacle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
