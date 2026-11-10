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

np.set_printoptions(2, suppress=True, threshold=np.inf) # Print numpy arrays to specified d.p., suppress scientific notation and do not truncate
set_logger_level("obstaclecourse", level=LoggingSeverity.INFO) # Configure to either LoggingSeverity.INFO or LoggingSeverity.DEBUG  

'''
- split up into 16 directions, each 22.5 degrees apart
- for each direction, determine if moving 15cm in that direction would hit any obstacles. 
those that do not hit obstacles are feasible paths, and we move on to assign them each a score based on general goal directino and proximity
to previously visited points
- hence, we will need to periodically add the robot's center to an array of visited points for the score. tentatively: every 10 timer callbacks
if the speed is 0.3m/s, and the proximity score is linear and negative, meaning the closer to a visited point, the more negative the score.
we pick the highest overall score(score is calcualted based on where the robot would be after travelling 15cm in a particular direciton).

- for detecting dynamic obstacles (which are the swinging obstacles), the idea is: if something that was once there isnt there anymore,
we have found a dynamic obstacle. 
- this is done through detecting whether an obstacle that was hit by the lidar in a previous scan is no longer hit by the lidar in a subsequent
scan. (since the robot is always moving, we have to do some geometry to know which subsequent lidar ray would supposedly hit the same point,
after adjusting for robot movement)
'''

_PACKAGE_DIR = os.path.dirname(os.path.realpath(__file__))

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

        self.last_scan = None
        
        self.feasible_directions  = []

        self.goal_direction = np.deg2rad(315) #in degrees
        

        self.visited_points = []
        self.tick = 0

                               # m/s
        self.selected_direction = None           # current direction (radians, robot frame)
        self.decision_point = None               # (x, y) where that direction was chosen
        self.last_best_score = None
        self.last_best_theta = None

        

        

        if is_simulation:
            self.speed = 0.3  
            self.test_distance = 0.10
            self.R = 0.15 #the maximum radius that counts towards proximity score
            self.visited_weight = 0.2 #weight for proximity in calculating score for which direction to travel in
            self.L = 0.1075  #robot's half length
            self.W = 0.095 #robot's half width
            self.lidar_offset = 0.041    
            self.w_free = 1.0 #weight for room: w_free * free distance (up to 1 m)
            self.w_new = 1.8   #1.5 originally. weight for novelty: bonus for heading somewhere the robot hasn't been
            self.probe = 0.4   #novelty is judged at the point up to 0.4 m along each direction (less if blocked sooner)
            self.hysteresis = 0.1
            self.min_speed = 0.2 #originally 0.075
            self.t_wait = 10.0 #if the pendulum hasn't come in front of the robot within this time, cross anyway
            self.wall_gap = 0.0
        else:
            self.speed = 0.3  
            self.test_distance = 0.10
            self.R = 0.15 #the maximum radius that counts towards proximity score
            self.visited_weight = 0.2 #we ight for proximity in calculating score for which direction to travel in
            self.L = 0.115  #robot's half length
            self.W = 0.10  #robot's half width
            self.lidar_offset = 0.045
            self.w_free = 1.0 #weight for room: w_free * free distance (up to 1 m)
            self.w_new = 1.8   #weight for novelty: bonus for heading somewhere the robot hasn't been
            self.probe = 0.4   #novelty is judged at the point up to 0.4 m along each direction (less if blocked sooner)
            self.hysteresis = 0.1
            self.min_speed = 0.075
            self.t_wait = 4.0  #if the pendulum hasn't come in front of the robot within this time, cross anyway
            self.wall_gap = 0.04   # extra room needed ahead, so walls stay outside the LiDAR's 15 cm blind zone
        # pendulum handling
        self.clock = 0.0              # time in seconds, counted in ticks (20 Hz)
        self.scan_history = []        # (time, pose, scan) of the last second of scans
        self.mode = 'navigate'        # 'navigate', 'position', 'wait', 'cross', 'find', 'goal', 'done' or 'check'
        self.line = None              # world (x, y) of the pendulum's near face
        self.mode_start = 0.0         # time the current mode started
        self.seen = False             # whether the pendulum has been in front of the robot while waiting
        self.last_clear = 0.0         # last time nothing was in front of the robot while waiting
        self.moving_at_line = 0       # moving points seen at the line while waiting (only a real pendulum counts as crossed)
        self.false_lines = []         # world (x, y) of places that looked like a pendulum but weren't
        self.crossed = 0              # pendulums crossed so far
        self.cross_start = None       # world (x, y) where the dash started
        self.cross_length = 0.0       # how far to dash to get the robot's rear past the pendulum
        self.standoff = 0.10          # wait with the robot's front this far before the pendulum line
        self.course_heading = None    # heading (deg) to hold: the start heading, then the direction of the wall at each pendulum
        self.turning = False          # True while turning back to course_heading
        self.depth = 0.10             # the pendulum's thickness plus a margin
        



    def sub_scan_callback(self, msg):
        """Scan subscriber"""
        if len(msg.ranges) <= 360:
            self.last_scan = np.array(msg.ranges)
        else:
            self.last_scan = np.array(msg.ranges)[::2] 

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
        if self.pose is None:
            print("No pose detected")
            return # Does not run if no pose received from Odom or Optitrack

        if self.calibrate and self.raw_pose is not None:
            self._ticks += 1
            if self._ticks % 10 == 0: # twice a second
                ox, oy, oh = self.raw_pose
                self.get_logger().info(f"optitrack ({ox:+.3f}, {oy:+.3f}, {oh:+7.1f} deg)  ->  maze ({self.pose[0]:+.3f}, {self.pose[1]:+.3f}, {self.pose[2]:+7.1f} deg)")
            return # Calibration mode: look, don't drive

        ###### INSERT CODE HERE ######
        if self.last_scan is None:
            return 
        self.angle_step =  2* np.pi/len(self.last_scan) 

        #first transform the lidar scan into occupied points in robot's frame
        self.occupied_points  = scan_to_points(self.last_scan,self.angle_step,self.lidar_offset)

        #pendulum handling
        self.clock = self.get_clock().now().nanoseconds * 1e-9   # real time, whatever the loop rate
        if self.course_heading is None:
            self.course_heading = self.pose[2]                   # hold the heading the robot starts with
        err = wrap_deg(self.pose[2] - self.course_heading)       # how far the robot has turned off it
        if abs(err) > 6.0:
            self.turning = True                                    # start turning back once it's 6 deg off
        elif abs(err) < 2.0:
            self.turning = False                                   # stop once it's within 2 deg
        if self.turning and self.mode != 'cross':                  # (but never stop mid-dash)
            self.move_2D(0.0, 0.0, -np.sign(err) * 0.2)
            return
        if pendulum_step(self):           # the pendulum code is driving the robot this tick
            return

        # #then we check which of the 16 directions is feasible by seeing if we move in each direction by 15cm, would we hit anyt
        # self.lanes = all_lanes(self.occupied_points, self.L, self.W)          # {theta: (free distance, sideways shift)}
        # self.feasible_directions = [theta for theta, (free, shift) in self.lanes.items() if free >= self.test_distance]
        # if len(self.feasible_directions)==0:
        #     print("no feasible directions")
        #     self.move_2D(0,0,0)
        #     return
        # self.tick += 1
        # if self.tick % 1 == 0:
        #     self.visited_points.append((self.pose[0], self.pose[1]))
        
        
        # # choose a new direction at the start, once the robot has moved 15 cm since the last choice,
        # # or if something now blocks the rest of that 15 cm
        # if self.decision_point is None:
        #     moved = np.inf
        # else:
        #     moved = np.hypot(self.pose[0] - self.decision_point[0], self.pose[1] - self.decision_point[1])
        # path_blocked = (self.selected_direction is not None and moved < self.test_distance
        #                 and self.lanes[self.selected_direction][0] < self.test_distance - moved)
        # if self.selected_direction is None or moved >= self.test_distance or path_blocked:
        self.tick += 1
        self.visited_points.append((self.pose[0], self.pose[1]))

        # choose a new direction at the start, once the robot has moved test_distance since the last choice,
        # or if something now blocks the rest of it. Only then check all 16 directions (slow on the Jetson);
        # in between, only check the chosen one.
        if self.decision_point is None:
            moved = np.inf
        else:
            moved = np.hypot(self.pose[0] - self.decision_point[0], self.pose[1] - self.decision_point[1])
        if self.selected_direction is not None and moved < self.test_distance:
            lane = best_lane(self.occupied_points, self.selected_direction, self.L, self.W)
            #if lane[0] >= self.test_distance - moved:
            if lane[0] >= self.test_distance + self.wall_gap - moved:
                self.lanes = {self.selected_direction: lane}           # still clear: keep going this way
            else:
                moved = np.inf                                         # something now blocks it: decide again
        if self.selected_direction is None or moved >= self.test_distance:
            self.lanes = all_lanes(self.occupied_points, self.L, self.W)          # {theta: (free distance, sideways shift)}
            #self.feasible_directions = [theta for theta, (free, shift) in self.lanes.items() if free >= self.test_distance]
            self.feasible_directions = [theta for theta, (free, shift) in self.lanes.items() if free >= self.test_distance + self.wall_gap]
            if len(self.feasible_directions)==0:
                print("no feasible directions")
                self.move_2D(0,0,0)
                return
            result = best_direction(self.visited_points, self.feasible_directions, self.pose,
                                    self.goal_direction, self.R, self.visited_weight,self.test_distance,
                                    self.lanes, self.w_free,self.w_new,self.probe)
            new_theta, new_score, scores= result[0], result[3],result[4]
            keep_old = (self.last_best_theta in self.feasible_directions
                        and new_score <= scores[self.last_best_theta] + self.hysteresis)     # new one isn't better by more than 0.2
            if keep_old:
                self.selected_direction = self.last_best_theta
            else:
                self.selected_direction = new_theta
                self.last_best_theta, self.last_best_score = new_theta, new_score
            self.decision_point = (self.pose[0], self.pose[1])

        if self.selected_direction is None:
            self.move_2D(0.0, 0.0, 0.0)          # no feasible direction: stop
            print("nowehre to go")
        else:
            theta = self.selected_direction
            free, shift = self.lanes[theta]
            forward = self.speed
            if shift != 0.0:                       # lining up with a gap: don't drive into what the straight path hits
                forward = min(self.speed, free_distance(self.occupied_points, theta, self.L, self.W))
            sideways = np.clip(2.0 * shift, -0.15, 0.15)       # slide towards the lane (positive = left of theta)
            total = np.hypot(forward, sideways)
            if total > self.speed:                 # keep the total speed at most self.speed
                forward, sideways = forward * self.speed / total, sideways * self.speed / total
            elif 0 < total < self.min_speed:       # too slow for the real motors: scale up, same direction
                forward, sideways = forward * self.min_speed / total, sideways * self.min_speed / total
            self.move_2D(forward * np.cos(theta) - sideways * np.sin(theta),
                         forward * np.sin(theta) + sideways * np.cos(theta), 0.0)          
            


        ###### INSERT CODE HERE ######

#pendulum helper
def robot_to_world(points, pose):
    '''(n, 2) points in the robot frame -> world frame, using pose (x, y, heading in degrees).'''
    h = np.deg2rad(pose[2])
    x = pose[0] + points[:, 0] * np.cos(h) - points[:, 1] * np.sin(h)
    y = pose[1] + points[:, 0] * np.sin(h) + points[:, 1] * np.cos(h)
    return np.column_stack((x, y))


def world_to_robot(points, pose):
    '''(n, 2) points in the world frame -> robot frame (x forward, y left), using pose (x, y, heading in degrees).'''
    h = np.deg2rad(pose[2])
    dx, dy = points[:, 0] - pose[0], points[:, 1] - pose[1]
    return np.column_stack((dx * np.cos(h) + dy * np.sin(h), -dx * np.sin(h) + dy * np.cos(h)))


def seen_past(a_scan, a_pose, b_scan, b_pose, angle_step, lidar_offset, margin=0.10):
    '''World positions of the points hit in scan a that scan b sees more than `margin` past, i.e. points where
    something was in one scan and not in the other. Only points up to 1 m ahead (and up to 2 m to each side, the whole corridor) are checked.'''
    a = robot_to_world(scan_to_points(a_scan, angle_step, lidar_offset), a_pose)
    rel = world_to_robot(a, b_pose)                            # those points, seen from where scan b was taken
    ahead = (rel[:, 0] > 0.15) & (rel[:, 0] < 1.0) & (np.abs(rel[:, 1]) < 0.6)
    a, rel = a[ahead], rel[ahead]
    lx, ly = rel[:, 0] - lidar_offset, rel[:, 1]               # seen from the LiDAR
    dist = np.hypot(lx, ly)
    n = len(b_scan)
    idx = np.round(np.mod(np.arctan2(ly, lx), 2 * np.pi) / angle_step).astype(int) % n   # the ray of scan b pointing at each one
    b = np.asarray(b_scan, dtype=float)
    past = np.ones(len(dist), dtype=bool)
    for k in (-1, 0, 1):                                       # that ray and both its neighbours must reach past it
        r = b[(idx + k) % n]
        past &= np.isfinite(r) & (r > dist + margin)
    return a[past]

def in_front(points, L, W, line_x, depth, margin=0.10):
    '''True if any point is in front of the robot: from its right side to `margin` past its left side (the open side,
    away from the right wall), and from its front to `depth` past the pendulum line.'''
    x, y = points[:, 0], points[:, 1]
    return bool(np.any((x > L) & (x < line_x + depth) & (y >= -W) & (y <= W + margin)))


def pendulum_step(node):
    '''Pendulum handling. Once a pendulum is spotted: creep up to 5 cm before its line and slide to the right wall, wait
    until it has passed in front of the robot (or hasn't come for t_wait), dash across, then do the same for the
    second one. After the second, drive straight on until the robot can't go any further: the goal.
    Returns True if it drove the robot this tick (so normal navigation is skipped).'''
    # 1) spot moving things: compare each new scan with the one about 0.5 s older
    if not node.scan_history or node.last_scan is not node.scan_history[-1][2]:
        node.scan_history.append((node.clock, node.pose.copy(), node.last_scan))
        node.scan_history = [h for h in node.scan_history if node.clock - h[0] <= 1.0]
        older = [h for h in node.scan_history if node.clock - h[0] >= 0.5]
        if older:
            t0, pose0, scan0 = older[-1]
            moved = np.vstack((seen_past(scan0, pose0, node.last_scan, node.pose, node.angle_step, node.lidar_offset),   # something left
                               seen_past(node.last_scan, node.pose, scan0, pose0, node.angle_step, node.lidar_offset)))  # something arrived
            for f in node.false_lines:                             # ignore places already found not to be pendulums
                moved = moved[np.hypot(moved[:, 0] - f[0], moved[:, 1] - f[1]) > 0.2]
            rel = world_to_robot(moved, node.pose)
            if node.line is None and node.mode in ('navigate', 'find') and len(moved) >= 10:
                node.line = moved[np.argmin(rel[:, 0])]           # the pendulum's near face: the line to deal with next
                print(f"PENDULUM DETECTED, line {rel[:, 0].min():.2f} m ahead")
            if node.mode == 'wait' or (node.mode == 'check' and node.clock - node.mode_start >= 1.0):
                line_x = world_to_robot(node.line[None], node.pose)[0, 0]   # count moving points at the line (while standing still)
                node.moving_at_line += int(np.sum(np.abs(rel[:, 0] - line_x) < 0.15))

    # 7) after the second pendulum: drive straight on until the robot can't go any further
    if node.mode == 'goal':
        ahead = free_distance(node.occupied_points, 0.0, node.L, node.W)
        if ahead > 0.05:
            node.move_2D(max(node.min_speed, min(node.speed, ahead)), 0.0, 0.0)
        else:
            node.move_2D(0.0, 0.0, 0.0)
            node.mode = 'done'
            print("goal reached")
        return True
    if node.mode == 'done':
        node.move_2D(0.0, 0.0, 0.0)
        return True

    # 6) after the first dash: stand still until the second pendulum is spotted (none within t_wait: go for the goal)
    if node.mode == 'find':
        node.move_2D(0.0, 0.0, 0.0)
        if node.line is None:
            if node.clock - node.mode_start >= node.t_wait:
                node.mode = 'goal'
                print("no second pendulum found: driving to the goal")
            return True
        node.mode, node.mode_start, node.moving_at_line = 'check', node.clock, 0
        print("stopping 2 s to check it really moves")

    if node.line is None:
        return False
    line_x = world_to_robot(node.line[None], node.pose)[0, 0]      # how far ahead of the robot centre the line is
    to_stop = line_x - node.L - node.standoff                      # how far to drive to stop 5 cm short of it

    # 2) start once the straight way to the stopping point is clear (so the door is no longer in the way)
    if node.mode == 'navigate':
        if line_x < node.L:                                        # line behind the robot's front: forget it
            node.line = None
            print("pendulum line already behind the robot: forgetting it")
            return False
        before_line = node.occupied_points[node.occupied_points[:, 0] < line_x - 0.05]   # leave the pendulum itself out
        if free_distance(before_line, 0.0, node.L, node.W) < to_stop:
            return False
        node.mode, node.mode_start, node.moving_at_line = 'check', node.clock, 0
        print("stopping 2 s to check it really moves")

    # 2b) stand still for 2 s to make sure something really moves at the line (while the robot is moving, static
    #     things can look as if they moved)
    if node.mode == 'check':
        node.move_2D(0.0, 0.0, 0.0)
        if node.clock - node.mode_start < 2.0:
            return True
        if node.moving_at_line < 5:                                # nothing moved there: not a pendulum
            print(f"FALSE ALARM ({node.moving_at_line} moving points): resuming navigation")
            node.false_lines.append(node.line)
            node.line = None
            node.mode, node.mode_start = ('find' if node.crossed else 'navigate'), node.clock   # carry on as before
            return node.crossed > 0
        print("pendulum confirmed: moving up to the line, then to the right wall")
        node.mode = 'position'

    # 3) creep up to 5 cm before the line, then slide along it to the right wall (about 3 cm from it)
    if node.mode == 'position':
        if abs(to_stop) > 0.01:                                    # forwards, or backwards if it's already too close
            node.move_2D(np.sign(to_stop) * max(node.min_speed, min(node.speed, 2 * abs(to_stop))), 0.0, 0.0)
            return True
        right = free_distance(node.occupied_points, -np.pi / 2, node.L, node.W)
        if right > 0.04:                                           # stop about 6 cm from the wall
            node.move_2D(0.0, -max(node.min_speed, min(node.speed, right)), 0.0)
            return True
        side = node.occupied_points[(np.abs(node.occupied_points[:, 0]) <= node.L) & (node.occupied_points[:, 1] < -node.W)
                                    & (node.occupied_points[:, 1] > -node.W - 0.10)]          # the right wall, alongside the robot
        if len(side) >= 5:                                         # hold the wall's direction from now on
            node.course_heading = node.pose[2] + np.degrees(np.arctan(np.polyfit(side[:, 0], side[:, 1], 1)[0]))
            node.turning = True
        print(f"at the wall: lining up ({wrap_deg(node.course_heading - node.pose[2]):+.1f} deg), waiting for the pendulum")
        node.mode, node.mode_start, node.last_clear, node.seen, node.moving_at_line = 'wait', node.clock, node.clock, False, 0

    # 4) wait until the pendulum has been in front of the robot and left again (or hasn't come for t_wait)
    if node.mode == 'wait':
        node.move_2D(0.0, 0.0, 0.0)
        if in_front(node.occupied_points, node.L, node.W, line_x, node.depth):
            if not node.seen:
                print("pendulum in front")
            node.seen = True
            if node.clock - node.last_clear >= node.t_wait:        # something sits there for good: not a pendulum
                print("something static in front: not a pendulum, resuming navigation")
                node.false_lines.append(node.line)
                node.mode, node.line = 'navigate', None
            return True
        node.last_clear = node.clock
        dash = line_x + node.depth + node.L                        # until the robot's rear is `depth` past the line
        if node.seen or node.clock - node.mode_start >= node.t_wait:
            if free_distance(node.occupied_points, 0.0, node.L, node.W) >= dash:   # only if the whole dash is clear now
                node.mode, node.cross_start, node.cross_length = 'cross', node.pose[:2].copy(), dash
                print("GO: pendulum has passed" if node.seen else "GO: timeout")
        return True

    # 5) dash straight across at full speed until the robot's rear is past the line
    if node.mode == 'cross':
        travelled = np.hypot(node.pose[0] - node.cross_start[0], node.pose[1] - node.cross_start[1])
        if travelled < node.cross_length:
            node.move_2D(node.speed, 0.0, 0.0)
            return True
        node.line = None
        node.move_2D(0.0, 0.0, 0.0)
        if node.moving_at_line < 5:                                # nothing moved at that line: it wasn't a pendulum
            print("crossed, but nothing moved at that line: resuming navigation")
            node.mode = 'navigate'
            return True
        node.crossed += 1
        print(f"CROSSED pendulum {node.crossed}")
        node.mode, node.mode_start = ('find' if node.crossed == 1 else 'goal'), node.clock
        return True
    return False

def best_direction(visited_points,feasible_directions, current_pose,goal_direction,R,visited_weight,test_distance,lanes, w_free,w_new,probe):
    heading = np.deg2rad(current_pose[2])
    visited = np.array(visited_points).reshape(-1, 2)          # (n, 2) array of past (x, y)
    best_thetas, best_score, goal_score, proximity_score = None, -np.inf,None,None
    scores = {}
    for theta in feasible_directions:
        px = current_pose[0] +  test_distance* np.cos(heading + theta)
        py = current_pose[1] + test_distance * np.sin(heading + theta)
        goal_score = np.cos(theta - goal_direction)

        d = np.hypot(visited[:, 0] - px, visited[:, 1] - py)            # distance from the 15 cm spot to each visited point
        d = d[(d<=R)] #only consider those Rcm or closer 
        # count nearest visited point only
        proximity_score = -visited_weight * sum(R - d)/R if len(d) else 0.0
        free = lanes[theta][0]                                      # how far it can go this way (best lane), up to 1 m
        room_score = w_free * free

        # novelty, judged at the point it can actually reach this way (up to probe)
        look = min(probe, free)
        nx = current_pose[0] + look * np.cos(heading + theta)
        ny = current_pose[1] + look * np.sin(heading + theta)
        if len(visited):
            nearest = np.min(np.hypot(visited[:, 0] - nx, visited[:, 1] - ny))
            novelty_score = w_new * min(1.0, nearest / probe)      # 0 on a visited point, w_new at probe or further
        else:
            novelty_score = w_new

        score = goal_score + proximity_score +room_score +novelty_score     # computed for this theta as above
        scores[theta] = score
        if score > best_score:
            best_thetas, goal_score, proximity_score, best_score = theta,  goal_score, proximity_score,score
    
    return (best_thetas,goal_score,proximity_score,best_score,scores)
        #note:
        # we need heading hold, so robot's heading is always aligned, since real robot will drift. also, the real course axes are rotated
        # by about -90 degrees from the gazebo axes, so we need to account for that as well. 


#legacy function, before lane shift logic was added
def feasible_directions(occupied_points,L,W,test_distance):
    feasible_directions = []
    for idx in range(16):
        theta = np.deg2rad(idx*22.5)
        if not blocked(occupied_points,theta,L,W,test_distance):
            feasible_directions.append(theta)
    return feasible_directions



def free_distance(occupied_points, theta, L, W, max_d=1.0, step=0.01):
    '''How far the robot can slide along theta, up to max_d (same stepped rectangles as blocked(), all steps checked at once).'''
    x, y = occupied_points[:, 0], occupied_points[:, 1]
    outside_body = ~((np.abs(x) <= L) & (np.abs(y) <= W))      # ignore points inside the robot's own body
    c, s = np.cos(theta), np.sin(theta)
    reach = L + W + step                                         # no point further than this from the path can be hit
    along, side = x * c + y * s, -x * s + y * c                  # each point's distance along the path and to the side of it
    near = outside_body & (along > -reach) & (along < max_d + reach) & (np.abs(side) < reach)
    x, y = x[near], y[near]
    t = np.arange(step, max_d + 1e-9, step)[:, None]             # one row per step
    cx, cy = t * c, t * s                                        # robot centre after sliding t
    
    m = 0.01                                                     # side clearance
    x_lo = cx - L - (0 if c > 1e-9 else m) - (step if c < -1e-9 else 0)
    x_hi = cx + L + (0 if c < -1e-9 else m) + (step if c > 1e-9 else 0)
    y_lo = cy - W - (0 if s > 1e-9 else m) - (step if s < -1e-9 else 0)
    y_hi = cy + W + (0 if s < -1e-9 else m) + (step if s > 1e-9 else 0)
    
    hit = np.any((x >= x_lo) & (x <= x_hi) & (y >= y_lo) & (y <= y_hi), axis=1)   # True for each step that hits
    if not hit.any():
        return max_d
    return float(t[np.argmax(hit), 0] - step)                   # last distance that was still clear

def best_lane(occupied_points, theta, L, W, max_shift=0.12, shift_step=0.02, cap=0.6):
    '''Free distance along theta from the best of a few sideways-shifted start positions (positive shift = left of theta).
    A shift only counts if the robot can slide that far sideways first. Prefers no shift unless a shift clearly
    gives more room. Returns (free distance, shift).'''
    straight = free_distance(occupied_points, theta, L, W)
    if straight >= cap:                                   # already plenty of room: no need to shift
        return straight, 0.0
    room_left = free_distance(occupied_points, theta + np.pi / 2, L, W, max_d=max_shift)
    room_right = free_distance(occupied_points, theta - np.pi / 2, L, W, max_d=max_shift)
    best_free, best_shift, best_value = straight, 0.0, straight
    for k in range(1, int(round(max_shift / shift_step)) + 1):
        for s in (k * shift_step, -k * shift_step):
            if (s > 0 and s > room_left + 1e-9) or (s < 0 and -s > room_right + 1e-9):
                continue                                  # not enough room to slide that far sideways
            offset = s * np.array([-np.sin(theta), np.cos(theta)])   # shifted start, s to the left of theta
            f = free_distance(occupied_points - offset, theta, L, W, max_d=cap)
            value = f - abs(s)                            # a shift has to buy more room than it costs
            if value > best_value:
                best_free, best_shift, best_value = f, s, value
    return best_free, best_shift


def all_lanes(occupied_points, L, W):
    '''Best lane for each of the 16 directions: {theta: (free distance, sideways shift)}.'''
    lanes = {}
    for idx in range(16):
        theta = np.deg2rad(idx*22.5)
        lanes[theta] = best_lane(occupied_points, theta, L, W)
    return lanes

#legacy function, before lane shift was added
def blocked(occupied_points, theta, L, W, d, step=0.01):
    '''True if sliding d along theta would hit a LiDAR point: checks the robot's rectangle at every step along the way.'''
    #L is the half length of robot, W is half width of robot, d is how far robot moves, step is the smaller discretized step distance we test
    x, y = occupied_points[:, 0], occupied_points[:, 1]
    outside_body = ~((np.abs(x) <= L) & (np.abs(y) <= W))      # ignore points inside the robot's own body
    c, s = np.cos(theta), np.sin(theta)
    for t in np.arange(step, d + 1e-9, step):
        cx, cy = t * c, t * s                                    # robot centre after sliding t
        # grow the rectangle by step only on the side(s) it is moving towards
        x_lo, x_hi = cx - L - (step if c < -1e-9 else 0), cx + L + (step if c > 1e-9 else 0)
        y_lo, y_hi = cy - W - (step if s < -1e-9 else 0), cy + W + (step if s > 1e-9 else 0)
        if np.any(outside_body & (x >= x_lo) & (x <= x_hi) & (y >= y_lo) & (y <= y_hi)):
            return True
    return False
    

def scan_to_points(scan,angle_step, lidar_offset):
    #converts the lidar scan into an array of occupied points in robot's reference frame
    scan = np.asarray(scan, dtype=float)
    angle = np.arange(len(scan)) * angle_step
    valid = np.isfinite(scan) & (scan > 0.05)                    #check whether each lidar value is valid
    return np.column_stack((lidar_offset + np.cos(angle[valid]) * scan[valid], np.sin(angle[valid]) * scan[valid]))


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
