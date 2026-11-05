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

# ---------------------------------------------------------------------------------------------------------
# Reactive navigation helpers (no occupancy grid). Everything here is in the robot's own frame:
# x = forward, y = left, measured from the robot's centre. The robot never rotates, so "forward" always
# points the way it faced at the start.
# ---------------------------------------------------------------------------------------------------------

def scan_to_points(scan, angle_step, lidar_ahead, half_length, half_width):
    '''LiDAR ranges -> (n, 2) array of (x, y) points in the robot frame, measured from the robot's centre.
    Reading i points i * angle_step radians anticlockwise from forward. Readings that are not finite or are
    shorter than 0.05 m are dropped, and so are points inside the robot's own footprint (they can't be obstacles).'''
    ranges = np.asarray(scan, dtype=float)
    ok = np.isfinite(ranges) & (ranges >= 0.05)
    angles = np.arange(len(ranges))[ok] * angle_step
    x = lidar_ahead + ranges[ok] * np.cos(angles)
    y = ranges[ok] * np.sin(angles)
    inside_robot = (np.abs(x) <= half_length) & (np.abs(y) <= half_width)
    return np.column_stack((x[~inside_robot], y[~inside_robot]))

def strip_coords(points, theta, half_length, half_width):
    '''For the robot sliding along direction theta (radians, anticlockwise from forward):
    along: each point's distance ahead along theta;  side: each point's distance to the side of that line;
    reach: how far the robot's edge sticks out ahead of its centre along theta;
    half_strip: half the width of the strip the robot sweeps (its outline projected sideways).'''
    ux, uy = np.cos(theta), np.sin(theta)
    along = points[:, 0] * ux + points[:, 1] * uy
    side = -points[:, 0] * uy + points[:, 1] * ux
    reach = abs(half_length * ux) + abs(half_width * uy)
    half_strip = abs(half_length * uy) + abs(half_width * ux)
    return along, side, reach, half_strip

def free_distance(points, theta, half_length, half_width, side_margin, shift=0.0, max_range=3.0):
    '''How far the robot can slide along theta before a LiDAR point enters the strip it sweeps
    (its own width plus side_margin on each side). With shift, the strip is moved sideways by that much first
    (positive = to the left of theta). max_range if nothing is in the way.'''
    along, side, reach, half_strip = strip_coords(points, theta, half_length, half_width)
    in_strip = (np.abs(side - shift) <= half_strip + side_margin) & (along > 0)
    if not np.any(in_strip):
        return max_range
    return float(np.min(along[in_strip]) - reach)

def best_lane(points, theta, half_length, half_width, side_margin, shifts=(0.0, 0.04, -0.04, 0.08, -0.08, 0.12, -0.12), cap=0.6):
    '''Lines the robot up with gaps: tries sliding along theta from a few sideways-shifted positions
    (positive = left of theta). A shift only counts if the robot has room to slide that far sideways first.
    Prefers no shift unless a shift gives clearly more room. Returns (free distance, shift).'''
    room_left = free_distance(points, theta + np.pi / 2, half_length, half_width, side_margin)
    room_right = free_distance(points, theta - np.pi / 2, half_length, half_width, side_margin)
    best_free, best_shift, best_value = 0.0, 0.0, -np.inf
    for s in shifts:
        if (s > 0 and s > room_left) or (s < 0 and -s > room_right):
            continue
        f = free_distance(points, theta, half_length, half_width, side_margin, shift=s)
        value = min(f, cap) - abs(s)
        if value > best_value:
            best_free, best_shift, best_value = f, s, value
    return best_free, best_shift

def novelty(visited, px, py, probe):
    """1 if the world point (px, py) is at least `probe` m from every recorded position, 0 if it is on one,
    and in between linearly."""
    if len(visited) == 0:
        return 1.0
    d = np.min(np.hypot(visited[:, 0] - px, visited[:, 1] - py))
    return min(1.0, d / probe)

def visits_near(visited, px, py, radius):
    """How many of the robot's recorded positions are within `radius` of the world point (px, py).
    Positions are recorded every 0.25 s, so this grows with the time spent there."""
    if len(visited) == 0:
        return 0
    return int(np.count_nonzero(np.hypot(visited[:, 0] - px, visited[:, 1] - py) <= radius))

def seen_past(a, b, angle_step, margin):
    """Points of scan a (world coordinates) that scan b, looking towards them (the reading pointing at each one
    and its two neighbours), sees more than `margin` past. a, b = (lidar_x, lidar_y, heading_rad, ranges)."""
    xa, ya, ha, ra = a
    xb, yb, hb, rb = b
    ra = np.asarray(ra, dtype=float)
    rb = np.asarray(rb, dtype=float)
    ok = np.isfinite(ra) & (ra >= 0.05)
    ang = ha + np.arange(len(ra))[ok] * angle_step
    wx = xa + ra[ok] * np.cos(ang)
    wy = ya + ra[ok] * np.sin(ang)
    dist = np.hypot(wx - xb, wy - yb)                                   # each point's distance from scan b's LiDAR
    bearing = np.mod(np.arctan2(wy - yb, wx - xb) - hb, 2 * np.pi)       # and its direction in scan b
    k = np.round(bearing / angle_step).astype(int) % len(rb)             # scan b's reading pointing at it
    seen = np.where(np.isnan(rb) | (rb < 0.05), 0.0, rb)                 # invalid reading: can't see past anything
                                                                         # (inf = nothing in range, so it does see past)
    nearest = np.minimum(np.minimum(seen[(k - 1) % len(rb)], seen[k]), seen[(k + 1) % len(rb)])
    gone = nearest > dist + margin
    return np.column_stack((wx[gone], wy[gone]))

def changed_points(prev, now, angle_step, margin=0.10):
    """Moving-obstacle test between two consecutive scans. Returns world (x, y) of points that moved:
    points of the previous scan that the new scan now sees past (something left), and points of the new scan
    that the previous scan saw past (something arrived). A static wall gives neither, however the robot moves."""
    return np.vstack((seen_past(prev, now, angle_step, margin), seen_past(now, prev, angle_step, margin)))

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

        # ---------------- reactive navigation (no occupancy grid) ----------------
        self.last_scan = None           # set by sub_scan_callback
        if is_simulation:
            self.centre_ahead = 0.0725      # robot centre is 7.25 cm ahead of the Gazebo pose point
            self.lidar_ahead = 0.114 - 0.0725   # LiDAR ahead of the robot centre
            self.half_length = 0.11         # robot 0.22 m long (x) ...
            self.half_width = 0.12          # ... and 0.24 m wide (y)
            self.angle_step = 6.28 / 359    # Gazebo: 360 readings from -3.14 to 3.14
        else:
            self.centre_ahead = 0.0         # assumes the Optitrack pose point is the robot centre (as in R1)
            self.lidar_ahead = 0.045        # LiDAR 4.5 cm ahead of the centre (R1)
            self.half_length = 0.115        # real robot about 0.23 m long ...
            self.half_width = 0.10          # ... and 0.20 m wide
            self.angle_step = None          # real: 2*pi / number of readings, worked out from each scan
        self.speed = 0.3                # m/s
        self.n_directions = 16          # candidate directions, every 22.5 degrees
        self.goal_direction = np.deg2rad(-45)   # the goal's general direction: forward-right
        self.free_min = 0.10            # a direction is blocked if the robot can't slide at least this far
        self.side_margin = 0.03         # extra clearance on each side of the strip the robot sweeps
        self.probe = 0.4                # look this far along each direction (or less, if it's blocked sooner)
        self.w_free = 1.0               # bonus for room: w_free * (free distance, up to 1 m)
        self.w_new = 1.5                # bonus for somewhere new: w_new * novelty of the point looked at
        self.visit_radius = 0.25        # time-spent penalty: recorded positions within 0.25 m of that point ...
        self.w_visit = 0.1              # ... cost 0.1 each (a single pass leaves about 7, rocking on the spot many more)
        self.record_every = 5           # record the robot's position every 5 ticks (0.25 s), so time spent counts
        self.commit = 0.15              # keep a chosen direction for 0.15 m (or until it's blocked) before deciding again
        self.decided_at = None          # robot centre (world) when the current direction was chosen
        self.hysteresis = 0.2           # keep the current direction unless another beats it by this much
        self.visited = []               # world positions of the robot centre, every 0.25 s
        self.direction_k = None         # current direction (index k: k * 22.5 deg anticlockwise from forward)
        # moving obstacles (pendulums)
        self.prev_scan = None           # (lidar_x, lidar_y, heading, ranges) of the previous scan
        self.scan_used = None           # the scan array used last tick, to notice when a new one arrives
        self.seen_min = 5               # this many moved points ahead on 2 scans in a row = the pendulums are moving
        self.prev_ahead_count = 0       # moved points ahead in the previous scan pair
        self.wait_min = 3               # this many moved points in the robot's path = wait
        self.wait_ahead = 0.5           # path region: up to 0.5 m beyond the robot's edge ...
        self.wait_side = 0.3            # ... and 0.3 m wider than its strip on each side
        self.wait_ticks = 10            # keep waiting 10 ticks (0.5 s) after the last movement in the path
        self.wait_until = 0
        self.pendulums_seen = False     # pendulums seen moving ahead (just for printing)
        self.pendulums_behind = False   # pendulums seen moving behind the robot: it's past them
        self.prev_behind_count = 0
        self.goal_ticks = 0             # ticks in a row that forward has been blocked after the pendulums
        self.done = False
        self.tick = 0

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
        self.tick += 1
        if self.last_scan is None:
            return
        if self.done:
            self.move_2D(0.0, 0.0, 0.0)
            return

        heading = np.deg2rad(self.pose[2])
        cx = self.pose[0] + self.centre_ahead * np.cos(heading)       # robot centre in the world
        cy = self.pose[1] + self.centre_ahead * np.sin(heading)
        step = self.angle_step if self.angle_step else 2 * np.pi / len(self.last_scan)
        points = scan_to_points(self.last_scan, step, self.lidar_ahead, self.half_length, self.half_width)

        # remember where the robot has been, and for how long (for the visited penalty)
        if self.tick % self.record_every == 0:
            self.visited.append((cx, cy))

        # 1) moving obstacles: each time a new scan arrives, compare it with the previous one
        if self.last_scan is not self.scan_used:
            self.scan_used = self.last_scan
            lx = cx + self.lidar_ahead * np.cos(heading)               # LiDAR position in the world
            ly = cy + self.lidar_ahead * np.sin(heading)
            now = (lx, ly, heading, self.last_scan)
            if self.prev_scan is not None:
                wxy = changed_points(self.prev_scan, now, step)
                dx, dy = wxy[:, 0] - cx, wxy[:, 1] - cy                     # moved points in the robot frame
                moved = np.column_stack((dx * np.cos(heading) + dy * np.sin(heading), -dx * np.sin(heading) + dy * np.cos(heading)))
                # anywhere up to 1 m ahead and 0.5 m to each side: the pendulums have been seen moving
                ahead = np.count_nonzero((moved[:, 0] >= 0) & (moved[:, 0] <= 1.0) & (np.abs(moved[:, 1]) <= 0.5))
                if ahead >= self.seen_min and self.prev_ahead_count >= self.seen_min and not self.pendulums_seen:
                    self.pendulums_seen = True
                    print("pendulums seen moving")
                self.prev_ahead_count = ahead
                # up to 2 m behind the robot: it has gone past the pendulums
                behind = np.count_nonzero((moved[:, 0] < -self.half_length) & (moved[:, 0] >= -2.0) & (np.abs(moved[:, 1]) <= 1.0))
                if behind >= self.seen_min and self.prev_behind_count >= self.seen_min and not self.pendulums_behind:
                    self.pendulums_behind = True
                    print("pendulums seen moving behind: past them")
                self.prev_behind_count = behind
                # in the robot's path (the strip along its current direction, just ahead of it): wait
                theta = 0.0 if self.direction_k is None else self.direction_k * 2 * np.pi / self.n_directions
                along, side, reach, half_strip = strip_coords(moved, theta, self.half_length, self.half_width)
                in_path = np.count_nonzero((along >= reach) & (along <= reach + self.wait_ahead)
                                           & (np.abs(side) <= half_strip + self.wait_side))
                if in_path >= self.wait_min:
                    self.wait_until = self.tick + self.wait_ticks
            self.prev_scan = now

        # 2) something is moving in the robot's path: stop and wait
        if self.tick < self.wait_until:
            self.goal_ticks = 0
            self.move_2D(0.0, 0.0, 0.0)
            return

        # 3) goal: the pendulums are behind the robot, and it can't move forward any more (for 1 s)
        if self.pendulums_behind and free_distance(points, 0.0, self.half_length, self.half_width, self.side_margin) < self.free_min:
            self.goal_ticks += 1
            self.move_2D(0.0, 0.0, 0.0)
            if self.goal_ticks >= 20:
                self.done = True
                print("goal reached")
            return
        self.goal_ticks = 0

        # 4) score every direction the robot can slide in:
        #    goal bias + room to move + somewhere new - time already spent there
        visited = np.array(self.visited)
        scores = {}                     # k -> (score, free distance, sideways shift)
        for k in range(self.n_directions):
            theta = k * 2 * np.pi / self.n_directions                  # anticlockwise from forward
            free, shift = best_lane(points, theta, self.half_length, self.half_width, self.side_margin)
            if free < self.free_min:
                continue                                               # blocked, even lined up with a gap
            goal = np.cos(theta - self.goal_direction)
            look = min(self.probe, free)                               # as far as it can actually get that way, up to 0.4 m
            px = cx + look * np.cos(heading + theta)                   # that point, in the world
            py = cy + look * np.sin(heading + theta)
            score = (goal + self.w_free * min(free, 1.0) + self.w_new * novelty(visited, px, py, self.probe)
                     - self.w_visit * visits_near(visited, px, py, self.visit_radius))
            scores[k] = (score, free, shift)
        if not scores:
            self.move_2D(0.0, 0.0, 0.0)                                # boxed in: wait
            return
        moved = np.inf if self.decided_at is None else np.hypot(cx - self.decided_at[0], cy - self.decided_at[1])
        if self.direction_k not in scores or moved >= self.commit:     # blocked, or gone far enough: decide again
            best = max(scores, key=lambda k: scores[k][0])
            if self.direction_k in scores and scores[self.direction_k][0] >= scores[best][0] - self.hysteresis:
                best = self.direction_k                                # not clearly better: keep going the same way
            self.direction_k = best
            self.decided_at = (cx, cy)
        best = self.direction_k

        # 5) move that way (robot frame), sliding sideways towards the best lane; slower when close to something
        theta = best * 2 * np.pi / self.n_directions
        shift = scores[best][2]
        free_now = free_distance(points, theta, self.half_length, self.half_width, self.side_margin)   # unshifted
        along_speed = min(self.speed, max(0.0, free_now))              # don't run into anything while lining up
        side_speed = np.clip(2.0 * shift, -0.15, 0.15)                 # positive = to the left of theta
        vx = along_speed * np.cos(theta) - side_speed * np.sin(theta)
        vy = along_speed * np.sin(theta) + side_speed * np.cos(theta)
        total = np.hypot(vx, vy)
        if total > self.speed:                                         # keep the total speed at most 0.3 m/s
            vx, vy = vx * self.speed / total, vy * self.speed / total
        self.move_2D(vx, vy, 0.0)
        ###### INSERT CODE HERE ######
                

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