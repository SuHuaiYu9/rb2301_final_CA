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

def best_lane(points, theta, half_length, half_width, side_margin, shifts=(0.0, 0.02, -0.02, 0.04, -0.04, 0.06, -0.06, 0.08, -0.08, 0.10, -0.10, 0.12, -0.12), cap=0.6):
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
    seen = np.where(np.isfinite(rb) & (rb >= 0.05), rb, 0.0)            # inf / nan / too short: can't tell, so it can't
                                                                         # count as seen past (real LiDARs report missed returns as inf)
    nearest = np.minimum(np.minimum(seen[(k - 1) % len(rb)], seen[k]), seen[(k + 1) % len(rb)])
    gone = nearest > dist + margin
    return np.column_stack((wx[gone], wy[gone]))

def changed_points(prev, now, angle_step, margin=0.10):
    """Moving-obstacle test between two consecutive scans. Returns world (x, y) of points that moved:
    points of the previous scan that the new scan now sees past (something left), and points of the new scan
    that the previous scan saw past (something arrived). A static wall gives neither, however the robot moves."""
    return np.vstack((seen_past(prev, now, angle_step, margin), seen_past(now, prev, angle_step, margin)))

def along_and_left(wx, wy, heading):
    """World (x, y) -> (s, l): s = distance along the robot's forward direction, l = distance to its left.
    The robot never rotates, and each pendulum swings across the course, so a pendulum's line keeps the same s."""
    c, sn = np.cos(heading), np.sin(heading)
    return wx * c + wy * sn, -wx * sn + wy * c

def update_lines(lines, s_moved, tick, window=40, need=5, tol=0.12, support=4):
    """Finds pendulum lines from moving points: a pendulum's moving points keep the same s.
    lines: list of dicts {'s' (the pendulum's near face), 'hits' (ticks with support), 'confirmed', 'crossed'}.
    A line is confirmed once it has support (`support`+ points within tol) on `need` scan pairs within `window` ticks.
    The line is kept at the nearest its points have been (10th percentile, and it only ever moves closer), because
    a thick pendulum's sides add moving points further back, and stopping early is the safe mistake."""
    s_moved = np.sort(s_moved)
    used = np.zeros(len(s_moved), dtype=bool)
    for line in lines:
        near = np.abs(s_moved - line['s']) <= tol
        if np.count_nonzero(near) >= support:
            line['s'] = min(line['s'], float(np.percentile(s_moved[near], 10)))
            line['hits'] = [t for t in line['hits'] if tick - t <= window] + [tick]
            if not line['confirmed'] and len(line['hits']) >= need:
                line['confirmed'] = True
                print("pendulum line found")
        used |= near
    rest = s_moved[~used]                       # leftover points: new candidate lines where `support`+ sit together
    start = 0
    for i in range(1, len(rest) + 1):
        if i == len(rest) or rest[i] - rest[i - 1] > tol:
            if i - start >= support:
                lines.append({'s': float(np.percentile(rest[start:i], 10)), 'hits': [tick], 'confirmed': False, 'crossed': False})
            start = i

def line_sides(s, l, s_line, lane_lo, lane_hi, before=0.06, behind=0.20):
    """Looks along a pendulum line (scan points with s from `before` in front of s_line to `behind` past it, allowing
    for the pendulum's thickness). Returns (something inside the robot's lane [lane_lo, lane_hi]?,
    distance from the lane to the nearest point on its left, ... on its right); inf if there is none.
    A wall's distance stays the same; a pendulum's changes as it swings."""
    on = l[(s >= s_line - before) & (s <= s_line + behind)]
    inside = bool(np.any((on >= lane_lo) & (on <= lane_hi)))
    left = on[on > lane_hi] - lane_hi
    right = lane_lo - on[on < lane_lo]
    return inside, (float(left.min()) if len(left) else np.inf), (float(right.min()) if len(right) else np.inf)

def drive(node, vx, vy):
    """Every movement command goes through here. Holds the heading at node.target_heading with a proportional
    turn, and only translates (vx, vy in the robot frame) when the heading is within node.heading_tolerance of it."""
    err = wrap_deg(node.target_heading - node.pose[2])                   # degrees, positive = need to turn left
    turn = float(np.clip(node.k_heading * np.deg2rad(err), -node.max_turn, node.max_turn))
    if abs(err) > node.heading_tolerance:
        vx = vy = 0.0                                                    # too far off: turn on the spot first
    node.move_2D(vx, vy, turn)

def wall_alignment(points, max_angle=25.0, step=0.5, bin_size=0.02, max_range=1.5):
    """Angle in degrees (anticlockwise) the robot should turn so the walls it sees line up with its own x and y axes.
    Tries every angle from -max_angle to +max_angle: rotates the LiDAR points as they would look after that turn, and
    measures how sharply they pile up along x and along y (a straight wall parallel to an axis puts many points in the
    same 2 cm bin). The sharpest angle wins. Only points within max_range are used."""
    p = points[np.hypot(points[:, 0], points[:, 1]) < max_range]
    best_angle, best_sharpness = 0.0, -1
    for a in np.arange(-max_angle, max_angle + 1e-9, step):
        r = np.deg2rad(a)
        x = p[:, 0] * np.cos(r) + p[:, 1] * np.sin(r)                    # how the points look after turning by a
        y = -p[:, 0] * np.sin(r) + p[:, 1] * np.cos(r)
        sharpness = (np.sum(np.unique(np.round(x / bin_size), return_counts=True)[1] ** 2)
                     + np.sum(np.unique(np.round(y / bin_size), return_counts=True)[1] ** 2))
        if sharpness > best_sharpness:
            best_angle, best_sharpness = a, sharpness
    return float(best_angle)

def side_lane(swing_lo, swing_hi, l_robot, room_left, room_right, half_width, clearance, step=0.02):
    """Sideways offset (m, positive = left) to the nearest lane whose full width, plus `clearance` on each side, stays
    outside the pendulum's swing range [swing_lo, swing_hi] (sideways positions it has reached). Only offsets the robot
    can slide to (room_left / room_right) are tried. None if there is no such lane."""
    best = None
    for off in np.arange(-min(room_right, 1.0), min(room_left, 1.0) + 1e-9, step):
        lo = l_robot + off - half_width - clearance
        hi = l_robot + off + half_width + clearance
        if (hi < swing_lo or lo > swing_hi) and (best is None or abs(off) < abs(best)):
            best = float(off)
    return best

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
        self.scan_history = []          # the last few scans; lines are found by comparing with the one 0.3 s back
        self.compare_back = 3           # scans back (0.3 s at 10 Hz): slow pendulums move further, so they show up better
        self.scan_used = None           # the scan array used last tick, to notice when a new one arrives
        self.lines = []                 # pendulum lines found so far (see update_lines)
        self.pendulums_seen = False     # at least one pendulum line found
        self.standoff = 0.08            # wait with the robot's front this far before a pendulum line
        self.line_depth = 0.20          # a pendulum line is crossed once the robot's rear is this far past it (pendulum thickness + margin)
        self.blocked_ticks = 0          # ticks a crossing has been physically blocked
        self.lane_margin = 0.05         # the robot's lane: its width plus this much on each side
        self.waiting_at = None          # s of the line the robot is waiting at
        self.waiting = False
        self.prev_sides = None          # (left, right) distances from the lane on that line in the previous scan
        self.prev_sides_tick = 0
        self.in_lane_tick = -1000       # last tick a pendulum overlapped the robot's lane
        self.moving_tick = 0            # last tick anything on the line moved
        self.go = False                 # cleared to cross the line
        # fix 1: go around a small swing
        self.swing_samples = []         # (tick, sideways position) of the pendulum's moving points on the line being watched
        self.swing_window = 100         # use the last 5 s of them (more than one swing)
        self.watch_ticks = 100          # watch a line for 5 s (2+ swings of a real pendulum) before trusting its swing range
        self.swing_clearance = 0.10     # keep the robot's sides this far outside the swing range
        self.watch_start = 0
        self.side_target = None         # sideways position (l) of a lane beside the swing, while sliding to it
        self.side_tried = False         # a lane beside the swing is tried only once per line (a slow swing can fool it)
        self.crossing = None            # s of the line being crossed
        self.goal_ticks = 0             # ticks in a row that forward has been blocked after the pendulums
        self.done = False
        self.tick = 0
        # heading control (the robot should never rotate: hold the heading it starts the run with)
        self.k_heading = 1.5            # turn rate (rad/s) per radian of heading error
        self.max_turn = 0.5             # rad/s
        self.heading_tolerance = 10.0   # degrees: further off than this, stop moving and only turn
        self.target_heading = None      # degrees, in the pose's frame (set at start-up)
        self.heading_fix = 0.0          # degrees added to the pose heading for world-frame maths (see the forward check)
        # start-up checks, real robot only: spin check -> square up with the walls -> forward check -> run
        self.startup = 'run' if is_simulation else 'spin'
        self.startup_ticks = 0
        self.spin_rate = 0.4             # rad/s for the spin check (stays well inside the template's 0.6 limit)
        self.startup_ref = None
        self.settled = 0
        self.stopped = False            # a start-up check failed: never drive
        # stale data: stop if pose or LiDAR updates stop arriving
        self.pose_seen = None
        self.scan_seen = None
        self.pose_stale = 0
        self.scan_stale = 0

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
        if self.done or self.stopped:
            self.move_2D(0.0, 0.0, 0.0)
            return

        # stop if the pose (Optitrack / odometry) or the LiDAR stops updating for 0.5 s; carry on when they come back
        self.pose_stale = self.pose_stale + 1 if self.pose is self.pose_seen else 0
        self.scan_stale = self.scan_stale + 1 if self.last_scan is self.scan_seen else 0
        self.pose_seen, self.scan_seen = self.pose, self.last_scan
        if self.pose_stale > 10 or self.scan_stale > 10:
            if self.tick % 20 == 0:
                print("no new pose or LiDAR data: stopped")
            self.move_2D(0.0, 0.0, 0.0)
            return
        if self.target_heading is None:
            self.target_heading = float(self.pose[2])                  # hold the heading the robot starts with

        heading = np.deg2rad(self.pose[2] + self.heading_fix)         # the robot's true facing direction (see the forward check)
        cx = self.pose[0] + self.centre_ahead * np.cos(heading)       # robot centre in the world
        cy = self.pose[1] + self.centre_ahead * np.sin(heading)
        step = self.angle_step if self.angle_step else 2 * np.pi / len(self.last_scan)
        points = scan_to_points(self.last_scan, step, self.lidar_ahead, self.half_length, self.half_width)

        # 0) real robot start-up checks, before anything else
        if self.startup != 'run':
            self.startup_ticks += 1
            if self.startup == 'spin':
                # turn left (up to 2 s): the heading must go up. If it goes down, heading control would spin the robot;
                # if it doesn't change, the robot isn't turning or the pose isn't following it.
                if self.startup_ref is None:
                    self.startup_ref = float(self.pose[2])
                    print(f"spin check: turning left, start heading {self.startup_ref:+.1f} deg")
                change = wrap_deg(self.pose[2] - self.startup_ref)
                if change <= -3.0:
                    print(f"START-UP CHECK FAILED: told to turn left, but the heading went DOWN by {-change:.1f} deg. "
                          f"The heading's sign is reversed (check the Optitrack setup). Not driving.")
                    self.stopped = True
                    self.move_2D(0.0, 0.0, 0.0)
                    return
                if change < 3.0:
                    if self.startup_ticks <= 40:
                        self.move_2D(0.0, 0.0, self.spin_rate)
                        return
                    print(f"START-UP CHECK FAILED: told to turn left at {self.spin_rate} rad/s for 2 s, but the heading changed by only "
                          f"{change:+.1f} deg. Either the robot didn't turn (is base control running? is it receiving cmd_vel?) "
                          f"or the pose isn't following it (is Optitrack tracking this robot?). Not driving.")
                    self.stopped = True
                    self.move_2D(0.0, 0.0, 0.0)
                    return
                print(f"spin check OK: heading went up {change:.1f} deg after {self.startup_ticks * 0.05:.2f} s")
                self.startup, self.startup_ticks, self.startup_ref = 'align', 0, None
            if self.startup == 'align':
                # square up with the corridor walls, then hold that heading for the whole run
                if self.startup_ref is None:
                    turn_by = wall_alignment(points)
                    self.target_heading = wrap_deg(self.pose[2] + turn_by)
                    self.startup_ref = True
                    print(f"squaring up with the walls: turning {turn_by:+.1f} deg")
                self.settled = self.settled + 1 if abs(wrap_deg(self.target_heading - self.pose[2])) < 1.0 else 0
                if self.settled < 10 and self.startup_ticks < 200:
                    drive(self, 0.0, 0.0)
                    return
                self.startup, self.startup_ticks, self.startup_ref = 'check', 0, None
            if self.startup == 'check':
                # slide 10 cm in the direction with the most room: the direction the robot actually moves, minus the
                # direction it was told to move, is its true heading. If that's far from the pose heading (a wrong
                # Optitrack heading offset), use the measured heading for all world-frame maths.
                if self.startup_ref is None:
                    room = {k: free_distance(points, k * np.pi / 8, self.half_length, self.half_width, self.side_margin) for k in range(16)}
                    k_best = max(room, key=room.get)
                    if room[k_best] < 0.35:
                        print("START-UP CHECK FAILED: no direction with room to check the heading. Not driving.")
                        self.stopped = True
                        self.move_2D(0.0, 0.0, 0.0)
                        return
                    self.startup_ref = (float(self.pose[0]), float(self.pose[1]), k_best * np.pi / 8)
                dx, dy = self.pose[0] - self.startup_ref[0], self.pose[1] - self.startup_ref[1]
                theta = self.startup_ref[2]
                if np.hypot(dx, dy) < 0.10 and self.startup_ticks < 60:
                    drive(self, 0.1 * np.cos(theta), 0.1 * np.sin(theta))
                    return
                if np.hypot(dx, dy) < 0.05:
                    print("START-UP CHECK FAILED: told to move but the pose moved less than 5 cm. Not driving.")
                    self.stopped = True
                    self.move_2D(0.0, 0.0, 0.0)
                    return
                diff = wrap_deg(np.rad2deg(np.arctan2(dy, dx) - theta) - self.pose[2])
                if abs(diff) > 15.0:
                    self.heading_fix = diff
                    print(f"WARNING: the robot's true heading is {diff:+.0f} deg from the pose heading. Using the measured "
                          f"heading instead (check heading_offset_deg in optitrack_variables.config)")
                else:
                    print(f"heading check OK: true heading {diff:+.1f} deg from the pose heading")
                self.startup = 'run'
            self.move_2D(0.0, 0.0, 0.0)
            return

        # remember where the robot has been, and for how long (for the visited penalty)
        if self.tick % self.record_every == 0:
            self.visited.append((cx, cy))

        # where the robot is along the course (s) and across it (l); the next pendulum line ahead of it
        s_robot, l_robot = along_and_left(cx, cy, heading)
        front = s_robot + self.half_length
        for line in self.lines:
            if line['confirmed'] and not line['crossed'] and s_robot - self.half_length > line['s'] + self.line_depth:
                line['crossed'] = True                                 # the whole robot is past it
        ahead = [line for line in self.lines if line['confirmed'] and not line['crossed'] and line['s'] > front - 0.03]
        line = min(ahead, key=lambda L: L['s']) if ahead else None
        # possible lines too (movement seen there in the last 2 s, not confirmed yet): don't drive into them
        maybe = [L for L in self.lines if not L['crossed'] and L['s'] > front - 0.03 and (L['confirmed'] or self.tick - L['hits'][-1] <= 40)]
        stop_at = min(L['s'] for L in maybe) if maybe else None   # s of the nearest line the robot must stop short of
        at_line = line is not None and line['s'] - front <= self.standoff + 0.02
        if at_line and self.waiting_at is not line:                    # just arrived at this line: start watching it
            self.waiting_at = line
            self.prev_sides, self.in_lane_tick, self.moving_tick, self.go = None, -1000, self.tick, False
            self.swing_samples, self.watch_start, self.side_target, self.side_tried = [], self.tick, None, False
        if not at_line:
            self.waiting_at = None

        # 1) each time a new scan arrives: find pendulum lines, and watch the pendulum on the line the robot is at
        if self.last_scan is not self.scan_used:
            self.scan_used = self.last_scan
            lx = cx + self.lidar_ahead * np.cos(heading)               # LiDAR position in the world
            ly = cy + self.lidar_ahead * np.sin(heading)
            now = (lx, ly, heading, self.last_scan)
            self.scan_history = (self.scan_history + [now])[-(self.compare_back + 1):]
            if len(self.scan_history) > self.compare_back:
                wxy = changed_points(self.scan_history[0], now, step)
                s_moved, l_moved = along_and_left(wxy[:, 0], wxy[:, 1], heading)
                keep = (s_moved > s_robot + 0.15) & (s_moved < s_robot + 2.5) & (np.abs(l_moved - l_robot) < 1.2)
                update_lines(self.lines, s_moved[keep], self.tick)
                if at_line:                                            # moving points on the line being watched: the swing range
                    on = (s_moved >= line['s'] - 0.06) & (s_moved <= line['s'] + self.line_depth)
                    self.swing_samples += [(self.tick, float(v)) for v in l_moved[on]]
                    self.swing_samples = [(t, v) for t, v in self.swing_samples if self.tick - t <= self.swing_window]
                if not self.pendulums_seen and any(L['confirmed'] for L in self.lines):
                    self.pendulums_seen = True
            self.prev_scan = now

            if at_line:
                lane_lo = l_robot - self.half_width - self.lane_margin
                lane_hi = l_robot + self.half_width + self.lane_margin
                in_lane, d_left, d_right = line_sides(s_robot + points[:, 0], l_robot + points[:, 1], line['s'], lane_lo, lane_hi)
                v_left = v_right = 0.0                                 # how fast the gap to the lane grows on each side
                dt = (self.tick - self.prev_sides_tick) * 0.05
                if self.prev_sides is not None and dt > 0:
                    if np.isfinite(d_left) and np.isfinite(self.prev_sides[0]):
                        v_left = (d_left - self.prev_sides[0]) / dt
                    if np.isfinite(d_right) and np.isfinite(self.prev_sides[1]):
                        v_right = (d_right - self.prev_sides[1]) / dt
                approaching = (v_left < -0.05 and d_left < 1.0) or (v_right < -0.05 and d_right < 1.0)
                moving = abs(v_left) > 0.05 or abs(v_right) > 0.05
                if line['hits'] and self.tick - line['hits'][-1] <= 20:
                    moving = True                                      # the motion test saw movement on this line in the last 1 s
                if in_lane:
                    self.in_lane_tick = self.tick
                if moving:
                    self.moving_tick = self.tick
                if self.tick - self.watch_start >= 10 and line['hits'] and line['hits'][-1] == self.tick:
                    line['real'] = True                                # it moved while the robot stood still: a real pendulum
                room_left = free_distance(points, np.pi / 2, self.half_length, self.half_width, self.side_margin)
                room_right = free_distance(points, -np.pi / 2, self.half_length, self.half_width, self.side_margin)
                leaving_far_side = v_left > 0.05 if room_left > room_right else v_right > 0.05
                travel = line['s'] + self.line_depth + 2 * self.half_length - front   # until the rear is past the line

                # fix 1: after watching for 3 s, look for a lane beside the swing, with room to cross straight from it
                swing = [v for t, v in self.swing_samples]
                if not self.side_tried and self.tick - self.watch_start >= self.watch_ticks and len(swing) >= 20:
                    off = side_lane(min(swing), max(swing), l_robot, room_left, room_right, self.half_width, self.swing_clearance)
                    if off is not None and free_distance(points, 0.0, self.half_length, self.half_width, self.side_margin, shift=off) >= travel:
                        self.side_target, self.side_tried = l_robot + off, True
                        print(f"pendulum swings between {min(swing):+.2f} and {max(swing):+.2f}: sliding {off:+.2f} m to cross beside it")
                if self.side_target is not None and swing:            # the swing grew into the chosen lane: forget it
                    if not (self.side_target + self.half_width + self.swing_clearance < min(swing)
                            or self.side_target - self.half_width - self.swing_clearance > max(swing)):
                        self.side_target = None
                        print("the swing grew: lane beside it given up")

                if self.side_target is None and not in_lane and not approaching:
                    if leaving_far_side and self.tick - self.in_lane_tick <= 8:
                        self.go = "the pendulum just left the lane"    # heading for the far side: longest until it's back
                    elif self.tick - self.moving_tick >= 20:
                        self.go = "nothing on the line moved for 1 s"
                elif in_lane and self.tick - self.moving_tick >= 40:
                    line['crossed'] = line['ignored'] = True           # something static has sat there for 2 s: not a pendulum
                    print("not a pendulum line: ignoring it")
                self.prev_sides, self.prev_sides_tick = (d_left, d_right), self.tick

        # 2) crossing a line: straight across at full speed, and don't give up halfway
        if self.crossing is not None:
            if s_robot - self.half_length > self.crossing + self.line_depth:
                self.crossing = None                                   # the whole robot is across
            else:
                free_ahead = free_distance(points, 0.0, self.half_length, self.half_width, self.side_margin)
                self.blocked_ticks = self.blocked_ticks + 1 if free_ahead < 0.02 else 0
                if self.blocked_ticks >= 60:
                    self.crossing = None                               # blocked for 3 s: something static, give up
                    print("crossing blocked: giving up")
                else:
                    drive(self, self.speed if free_ahead >= 0.02 else 0.0, 0.0)
                    return

        # 3) at a line: wait at its edge until cleared to cross
        self.waiting = at_line and line is self.waiting_at and not line['crossed']
        if self.waiting:
            if self.side_target is not None:                           # fix 1: slide sideways (still short of the line) first
                off = self.side_target - l_robot
                if abs(off) > 0.01:
                    drive(self, 0.0, float(np.clip(2.0 * off, -0.15, 0.15)))
                    return
                self.side_target = None
                self.go = "beside the pendulum's swing"
            if self.go:
                print("crossing: " + self.go)
                self.crossing, self.go, self.waiting, self.blocked_ticks = line['s'], False, False, 0
                drive(self, self.speed, 0.0)
            else:
                drive(self, 0.0, 0.0)
            return

        # 4) past the pendulums (a line has been crossed and no line is ahead): drive straight forward until the robot
        #    can't move forward any more, for 1 s. That's the goal.
        if not ahead and any(L['confirmed'] and L['crossed'] and L.get('real') for L in self.lines):
            free_ahead = free_distance(points, 0.0, self.half_length, self.half_width, self.side_margin)
            wall = all(free_distance(points, a, self.half_length, self.half_width, self.side_margin) < 0.25 for a in (np.pi / 4, -np.pi / 4))
            if free_ahead < self.free_min:
                self.goal_ticks = self.goal_ticks + 1 if wall else 0      # a wall, not a pendulum, in front
                drive(self, 0.0, 0.0)
                if self.goal_ticks >= 20:
                    self.done = True
                    print("goal reached")
            else:
                self.goal_ticks = 0
                speed = min(self.speed, free_ahead)
                if stop_at is not None:                                # a possible pendulum line ahead: stop short of it
                    speed = min(speed, max(0.0, 3.0 * (stop_at - front - self.standoff)))
                drive(self, speed, 0.0)
            return

        # 5) otherwise, score every direction the robot can slide in:
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
            drive(self, 0.0, 0.0)                                      # boxed in: wait
            return
        moved = np.inf if self.decided_at is None else np.hypot(cx - self.decided_at[0], cy - self.decided_at[1])
        if self.direction_k not in scores or moved >= self.commit:     # blocked, or gone far enough: decide again
            best = max(scores, key=lambda k: scores[k][0])
            if self.direction_k in scores and scores[self.direction_k][0] >= scores[best][0] - self.hysteresis:
                best = self.direction_k                                # not clearly better: keep going the same way
            self.direction_k = best
            self.decided_at = (cx, cy)
        best = self.direction_k

        # 6) move that way (robot frame), sliding sideways towards the best lane; slower when close to something
        theta = best * 2 * np.pi / self.n_directions
        shift = scores[best][2]
        free_now = free_distance(points, theta, self.half_length, self.half_width, self.side_margin)   # unshifted
        along_speed = min(self.speed, max(0.0, free_now))              # don't run into anything while lining up
        side_speed = np.clip(2.0 * shift, -0.15, 0.15)                 # positive = to the left of theta
        vx = along_speed * np.cos(theta) - side_speed * np.sin(theta)
        vy = along_speed * np.sin(theta) + side_speed * np.cos(theta)
        if stop_at is not None:                                        # a (possible) pendulum line ahead: stop at its edge
            vx = min(vx, max(0.0, 3.0 * (stop_at - front - self.standoff)))
        total = np.hypot(vx, vy)
        if total > self.speed:                                         # keep the total speed at most 0.3 m/s
            vx, vy = vx * self.speed / total, vy * self.speed / total
        drive(self, vx, vy)
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