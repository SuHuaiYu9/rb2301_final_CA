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
        self.L = 0.115  #robot's half length
        self.W = 0.11  #robot's half width
        self.lidar_offset = 0.045    
        self.feasible_directions  = []

        self.goal_direction = np.deg2rad(315) #in degrees
        self.R = 0.2 #the maximum radius that counts towards proximity score
        self.visited_weight = 5 #weight for proximity in calculating score for which direction to travel in

        self.visited_points = []
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
        if self.last_scan is None:
            return 
        self.angle_step =  2* np.pi/len(self.last_scan) 

        #first transform the lidar scan into occupied points in robot's frame
        self.occupied_points  = scan_to_points(self.last_scan,self.angle_step,self.lidar_offset)

        #then we check which of the 16 directions if feasible by seeing if we move in each direction by 15cm, would we hit anyt
        self.feasible_directions = feasible_directions(self.occupied_points,self.L,self.W) #directions given in radians CCW from robot direction
        
        self.tick += 1
        if self.tick % 10 == 0:
            self.visited_points.append((self.pose[0], self.pose[1]))
        self.selected_direction = best_direction(self.visited_points,self.feasible_directions,self.pose,self.goal_direction,self.R,self.visited_weight)
                
            


        ###### INSERT CODE HERE ######

def best_direction(visited_points,feasible_directions, current_pose,goal_direction,R,visited_weight):
    heading = np.deg2rad(current_pose[2])
    visited = np.array(visited_points).reshape(-1, 2)          # (n, 2) array of past (x, y)
    best_thetas, best_score = None, -np.inf
    for theta in feasible_directions:
        px = current_pose[0] + 0.15 * np.cos(heading + theta)
        py = current_pose[1] + 0.15 * np.sin(heading + theta)
        goal_score = np.cos(theta - goal_direction)

        d = np.hypot(visited[:, 0] - px, visited[:, 1] - py)            # distance from the 15 cm spot to each visited point
        # count nearest visited point only
        proximity_score = -visited_weight * max(0.0, R - d.min()) if len(d) else 0.0

        score = goal_score + proximity_score      # computed for this theta as above
        if score > best_score:
            best_thetas, best_score = theta, score
    return best_thetas
        #note:
        # we need heading hold, so robot's heading is always aligned, since real robot will drift. also, the real course axes are rotated
        # by about -90 degrees from the gazebo axes, so we need to account for that as well. 



def feasible_directions(occupied_points,L,W):
    feasible_directions = []
    for idx in range(16):
        theta = np.deg2rad(idx*22.5)
        if not blocked(occupied_points,theta,L,W):
            feasible_directions.append(theta)
    return feasible_directions


def blocked(occupied_points, theta, L, W, d=0.15, step=0.01):
    '''True if sliding d along theta would hit a LiDAR point: checks the robot's rectangle at every step along the way.'''
    #L is the half length of robot, W is half width of robot, d is how far robot moves, step is the smaller discretized step distance we test
    x, y = occupied_points[:, 0], occupied_points[:, 1]
    outside_body = ~((np.abs(x) <= L) & (np.abs(y) <= W))      # ignore points inside the robot's own body
    for t in np.arange(step, d + 1e-9, step):
        cx, cy = t * np.cos(theta), t * np.sin(theta)           # robot centre after sliding t
        if np.any(outside_body & (np.abs(x - cx) <= L+step) & (np.abs(y - cy) <= W+step)):
            return True
    return False

def scan_to_points(scan,angle_step, lidar_offset):
    #converts the lidar scan into an array of occupied points in robot's reference frame
    x_list = []
    y_list = []
    for idx in range(len(scan)):
        if not math.isfinite(scan[idx]) or scan[idx]<=0.05: #check whether this lidar value is valid bro
            continue
        angle = idx*angle_step
        x = lidar_offset + np.cos(angle)*scan[idx]
        y = np.sin(angle)*scan[idx]
        x_list.append(x)
        y_list.append(y)
    final_array = np.column_stack((np.array(x_list),np.array(y_list)))
    return final_array


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
