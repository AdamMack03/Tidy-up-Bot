import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import numpy as np
from math import atan2, pi

class NavigationNode(Node):
  def __init__(self):
    super().__init__('navigation_node')
    self.point_sub = self.create_subscription(Point, 'detected_objects', self.detected_objects_callack, 10)
    self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    self.tw = Twist()
    
    # Flag to indicate whether an object is currently being approached
    self.is_approaching_object = False
    
    # Distance threshold for considering the object as close enough to start pushing
    self.pushing_distance_threshold = 210
    self.push_flag = False
  
    # Position of the detected object
    self.detected_object_position = Point()
    
    self.front_distance_to_wall = 0.0
    self.min_distance_threshold = 0.15
    self.max_velocity = 0.5
  
  def scan_callback(self, msg):
    twist = Twist()
    front_scan_range = msg.ranges[len(msg.ranges) // 2]
    if not np.isnan(front_scan_range) and not np.isinf(front_scan_range):
      self.front_distance_to_wall = front_scan_range
    else:
      self.front_distance_to_wall = 0.0
      
    # Process laser scan data to detect obstacles
    is_obstacle_detected = self.detect_obstacle(msg.ranges)
    
    if is_obstacle_detected:
      # Determine whether obstacle is on left or right side
      mid_index = len(msg.ranges) // 2
      # Split laser scan data into left and right halves
      left_ranges = msg.ranges[:mid_index:10]
      right_ranges = msg.ranges[mid_index::10]
      
      left_obstacle_detected = any(distance < self.min_distance_threshold for distance in left_ranges)
      right_obstacle_detected = any(distance < self.min_distance_threshold for distance in right_ranges)
      
      if left_obstacle_detected and right_obstacle_detected:
        self.update_navigation()
      elif left_obstacle_detected:
        # Obstacle detected on left side, turn right
        twist.linear.x =+ -0.1
        twist.angular.z =+ 0.6
        self.move(twist)
      elif right_obstacle_detected:
        # Obstacle detected on right side, turn left
        twist.linear.x =+ -0.1
        twist.angular.z =+ -0.6
        self.move(twist)
    else:
      self.update_navigation()

  
  def detect_obstacle(self, ranges):
    # Check if any obstacle is within a certain threshold distance
    for distance in ranges:
      if distance < self.min_distance_threshold:
        return True # Obstacle detected
    return False  # No obstacle detected
  
  def odom_callback(self, msg):
    orientation = msg.pose.pose.orientation
    _, _, yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
    target_yaw = round(yaw / (pi / 2)) * (pi / 2)
    self.angular_velocity = self.calculate_angular_velocity(yaw, target_yaw)
  
  def detected_objects_callack(self, msg):
    # Object detected, update navigation behaviour
    if not self.is_approaching_object:
      self.is_approaching_object = True
    self.detected_object_position = msg
    if self.compute_distance_to_object(msg) >= self.pushing_distance_threshold:
      self.push_flag = True
  
  def update_navigation(self):
    twist = Twist()
    if self.front_distance_to_wall > 0.2 and self.is_approaching_object:
      twist.linear.x =+ 0.1
      twist.angular.z =+ -self.calculate_desired_angular_velocity()
      if self.push_flag:
        twist.linear.x =+ 0.1
        twist.angular.z =+ self.angular_velocity
    else:
      # No object detected, turn and look for objects
      self.is_approaching_object = False
      self.push_flag = False
      twist.linear.x =+ 0.0
      twist.angular.z =+ 0.3
    self.move(twist)

  def quaternion_to_euler(self, x, y, z, w):
    # Convert quaternion to Euler angles
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    pitch = atan2(2 * (w * y - z * x), 1 - 2 * (y**2 + z**2))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z**2 + y**2))
    return roll, pitch, yaw

  def calculate_desired_angular_velocity(self):
    object_angle = atan2(self.detected_object_position.x - 320, 120)  # Angle of object relative to robot's orientation
    if abs(object_angle) < 0.1:
      desired_angular_velocity = 0.0
    else:
      desired_angular_velocity = 0.6 * object_angle  # Proportional control
    return desired_angular_velocity
  
  def calculate_angular_velocity(self, current_yaw, target_yaw):
    error = target_yaw - current_yaw
    if error > 0:
      error += pi
    elif error < 0:
      error -= pi
    return error * 0.1
  
  def compute_distance_to_object(self, object_position):
    print("y =", object_position.y)
    distance = object_position.y
    return distance

  def move(self, twist):    
    # Ensure wheel velocities are within limits
    twist_msg = Twist()
    
    if twist.linear.x != 0.0:
      forward_scaling_factor = 0.1 / abs(twist.linear.x)
      twist_msg.linear.x = twist.linear.x * forward_scaling_factor
    else:
      twist_msg.linear.x = 0.0
    if twist.angular.z != 0.0:
      angular_scaling_factor = self.max_velocity / abs(twist.angular.z)
      twist_msg.angular.z = twist.angular.z *angular_scaling_factor
    else:
      twist_msg.angular.z = 0.0
    # Publish wheel velocities as Twist messages
    self.publisher.publish(twist_msg)