import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
  def __init__(self):
    super().__init__('obstacle_avoidance_node')
    self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    self.publisher = self.create_publisher(Twist, 'cmd_vel', 20)
    self.tw = Twist()
    
    self.min_distance_threshold = 0.3
  
  def scan_callback(self, msg):
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
              pass
          elif left_obstacle_detected:
              # Obstacle detected on left side, turn right
              self.tw.linear.x = -0.3
              self.tw.angular.z = -0.3
          elif right_obstacle_detected:
              # Obstacle detected on right side, turn left
              self.tw.linear.x = -0.3
              self.tw.angular.z = 0.3
      
          self.publisher.publish(self.tw)
  
  def detect_obstacle(self, ranges):
    # Check if any obstacle is within a certain threshold distance
    for distance in ranges:
      if distance < self.min_distance_threshold:
        return True # Obstacle detected
    return False  # No obstacle detected