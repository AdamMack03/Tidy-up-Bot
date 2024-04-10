import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
  def __init__(self):
    super().__init__('obstacle_avoidance_node')
    self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
    self.publisher = self.create_publisher(Twist, 'desired_velocity', 10)
    self.tw = Twist()
  
  def scan_callback(self, msg):
    # Process laser scan data to detect obstacles
    is_obstacle_detected = self.detect_obstacle(msg.ranges)
    
    if is_obstacle_detected:
      #if an obstacle is detected, stop the robot
      self.tw.linear.x = 0.0
      self.tw.angular.z = 0.0
    else:
      # If no obstacle is detected, continue moving forward
      self.tw.linear.x = 0.0
      self.tw.angular.z = 0.0
    
    self.publisher.publish(self.tw)
  
  def detect_obstacle(self, ranges):
    # Check if any obstacle is within a certain threshold distance
    min_distance_threshold = 0.1
    for distance in ranges:
      if distance < min_distance_threshold:
        return True # Obstacle detected
    return False  # No obstacle detected