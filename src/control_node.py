import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlNode(Node):
  def __init__(self):
    super().__init__('control_node')
    self.subscription = self.create_subscription(Twist, 'desired_velocity', self.callback, 10)
    self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    self.r = 0.04
    self.L = 0.07
    self.max_velocity = 0.6
  
  def callback(self, msg):
    # Received desired velocity command
    # Convert linear and angular velocities to left and right wheel velocities
    left_wheel_velocity = (msg.linear.x - msg.angular.z * self.L / 2.0) / self.r
    right_wheel_velocity = (msg.linear.x + msg.angular.z * self.L / 2.0) / self.r
    
    # Ensure wheel velocities are within limits
    left_wheel_velocity = max(-self.max_velocity, min(left_wheel_velocity, self.max_velocity))
    right_wheel_velocity = max(-self.max_velocity, min(right_wheel_velocity, self.max_velocity))
    
    # Publish wheel velocities as Twist messages
    twist_msg = Twist()
    twist_msg.linear.x = left_wheel_velocity
    twist_msg.linear.y = right_wheel_velocity
    self.publisher.publish(msg)