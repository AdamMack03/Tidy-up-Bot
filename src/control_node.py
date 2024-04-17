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
    self.max_velocity = 0.5
  
  def callback(self, msg):    
    # Ensure wheel velocities are within limits
    twist_msg = Twist()
    if msg.linear.x != 0:
      forward_scaling_factor = 0.1 / abs(msg.linear.x)
      twist_msg.linear.x = msg.linear.x * forward_scaling_factor
    else:
      twist_msg.linear.x = 0.0
    if msg.angular.z != 0:
      angular_scaling_factor = self.max_velocity / abs(msg.angular.z)
      twist_msg.angular.z = msg.angular.z *angular_scaling_factor
    else:
      twist_msg.angular.z = 0.0
    
    # Publish wheel velocities as Twist messages
    self.publisher.publish(twist_msg)