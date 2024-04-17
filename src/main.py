import rclpy
from rclpy.executors import MultiThreadedExecutor
from object_detection_node import ObjectDetectionNode
from navigation_node import NavigationNode
import time

def main(args=None):
  rclpy.init(args=args)

  # Create nodes
  object_detection_node = ObjectDetectionNode()
  navigation_node = NavigationNode()

  # Create executor
  executor = MultiThreadedExecutor()

  # Add nodes to executor
  executor.add_node(object_detection_node)
  executor.add_node(navigation_node)

  # Spin all nodes
  try:
    while rclpy.ok:
      executor.spin_once()
      #navigation_node.update_navigation()
  finally:
    # Shutdown all nodes
    executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
  main()