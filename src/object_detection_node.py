import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header

class ObjectDetectionNode(Node):
  def __init__(self):
    super().__init__('object_detection_node')
    self.publisher = self.create_publisher(Point, 'detected_objects', 10)
    self.subscription = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.image_callback, 10)
    self.bridge = CvBridge()
  
  def image_callback(self, data):
    # Convert ROS Image message to OpenCV image
    cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    
    # Crop the image to use only the lower half
    height, width, _ = cv_image.shape
    cv_image_lower = cv_image[height // 2:height, :]
    
    # Perform object detection
    detected_objects = self.detect_objects(cv_image_lower)
    
    for obj in detected_objects:
      print(obj[0])
      object_position_msg = Point()
      object_position_msg.x = float(obj[0])  # x-coordinate
      object_position_msg.y = float(obj[1])  # y-coordinate
      self.publisher.publish(object_position_msg)
  
  def detect_objects(self, image):
    # Convert image to HSV colour space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define range of colours for object detection
    lower_color = np.array([40, 50, 50])
    higher_color = np.array([80, 255, 255])
    
    # Threshold the HSV image to get only the desired colours
    mask = cv2.inRange(hsv_image, lower_color, higher_color)
    
    # Find contours in the mask
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Sort by area (keep only the biggest one)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
    
    detected_objects = []
    
    #Loop over detected contours
    for contour in contours:
      # Calculate the centroid of the contour
      M = cv2.moments(contour)
      if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        detected_objects.append((cx, cy)) # Append centroid coordinates to detected objects list
    
    return detected_objects