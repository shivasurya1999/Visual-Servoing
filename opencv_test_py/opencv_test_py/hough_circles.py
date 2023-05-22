# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries

#!/usr/bin/env python
from __future__ import print_function
import numpy as np #ssl command (my command)
import argparse #ssl command (my command)
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.

    self.subscription = self.create_subscription(
      Image, 
      '/camera1/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'output_image', 10)

      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)


    # PLACE YOUR CODE HERE. PROCESS THE CURRENT FRAME AND PUBLISH IT. IF YOU ARE HAVING DIFFICULTY PUBLISHING IT YOU CAN USE THE FOLLOWING LINES TO DISPLAY IT VIA OPENCV FUNCTIONS

    # img = cv2.imread(current_frame)
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY) #converting to gray scale

    detected_circles=cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,dp=1,minDist=40,param1=50,param2=20,minRadius=20,maxRadius=30)  
    print(detected_circles) #printing detected circles to terminal

    if detected_circles is not None:
      # Convert the circle parameters a, b and r to integers.
      detected_circles = np.uint16(np.around(detected_circles))
  
      for pt in detected_circles[0, :]:
          a, b, r = pt[0], pt[1], pt[2]
  
          # Draw the circumference of the circle.
          cv2.circle(current_frame, (a, b), r, (0, 255, 0), 2)
  
          # Draw a small circle (of radius 1) to show the center.
          cv2.circle(current_frame, (a, b), 1, (0, 0, 255), 3)
          print("The center of the circle is (",a,",", b,")")   #Printing each detected center
          cv2.imshow("Detected Circle", current_frame) #showing the detected circles 
      cv2.waitKey(0)


    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
