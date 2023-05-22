# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries

#!/usr/bin/env python
from __future__ import print_function
import math
import numpy as np #ssl command (my command)
import argparse #ssl command (my command)
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from numpy.linalg import pinv
from std_msgs.msg import Float64MultiArray
 
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


    self.target_frame = 'base_link'  #base_link frame (from the topic /tf) is considered as reference to view link1 in lookup_transform used at later stage 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
    self.target_frame1 = 'link1' #link1 frame (from the topic /tf) is considered as reference to view link2 in lookup_transform used at later stage
    self.tf_buffer1 = Buffer()
    self.tf_listener1 = TransformListener(self.tf_buffer1, self)
      
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

    self.publisher1 = self.create_publisher(Float64MultiArray,'/forward_velocity_controller/commands', 1) #the message published from listener_callback below will be passed to velocity controller 

      
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
    # (h,w) = current_frame.shape[:2] #ssl command (my command)
    # cv2.imshow("output_image", current_frame)
    # cv2.waitKey(1)

    ######MY CODE######
    ## HSV THRESHOLDING HAS BEEN USED BELOW ##

    #Code to detect blue circle and print its center 
    lower_blue = [100, 100, 100]  #lower thresholds 
    upper_blue = [120, 255, 255]  #upper thresholds 
    lower_blue = np.array(lower_blue, dtype = "uint8") 
    upper_blue = np.array(upper_blue, dtype = "uint8")
    hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)  #converting the current image to hsv space 
    mask = cv2.inRange(hsv, lower_blue, upper_blue)       #using the threshold values and inRange function of OpenCV to create a mask for color detection
    output = cv2.bitwise_and(hsv, hsv, mask = mask)       #applying the mask to the hsv image to detect the required color 
    height = output.shape[0]    
    width = output.shape[1]
    agg_centre_x = 0
    agg_centre_y = 0
    elem = 0
    for i in range(0,height):   #iterating over the size of the image to check pixel values 
      for j in range (0,width):
        if all(output[i,j] != [0,0,0]) :  #if pixel values are not zero, there is our desired color in that pixel 
          agg_centre_x = agg_centre_x + j #adding the x coordinate of such non zero pixel values 
          agg_centre_y = agg_centre_y + i #adding the y coordinate of such non zero pixel values
          elem = elem + 1 #adding the number of such pixels
    [centre_xblue,centre_yblue] = [agg_centre_x/elem,agg_centre_y/elem] #averaging the pixel locations to get the desired color center location
    print("blue center= (",centre_xblue,",",centre_yblue,")") #printing the center to terminal
    cv2.imshow("thresholding", np.hstack([current_frame, output ])) #outputing the image obtained after the bitwise_and operation above
    cv2.waitKey(500) #waiting for 2 seconds before proceeding with the further code

    #Code to detect red circle and print its center 
    lower_red = [0, 100, 100]
    upper_red = [50, 255, 255]
    lower_red = np.array(lower_red, dtype = "uint8")
    upper_red = np.array(upper_red, dtype = "uint8")
    mask = cv2.inRange(hsv, lower_red, upper_red)
    output = cv2.bitwise_and(hsv, hsv, mask = mask)
    height = output.shape[0]
    width = output.shape[1]
    agg_centre_xred = 0
    agg_centre_yred = 0
    elemred = 0
    for k in range(0,height):
      for l in range (0,width):
        o = output[k,l]
        if ((o[0]!=0)or(o[1]!=0)or(o[2]!=0)):
          agg_centre_xred = agg_centre_xred + l
          agg_centre_yred = agg_centre_yred + k 
          elemred = elemred + 1
    [centre_xred,centre_yred] = [agg_centre_xred/elemred,agg_centre_yred/elemred]
    print("red center= (",centre_xred,",",centre_yred,")")
    cv2.imshow("thresholding", np.hstack([current_frame, output ]))
    cv2.waitKey(500)

    #Code to detect green circle and print its center
    lower_green = [50, 100, 100]
    upper_green = [100, 255, 255]
    lower_green = np.array(lower_green, dtype = "uint8")
    upper_green = np.array(upper_green, dtype = "uint8")
    mask = cv2.inRange(hsv, lower_green, upper_green)
    output = cv2.bitwise_and(hsv, hsv, mask = mask)
    height = output.shape[0]
    width = output.shape[1]
    agg_centre_x = 0
    agg_centre_y = 0
    elem = 0
    for i in range(0,height):
      for j in range (0,width):
        if all(output[i,j] != [0,0,0]) :
          agg_centre_x = agg_centre_x + j
          agg_centre_y = agg_centre_y + i 
          elem = elem + 1
    [centre_xgreen,centre_ygreen] = [agg_centre_x/elem,agg_centre_y/elem]
    print("green center= (",centre_xgreen,",",centre_ygreen,")")
    cv2.imshow("thresholding", np.hstack([current_frame, output ]))
    cv2.waitKey(500)

    #Code to detect pink circle and print its center
    lower_pink = [145, 100, 100]
    upper_pink = [150, 255, 255]
    lower_pink = np.array(lower_pink, dtype = "uint8")
    upper_pink = np.array(upper_pink, dtype = "uint8")
    mask = cv2.inRange(hsv, lower_pink, upper_pink)
    output = cv2.bitwise_and(hsv, hsv, mask = mask)
    height = output.shape[0]
    width = output.shape[1]
    agg_centre_x = 0
    agg_centre_y = 0
    elem = 0
    for i in range(0,height):
      for j in range (0,width):
        if all(output[i,j] != [0,0,0]) :
          agg_centre_x = agg_centre_x + j
          agg_centre_y = agg_centre_y + i 
          elem = elem + 1
    [centre_xpink,centre_ypink] = [agg_centre_x/elem,agg_centre_y/elem]
    print("pink center= (",centre_xpink,",",centre_ypink,")")
    cv2.imshow("thresholding", np.hstack([current_frame, output ]))
    cv2.waitKey(500)


    #In the below code the features have been first represented as a 4*4 matrix with rows as each features locations (first row :blue, second:red,third:green,fourth:pink)
    s_desired_T = np.array([[166,-357,0,1],[205,-431,0,1],[279,-391,0,1],[239,-317,0,1]]) #desired features after transforming from pixel frame to image plane frame
    s_actual_T = np.array([[centre_xblue,-centre_yblue,0,1],[centre_xred,-centre_yred,0,1],[centre_xgreen,-centre_ygreen,0,1],[centre_xpink,-centre_ypink,0,1]]) #actual features after transforming from pixel frame to image plane frame
    #The above s_desired_T and s_actual_T are transposed with colums as each features locations (first column :blue, second:red,third:green,fourth:pink). Note that 1 is appended as 4th 
    #element because in the future we need the velocity vectoor to be multiplied to a 4*4 transformation matrix 
    s_desired = s_desired_T.T
    s_actual = s_actual_T.T
    e = s_actual - s_desired #Calculate error between actual and desired features 
    #In below step for every feature lambda is considered as -np.array([0.04,0],[0,0.04]) which is multiplied with Le+(Image jacobian) =  np.array([-1,0],[0,1])
    Vcim = 0.04*e #The above calculation results in the constant 0.04 directly being multiplied to our 'e'
    from_frame_rel = self.target_frame
    to_frame_rel = self.target_frame1
    to_frame_rel1 = 'link2'
    #lookup_transform below gives the pose in quaternion form 
    t = self.tf_buffer.lookup_transform(to_frame_rel,from_frame_rel,rclpy.time.Time()) #transformation of link1 wrt base_frame
    t1 = self.tf_buffer.lookup_transform(to_frame_rel1,to_frame_rel,rclpy.time.Time()) #transformation of link2 wrt link1

    #quaternions are stored as below 
    q0 = t.transform.rotation.x
    q1 = t.transform.rotation.y
    q2 = t.transform.rotation.z
    q3 = t.transform.rotation.w
     
    #The below code converts quaternions to transformation matrices  
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1     


    qa0 = t1.transform.rotation.x
    qa1 = t1.transform.rotation.y
    qa2 = t1.transform.rotation.z
    qa3 = t1.transform.rotation.w

        # First row of the rotation matrix
    ra00 = 2 * (qa0 * qa0 + qa1 * qa1) - 1
    ra01 = 2 * (qa1 * qa2 - qa0 * qa3)
    ra02 = 2 * (qa1 * qa3 + qa0 * qa2)
     
    # Second row of the rotation matrix
    ra10 = 2 * (qa1 * qa2 + qa0 * qa3)
    ra11 = 2 * (qa0 * qa0 + qa2 * qa2) - 1
    ra12 = 2 * (qa2 * qa3 - qa0 * qa1)
     
    # Third row of the rotation matrix
    ra20 = 2 * (qa1 * qa3 - qa0 * qa2)
    ra21 = 2 * (qa2 * qa3 + qa0 * qa1)
    ra22 = 2 * (qa0 * qa0 + qa3 * qa3) - 1


    #below are the transformation matrices 

    #link1 wrt base_link 
    transf_matrixb21 = np.array([[r00, r01, r02, t.transform.translation.x],
                           [r10, r11, r12, t.transform.translation.y],
                           [r20, r21, r22, t.transform.translation.z],[0, 0, 0, 1]])    

    #link2 wrt link1
    transf_matrixl12 = np.array([[ra00, ra01, ra02, t1.transform.translation.x],
                           [ra10, ra11, ra12, t1.transform.translation.y],
                           [ra20, ra21, ra22, t1.transform.translation.z],[0, 0, 0, 1]])

    
    compound_transf_mat = transf_matrixb21.dot(transf_matrixl12) #transformation of link2 wrt base_link 
    Vcbase_mat = compound_transf_mat*Vcim #transforming camera velocity to base_frame from image_plane frame
    Vcbase = np.array([[0],[0],[0],[0]]) #place holder
    Vcbase[0,0] = Vcbase_mat[0,0] + Vcbase_mat[0,1] + Vcbase_mat[0,2] + Vcbase_mat[0,3] #intermediate variable 1 (can be ignored)
    Vcbase[1,0] = Vcbase_mat[1,0] + Vcbase_mat[1,1] + Vcbase_mat[1,2] + Vcbase_mat[1,3] #intermediate variable 2 (can be ignored)
    Vcbase[2,0] = Vcbase_mat[2,0] + Vcbase_mat[2,1] + Vcbase_mat[2,2] + Vcbase_mat[2,3] #intermediate variable 3 (can be ignored)
    TwistBase = np.array([[Vcbase[0,0]],[Vcbase[1,0]],[Vcbase[2,0]],[0],[0],[0]]) #Twist of the camera wrt base #assumption that the angular velocities of end effector are all zero 
    o20 = np.array([[compound_transf_mat[0,3]],[compound_transf_mat[1,3]],[compound_transf_mat[2,3]]]) #intermediate variable, (can be ignored)
    o10 = np.array([[transf_matrixb21[0,3]],[transf_matrixb21[1,3]],[transf_matrixb21[2,3]]]) #intermediate variable,(can be ignored)
    elem0 = o20-o10 #intermediate variable (can be ignored)
    vec1 = np.array([[-o20[1,0]],[o20[0,0]],[0]]) #intermediate variable (can be ignored)
    vec2 = np.array([[-elem0[1,0]],[elem0[0,0]],[0]]) #intermediate variable (can be ignored)
    vec3 = np.array([[0],[0],[0]]) #intermediate variable (can be ignored)
    #the below is the jacobian matrix that converts robot joint velocities to camera velocities 
    Jacobian = np.array([[vec1[0,0],vec2[0,0],vec3[0,0]],[vec1[1,0],vec2[1,0],vec3[1,0]],[vec1[2,0],vec2[2,0],vec3[2,0]],[0,0,0],[0,0,0],[1,1,1]]) #assumption about z axes being rotation axes
    pinvJacobian = pinv(Jacobian) #pseudo inverse of jacobian matrix 
    joint_angular_velocities = pinvJacobian.dot(TwistBase) #joint angular velocities from J+robot * Vcamera-base
    link_1_angular_vel = joint_angular_velocities[0,0] #angular velocity of first link 
    link_2_angular_vel = joint_angular_velocities[1,0] #angular velocity of second link 
    my_msg = Float64MultiArray() #message to be published
    d = [link_1_angular_vel,link_2_angular_vel]
    print(d)
    my_msg.data = d #angular velocities passed into the above created message 
    self.publisher1.publish(my_msg) #message published  

  
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
