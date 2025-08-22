# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# This code is modified by Berk Calli from the following author.
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np

 
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
      current_frame = self.br.imgmsg_to_cv2(data)
      # current_frame = cv2.resize(current_frame, (640, 480))


      #Step 4:Finding Centers of Circles
      # # Convert the ROS Image message to an OpenCV image
      # current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
      # Color Thresholding
      # lower_bound = np.array([0, 100, 100])#red
      # upper_bound = np.array([35, 255, 255])#red
      #blue color
      # lower_bound = np.array([100, 100, 100])
      # upper_bound = np.array([140, 255, 255])
      # #green color
      # lower_bound = np.array([40, 40, 40])
      # upper_bound = np.array([80, 255, 255])
      #purple color
      # lower_bound = np.array([125, 100, 100])
      # upper_bound = np.array([160, 255, 255])
      # mask = cv2.inRange(current_frame_hsv, lower_bound, upper_bound)
      
      # # Apply the mask to the current frame
      # current_frame = cv2.bitwise_and(current_frame, current_frame, mask=mask)

      # # Calculate the centroid of the detected area
      # x = 0
      # y = 0
      # count = 0
      # for i in range(current_frame.shape[0]):
      #     for j in range(current_frame.shape[1]):
      #         if current_frame[i][j][0] > 0: 
      #             x += j
      #             y += i
      #             count += 1
      # if count > 0:
      #     x = int(x / count)
      #     y = int(y / count)
      #     cv2.circle(current_frame, (x, y), 5, (0, 0, 255), -1)
      #     cv2.putText(current_frame, f"Centroid: ({x}, {y})", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)



      #Step5:Canny Edge Detection
      # gray=cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
      # Preprocessing: Gaussian Blur
      # blurred = cv2.GaussianBlur(gray, (5, 5), 0)
      # Edge detection
      # current_frame = cv2.Canny(blurred, 50, 150)
      # current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)



      #Step6:Harris Corner Detection
      # gray=cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
      # gray=np.float32(gray)
      # dst = cv2.cornerHarris(gray,2,3,0.04)
      # dst = cv2.dilate(dst,None)
      # threshold = 0.1*dst.max()
      # current_frame[dst>threshold]=[0,0,255]
      # current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)


      #step7:Hough Circle Detection
      gray=cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
      # Preprocessing: Gaussian Blur
      blurred = cv2.GaussianBlur(gray, (5, 5), 0)
      # Edge detection
      edges = cv2.Canny(blurred, 50, 150)
      # Hough Circle Transform
      circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                                  param1=50, param2=30, minRadius=0, maxRadius=0)
      # Mark detected circles
      if circles is not None:
          circles = np.uint16(np.around(circles))
          for i in circles[0, :]:
              # Draw the outer circle
              cv2.circle(current_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
              # # Draw the center of the circle
              cv2.circle(current_frame, (i[0], i[1]), 2, (0, 0, 255), 3)
              # cv2.putText(current_frame, f"Center: ({i[0]}, {i[1]})", (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
              cv2.putText(current_frame, f"Center: ({i[0]}, {i[1]})", (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)
              print(f"Center: ({i[0]}, {i[1]})")

      # Publish the processed image
      self.publisher_.publish(self.br.cv2_to_imgmsg(current_frame, encoding='bgr8'))

  
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
