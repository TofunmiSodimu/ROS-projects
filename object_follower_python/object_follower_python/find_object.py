# Intro to Robotics Research 7785
# Georgia Institute of Technology
# Tofunmi Sodimu

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge

class MinimalVideoSubscriber(Node):

	def __init__(self):		
		# Creates the node.
		super().__init__('minimal_video_subscriber')

		# Set Parameters
		self.declare_parameter('show_image_bool', True)
		self.declare_parameter('window_name', "Raw Image")

		#Determine Window Showing Based on Input
		self._display_image = bool(self.get_parameter('show_image_bool').value)
		self._display_image = False

		# Declare some variables
		self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		
		#Only create image frames if we are not running headless (_display_image sets this)
		if(self._display_image):
		# Set Up Image Viewing
			cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
			cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
		
		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
		    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
		    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
		    depth=1
		)

		#Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/camera/image/compressed',
				self._image_callback,
				image_qos_profile)
		self._video_subscriber # Prevents unused variable warning.

		#Declare publisher
		self.publisher_ = self.create_publisher(Point, 'topic', 10)

	def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		if(self._display_image):
			# Display the image in a window
			self.show_image(self._imgBGR)
				

	def get_image(self):
		print('Hey')
		return self._imgBGR

	def show_image(self, img):
		print('Now')
		blur_img = cv2.medianBlur(img,5)
		gray_img=cv2.cvtColor(blur_img, cv2.COLOR_BGR2GRAY)
		circles=cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT,1,90,param1=70,param2=60,minRadius=0,maxRadius=0)
		if(circles is not None):
			circles = np.uint16(np.around(circles))
			for i in circles[0,:]:
				cv2.circle(img, (i[0], i[1]), i[2], (0,255,0),3)
				cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
				message = Point(x=float(i[0]),y=float(i[1]),z=0.0)
				self.publisher_.publish(message)
				self.get_logger().info('Publishing: "%s"' %str(message))
		#cv2.imshow('deteted circles', gray_img)
	
		#cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

	def get_user_input(self):
		return self._user_input


def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = MinimalVideoSubscriber() #Create class object to be used.
	print('hey')
	while rclpy.ok():
		print('hey again')
		rclpy.spin_once(video_subscriber) # Trigger callback processing.
		video_subscriber.show_image(video_subscriber.get_image())		
		if video_subscriber.get_user_input() == ord('q'):
			cv2.destroyAllWindows()
			break
		#if(video_subscriber._display_image):
			#video_subscriber.show_image(video_subscriber.get_image())		
			#if video_subscriber.get_user_input() == ord('q'):
			#	cv2.destroyAllWindows()
			#	break

	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
