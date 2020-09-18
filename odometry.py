#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
import numpy as np


imageList = list()
counter = 0 
left_vote = 0
right_vote = 0
calib_vote = 0

class odometry:

	def __init__(self):
		self.result_pub = rospy.Publisher("/ardrone/calibration_response",String, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)


	def callback(self,data):
		global counter
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		global left_vote
		global right_vote
		global calib_vote

		imageList.append(cv_image)
		
		if(len(imageList) > 20):
			for image in imageList:
					gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
					(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
					cv2.circle(image, maxLoc, 5, (255, 0, 0), 2)	
					(x,y) = maxLoc
					#print(x)
					if(x < 300):
						#vote to move drone left
						left_vote += 1
					elif(x > 380):
						right_vote += 1
					else:
						calib_vote += 1
			cv2.circle(image, maxLoc, 5, (255, 0, 0), 2)
			cv2.imshow("Naive", image)
			cv2.waitKey(3)
				
			del imageList[:] 
		maxVote = max(right_vote,left_vote,calib_vote)
		if(left_vote+right_vote+calib_vote != 0):	
			if(maxVote == left_vote):
				print("left")
				msg = "left"
				self.result_pub.publish(msg)
			elif(maxVote == right_vote):
				print("rigth")
				msg = "right"
				self.result_pub.publish(msg)
			elif(maxVote == calib_vote):
				print("calib")
				msg = "calibrated"
				self.result_pub.publish(msg)

		#if(right_vote > left_vote):
		#	msg = "right"	
		#	self.result_pub.publish(msg)
		#elif(right_vote < left_vote):
		#	msg = "left"
		#	self.result_pub.publish(msg)
		left_vote = 0
		right_vote = 0
		calib_vote = 0
		#counter += 1
		#if(counter < 400):
		#	msg = "left"	
		#	self.result_pub.publish(msg)
		#elif(counter > 400 and counter < 800):
		#	msg = "right"	
		#	self.result_pub.publish(msg)
		#elif(counter > 800):
		#	msg = "calibrated"	
		#	self.result_pub.publish(msg)

def main(args):
	rospy.init_node('odometry', anonymous=True)
	od = odometry()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)
