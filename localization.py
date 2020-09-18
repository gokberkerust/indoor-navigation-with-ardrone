#!/usr/bin/env python
import roslib
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import imutils
import numpy as np


imageList = list()
counter = 0
left_vote = 0
right_vote = 0
calib_vote = 0

class localization:

	def __init__(self):
		self.result_pub = rospy.Publisher("/ardrone/localization", String, queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)

	def callback(self,data):
		global counter
		global left_vote
		global right_vote
		global calib_vote
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		imageList.append(cv_image)

		if(len(imageList) > 50):
			for image in imageList:
				lower = [70, 230, 240]
				upper = [120, 254,255]

				lower = np.array(lower, dtype = "uint8")
				upper = np.array(upper, dtype = "uint8")

				mask = cv2.inRange(image, lower, upper)
				output = cv2.bitwise_and(image, image, mask = mask)

				gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
				blurred = cv2.GaussianBlur(gray, (3, 3), 0)
				tight = cv2.Canny(blurred, 225, 250)

				

				ret,thresh = cv2.threshold(tight,127,255,0)
				contours,hierarchy = cv2.findContours(thresh, 1, 2)

				if len(contours) > 0:
					j = 0
					maxCont = 0
					tempArea = 0
					for cnt in contours:
						M = cv2.moments(cnt)
						if M["m00"] != 0:
							cX = int(M["m10"] / M["m00"])
							cY = int(M["m01"] / M["m00"])

						area = cv2.contourArea(cnt)
						if area > tempArea:
							maxCont = j
							tempArea = area
						j += 1
					print "tempArea: ", tempArea
					if tempArea > 10:
						cnt = contours[maxCont]
						M = cv2.moments(cnt)
						if M["m00"] != 0:
							cX = int(M["m10"] / M["m00"])
							cY = int(M["m01"] / M["m00"])						
						
						if(cX < 300):
								#vote to move drone left
							left_vote += 1
						elif(cX > 400):
							right_vote += 1
						else:
							calib_vote += 1

						cv2.circle(image, (cX,cY), 5, (255, 0, 0), 2)
						cv2.imshow("image", image)
						cv2.waitKey(3)

			del imageList[:] 
		#print ("left vote: ", left_vote, "right vote: ", right_vote, "calib vote: ", calib_vote)
		maxVote = max(right_vote,left_vote,calib_vote)
		if(left_vote+right_vote+calib_vote != 0):	
			if(maxVote == left_vote):
				msg = "left"
			elif(maxVote == right_vote):
				msg = "right"
			elif(maxVote == calib_vote):
				msg = "done"
			self.result_pub.publish(msg)

		left_vote = 0
		right_vote = 0
		calib_vote = 0




def main(args):
	rospy.init_node('localization', anonymous=True)
	lc = localization()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	main(sys.argv)