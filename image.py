#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


imageList = list()
counter = 0 

class image_converter:

	def __init__(self):
		self.result_pub = rospy.Publisher("/ardrone/humandetection",String, queue_size=1)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
		
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		global counter
		imageList.append(cv_image)
		
		if(len(imageList) > 30):
			for image in imageList:
				cascPath = "/home/gokberk/tum_simulator_ws/src/first_flight/src/cascades/HS.xml"
				pplCascade = cv2.CascadeClassifier(cascPath)
				
				gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
				gray = normalize_grayimage(gray)
				pedestrians = pplCascade.detectMultiScale(
					gray,
					scaleFactor=1.2,
					minNeighbors=10,
					minSize=(32,96),
					flags = cv2.cv.CV_HAAR_SCALE_IMAGE
				)
				if len(pedestrians) > 0 :
					for (x, y, w, h) in pedestrians:
						cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
					counter = counter + 1 
					cv2.imshow("Image window", image)
					cv2.waitKey(3)
				
			print(counter)
			del imageList[:]
		
		if(counter > 10):
			msg = "Detected"
		else:
			msg = "Clear"
		
		self.result_pub.publish(msg)
		counter = 0

			
def normalize_grayimage(image):
	image = cv2.equalizeHist(image)
	return image



def main(args):
	rospy.init_node('image_converter', anonymous=True)
	ic = image_converter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

