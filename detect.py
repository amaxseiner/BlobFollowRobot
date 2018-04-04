from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import rospy
from geometry_msgs.msg import Vector3



# initialize the camera and grab a reference to the raw camera capture
lower_red0 = np.array([0,50,50])
upper_red0 = np.array([10,255,255])
lower_red1 = np.array([170,50,50])
upper_red1 = np.array([180,255,255])
FOVW = 62.2
FOVH = 48.8

camera = PiCamera()

camera.resolution = (640, 480)
camera.framerate = 60

camera.start_preview() # camera head up time

time.sleep(0.1)

rawCapture = PiRGBArray(camera,size=(640,480))

stream = camera.capture_continuous(rawCapture,format="bgr", use_video_port=True)

pub = rospy.Publisher('object_center', Vector3, queue_size=10)
rospy.init_node('camera', anonymous=True)

def getNormAndFOV(FOVx,FOVy,mask,x,y):
	(H,W) = mask.shape[:2]
	dH = 2.0/(H)
	dW = 2.0/(W)
	xprime = (x - (W/2.0))* dW
	yprime = (y - (H/2.0))* -dH
	Xdeg = xprime * FOVH/2.0
	Ydeg = yprime * FOVW/2.0
	return (Xdeg, Ydeg)

def getNorm(mask,x,y):
	(H,W) = mask.shape[:2]
	dH = 2.0/(H)
	dW = 2.0/(W)
	xprime = (x - (W/2.0))* dW
	yprime = (y - (H/2.0))* -dH
	return (xprime,yprime)

def getFOV(FOVx,FOVy,Point):
	Xdeg = Point[0] * FOVH/2.0
	Ydeg = Point[1] * FOVW/2.0
	return (Xdeg, Ydeg)

def getCenter(mask):
	M = cv2.moments(mask)
	cx =  int(M['m10']/M['m00'])
	cy =  int(M['m01']/M['m00'])
	return (cx,cy)


for frame in stream:	
	tick = time.time()
	img1 = frame.array # ~.001 sec

	img = cv2.resize(img1,(240,160))
	img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # ~.01 sec
	# lower mask (0-10)
	
	mask0 = cv2.inRange(img_hsv, lower_red0, upper_red0)

	# upper mask (170-180)
	
	#mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)

	mask = mask0#+mask1

	#print mask
	#output_img = img.copy()
	#output_img[np.where(mask==0)] = 0

	#output_hsv = img_hsv.copy()
	#output_hsv[np.where(mask==0)] = 0

	#ret,thresh = cv2.threshold(mask,127,255,0)
	#im2,contours,hierarchy = cv2.findContours(thresh, 1, 2)
	
	(newX, newY) = getCenter(mask)

	degrees = getNormAndFOV(FOVW,FOVH,mask,newX, newY)
	#print (newX,newY) 
	#print degrees
	
	cv2.circle(img,(newX,newY),25,(0,0,255),-1)
	#cv2.imshow('mask',mask)
	cv2.imshow('new',img)
	#tock = time.time()
	rawCapture.truncate(0)
	tock = time.time()
	
	if not rospy.is_shutdown():
		centroid = Vector3()
		centroid.x = degrees[0]
		centroid.y = degrees[1]
		rospy.loginfo(centroid)
		pub.publish(centroid)

  	print "Time to complete = " + str(tock - tick) + " sec"
	cv2.waitKey(1)
