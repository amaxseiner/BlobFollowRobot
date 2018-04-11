#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
import sys
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

FOVW = 62.2
FOVH = 48.8

stepsPerRev = 4096

Kp = 0.6

degreesToStepsRatio = 1024.0/23.0

count= 0
# Physical pins 12,16,18,11
# GPIO18, GPIO23, GPIO24, GPIO17
StepPinsMotorRight = [18,23,24,17]

#Physical pins 13,15,29,31
# GPIO27, GPIO22, GPIO5, GPIO6
StepPinsMotorLeft = [27,22,5,6]

clockwiseStepDir = 1
clockwiseStepDirFast = 2
counterClockwiseStepDir = -1
counterClockwiseStepDirFast = -2

rightTurn = (-1,-1)
rightTurnFast = (-2,-2)
leftTurn = (1,1)
leftTurnFast = (2,2)

backwards = (1,-1)
backwardsFast = (2,-2)
forwards = (-1,1)
forwardsFast = (-2,2)
#StepDir = -2 # Set to 1 or 2 for clockwise
            # Set to -1 or -2 for anti-clockwise

WaitTime = (0.8)/float(1000)

StepCounterRight = 0
StepCounterLeft = 0

# sequence for the stepper motor
Seq = [[1,0,0,1],
       [1,0,0,0],
       [1,1,0,0],
       [0,1,0,0],
       [0,1,1,0],
       [0,0,1,0],
       [0,0,1,1],
       [0,0,0,1]]

StepCount = len(Seq)

pub_data = Vector3()


def setupMotors():
        # Set all pins as output
        for pin in StepPinsMotorLeft:
                GPIO.setup(pin,GPIO.OUT)
                GPIO.output(pin, False)

        for pin in StepPinsMotorRight:
                GPIO.setup(pin,GPIO.OUT)
                GPIO.output(pin,False)

def stepMotorLeft(StepDir):
        global StepCounterLeft
        #print StepCounterLeft,
        #print Seq[StepCounterLeft]
 
        for pin in range(0,4):
                xpinL=StepPinsMotorLeft[pin]# Get GPIO
                if Seq[StepCounterLeft][pin]!=0:
                        #print " Enable GPIO %i" %(xpinL)
                        GPIO.output(xpinL, True)
                else:
                        GPIO.output(xpinL, False)

        StepCounterLeft += StepDir 
        # If we reach the end of the sequence
        # start again
        if (StepCounterLeft>=StepCount):
                StepCounterLeft = 0
        if (StepCounterLeft<0):
                StepCounterLeft = StepCount+StepDir
 
        time.sleep(WaitTime)

def stepMotorRight(StepDir): 
        global StepCounterRight  
        #print StepCounterRight,
        #print Seq[StepCounterRight]
 
        for pin in range(0,4):
                xpinR=StepPinsMotorRight[pin]# Get GPIO
                if Seq[StepCounterRight][pin]!=0:
                        #print " Enable GPIO %i" %(xpinR)
                        GPIO.output(xpinR, True)
                else:
                        GPIO.output(xpinR, False)
 
        StepCounterRight += StepDir
 
        # If we reach the end of the sequence
        # start again
        if (StepCounterRight>=StepCount):
                StepCounterRight = 0
        if (StepCounterRight<0):
                StepCounterRight = StepCount+StepDir

def controlMotor(movement,stepLength):
	global forwards,backwards,leftTurn,rightTurn

	print(stepLength)
	if(movement == forwards):
		print("going forward")
		for i in range(stepLength):
			stepMotorLeft(forwards[0])
			stepMotorRight(forwards[1])
	elif(movement == backwards):
		print("going backwards")
		for i in range(stepLength):
			stepMotorLeft(backwards[0])
			stepMotorRight(backwards[1])
	elif(movement == rightTurn):
		print("turning right")
		for i in range(stepLength):
			stepMotorLeft(rightTurn[0])
			stepMotorRight(rightTurn[1])
	elif(movement == leftTurn):
		print("turning left")
		for i in range(-stepLength):
			stepMotorLeft(leftTurn[0])
			stepMotorRight(leftTurn[1])

def doControl(x):
	global stepsPerRev,forwards,leftTurn,rightTurn
	if((x < 5 and x > 0) or (x > -5 and x < 0)):
		controlMotor(forwards,int(Kp*256))
	else:
		e = Kp*(x - 0) # 0 is the target
		if(e > 0):
			controlMotor(rightTurn,int(e*degreesToStepsRatio))
		else:
			controlMotor(leftTurn, int(e*degreesToStepsRatio))

def callback(data):
	global pub_data,count
	pub_data = data
	rospy.loginfo("x = " + str(data.x) + " y = " + str(data.y))
	count += 1
	if count == 3:
		doControl(data.x)
		count = 0
def listener():
	global pub_data
	# In ROS, nodes are uniquely named. If two nodes with the same
	# node are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('centroid_listener', anonymous=True)

	rospy.Subscriber("object_center", Vector3, callback)

	# spin() simply keeps python from exiting until this node is stopped
	#    rospy.spin()


	rate = rospy.Rate(30) # 30hz
	while not rospy.is_shutdown():
		#rospy.loginfo(" --- OLD --- x = " + str(pub_data.x) + " y = " + str(pub_data.y))
		rate.sleep()

setupMotors()
if __name__ == '__main__':
	listener()
