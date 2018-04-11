#!/usr/bin/python
# Import required libraries
import sys
import time
import RPi.GPIO as GPIO
 
# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)
 
# Define GPIO signals to use
# Physical pins 12,16,18,11
# GPIO18, GPIO23, GPIO24, GPIO17
StepPinsMotorRight = [18,23,24,17]

#Physical pins 13,15,29,31
# GPIO27, GPIO22, GPIO5, GPIO6
StepPinsMotorLeft = [27,22,5,6]

# as shown in manufacturers datasheet
Seq = [[1,0,0,1],
       [1,0,0,0],
       [1,1,0,0],
       [0,1,0,0],
       [0,1,1,0],
       [0,0,1,0],
       [0,0,1,1],
       [0,0,0,1]]

StepCount = len(Seq)
StepDir = 1 # Set to 1 or 2 for clockwise
            # Set to -1 or -2 for anti-clockwise
StepDirL = 1
# Read wait time from command line
WaitTime = (0.5)/float(1000)

# Initialise variables
StepCounterRight = 0
StepCounterLeft = 0


def setupMotors():
	# Set all pins as output
	for pin in StepPinsMotorLeft:
		GPIO.setup(pin,GPIO.OUT)
		GPIO.output(pin, False)

	for pin in StepPinsMotorRight:
		GPIO.setup(pin,GPIO.OUT)
		GPIO.output(pin,False)


def stepMotorRight(): 
	global StepCounterRight  
	print StepCounterRight,
	print Seq[StepCounterRight]
 
	for pin in range(0,4):
		xpinR=StepPinsMotorRight[pin]# Get GPIO
		if Seq[StepCounterRight][pin]!=0:
			print " Enable GPIO %i" %(xpinR)
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
 
  # Wait before moving on
	time.sleep(WaitTime)
def stepMotorLeft():
	global StepCounterLeft
	print StepCounterLeft,
	print Seq[StepCounterLeft]
 
	for pin in range(0,4):
		xpinL=StepPinsMotorLeft[pin]# Get GPIO
		if Seq[StepCounterLeft][pin]!=0:
			print " Enable GPIO %i" %(xpinL)
			GPIO.output(xpinL, True)
		else:
			GPIO.output(xpinL, False)

	StepCounterLeft += StepDirL 
	# If we reach the end of the sequence
	# start again
	if (StepCounterLeft>=StepCount):
		StepCounterLeft = 0
	if (StepCounterLeft<0):
		StepCounterLeft = StepCount+StepDirL
 
	time.sleep(WaitTime)

setupMotors()
for i in range(606):
	stepMotorRight()
	stepMotorLeft()
