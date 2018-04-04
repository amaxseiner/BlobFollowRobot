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
StepPinsMotorLeft = [18,23,24,17]

#Physical pins 13,15,29,31
# GPIO27, GPIO22, GPIO5, GPIO6
StepPinsMotorRight = [27,22,5,6]

 
# Set all pins as output
for pin in StepPinsMotorLeft:
  GPIO.setup(pin,GPIO.OUT)
  GPIO.output(pin, False)

for pin in StepPinsMotorRight:
  GPIO.setup(pin,GPIO.out)
  GPIO.output(pin,False)

# Define advanced sequence
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
 
# Read wait time from command line
WaitTime = (0.5)/float(1000)
 

# Initialise variables
StepCounter = 0
 
# Start main loop
while True:
 
  print StepCounter,
  print Seq[StepCounter]
 
  for pin in range(0,4):
    xpinL=StepPinsMotorLeft[pin]# Get GPIO
    xpinR=StepPinsMotorLeft[pin]
    if Seq[StepCounter][pin]!=0:
      print " Enable GPIO %i" %(xpinL)
      print " Enable GPIO %i" %(xpinR)
      GPIO.output(xpinL, True)
      GPIO.output(XpinR, True)
    else:
      GPIO.output(xpinL, False)
      GPIO.output(xpinR, False)
 
  StepCounter += StepDir
 
  # If we reach the end of the sequence
  # start again
  if (StepCounter>=StepCount):
    StepCounter = 0
  if (StepCounter<0):
    StepCounter = StepCount+StepDir
 
  # Wait before moving on
  time.sleep(WaitTime)
