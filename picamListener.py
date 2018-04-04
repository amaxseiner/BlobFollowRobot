#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3

pub_data = Vector3()

def callback(data):
    global pub_data
    pub_data = data
    rospy.loginfo("x = " + str(data.x) + " y = " + str(data.y) + " z = " + str(data.z))
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
        rospy.loginfo(" --- OLD --- x = " + str(pub_data.x) + " y = " + str(pub_data.y))
        rate.sleep()

if __name__ == '__main__':
    listener()
