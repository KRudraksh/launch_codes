#!/usr/bin/env python
import rospy
import math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
def get_eulers(q):
    eulers = euler_from_quaternion(q)
    print ("Yaw: " ,eulers[2]*180 / (math.pi))

def callback(data):
    q = []
    q.append(data.pose.orientation.x)
    q.append(data.pose.orientation.y)
    q.append(data.pose.orientation.z)
    q.append(data.pose.orientation.w)

    get_eulers(q) 

def subs():
    rospy.init_node("chuchiya_madarchod", anonymous= True)
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    subs()
