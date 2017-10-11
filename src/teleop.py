#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

vel_pub = rospy.Publisher('/car/cmd_vel', Vector3, queue_size=10)

def callback(msg):
	vel_msg = Vector3()
	vel_msg.z = msg.axes[3]
	vel_msg.x  = msg.axes[1]
	vel_pub.publish(vel_msg)

joy_sub = rospy.Subscriber('/joy', Joy, callback)

if __name__ == '__main__':
	
	rospy.init_node('teleop_node')
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
		#vel_pub.publish(Twist())
		rate.sleep()		
	
