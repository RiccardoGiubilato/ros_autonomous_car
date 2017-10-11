#!/usr/bin/env python

import rospy
import tf
import math
import numpy
from geometry_msgs.msg import Vector3

# parameters for computing odometry
counter = 0
wh_r = 0.03 # wheel radius [m]
enc0 = 0
enc1 = 0
pose_x = 0
pose_y = 0
yaw0 = 0
car_pose_br = tf.TransformBroadcaster()

rospy.init_node('odom_node')

def callback(msg):
	global counter, wh_r, enc0, enc1, pose_x, pose_y, yaw0

	if counter == 0:
		enc0 = msg.x
		yaw0 = msg.y
	
	counter = counter + 1
	
	rotMatrix = numpy.array([[numpy.cos(math.pi/180*msg.y), -numpy.sin(math.pi/180*msg.y)], 
		                       [numpy.sin(math.pi/180*msg.y),  numpy.cos(math.pi/180*msg.y)]])
		                       
	# 1 wheel rotation = 60 encoder values	                     
	new_pose = numpy.dot(rotMatrix,numpy.array([2*math.pi*wh_r*(msg.x - enc0)/60, 0])) + numpy.array([pose_x,pose_y])

	pose_x = new_pose[0]
	pose_y = new_pose[1]
	
	enc0 = msg.x
	yaw0 = msg.y

	car_pose_br.sendTransform((pose_x, pose_y, 0),
		                        tf.transformations.quaternion_from_euler(0, 0, math.pi/180*msg.y),
		                        rospy.Time.now(),
		                        "car_base_link",
		                        "world")

joy_sub = rospy.Subscriber('/car/power', Vector3, callback)

if __name__ == '__main__':
	
	rate = rospy.Rate(20)
	
	while not rospy.is_shutdown():
		rate.sleep()
