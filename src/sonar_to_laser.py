#!/usr/bin/env python

import rospy
import tf
import math
import numpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan


rospy.init_node('sonar_to_laser_node')
pub = rospy.Publisher('car/scan', LaserScan, queue_size=10)

Scan_msg = LaserScan()
Scan_msg.angle_min = math.pi/4 - 0.05
Scan_msg.angle_max = math.pi/4 + 0.05
Scan_msg.angle_increment = 0.1
Scan_msg.range_min = 0.05
Scan_msg.range_max = 0.5

def callback(msg):
	Scan_msg.ranges = [msg.z / 100]
	pub.publish(Scan_msg)
	
sub = rospy.Subscriber('car/power', Vector3, callback)

rospy.spin()
	
