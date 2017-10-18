#!/usr/bin/env python

import rospy
import numpy
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# initialize node
rospy.init_node('later_to_occupancy_grid_node')

# listener of transforms between the car_base_link and the world frame
car_pose = tf.TransformListener()

# Occupancy grid message
map_msg = OccupancyGrid()
resolution = 0.01
width = 500
height = 500

# Car pose relative to world
x_car = 0.0
y_car = 0.0

# square size of the car footprint [m]
footprint = 0.1

# Map update rate (defaulted to 10 Hz)
rate = 2.0

# Range data
car_range = 0.0

def set_free_cells(grid, position, size):
	# set free the cells occupied by the car
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# size: 			r     radius of the footprint
	global resolution

	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2
    # Car pose relative to world

	rospy.loginfo("POSE: x_off: %f  y_off: %f", off_x, off_y)

	# set the roi to 1: known free positions
	for i in range(-size//2, size//2):
		for j in range(-size//2, size//2):
			grid[int(i + off_x), int(j + off_y)] = 1

def set_obstacle(grid, position, orientation, position_sonar, quaternion_sonar, car_range):
	# set the occupied cells when detecting an obstacle
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# orientation:      quaternion, orientation of the car
	global resolution

	off_x = position[1] // resolution + width  // 2
	off_y = position[0] // resolution + height // 2

	euler = tf.transformations.euler_from_quaternion(orientation)

	if not car_range == 0.0:

		rospy.loginfo("OBSTACLE: yaw: %f", euler[2])

		rotMatrix = numpy.array([[numpy.cos(euler[2]),   numpy.sin(euler[2])],
			                     [-numpy.sin(euler[2]),  numpy.cos(euler[2])]])
		obstacle = numpy.dot(rotMatrix,numpy.array([0, (car_range + position_sonar[0]) // resolution])) + numpy.array([off_x,off_y])
		rospy.loginfo("position obstacle: x:%f y:%f", obstacle[0], obstacle[1])

		# set probability of occupancy to 100 and neighbour cells to 50
		grid[obstacle[0], obstacle[1]] = int(100)
		if  grid[int(obstacle[0]+1), int(obstacle[1])]   < int(1):
			grid[int(obstacle[0]+1), int(obstacle[1])]   = int(50)
		if  grid[int(obstacle[0]), 	 int(obstacle[1]+1)] < int(1):
			grid[int(obstacle[0]),   int(obstacle[1]+1)] = int(50)
		if  grid[int(obstacle[0]-1), int(obstacle[1])]   < int(1):
			grid[int(obstacle[0]-1), int(obstacle[1])]   = int(50)
		if  grid[int(obstacle[0]),   int(obstacle[1]-1)] < int(1):
			grid[int(obstacle[0]),   int(obstacle[1]-1)] = int(50)

		t = 0.5
		i = 1
		free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
		while grid[int(free_cell[0]), int(free_cell[1])] < int(1):
			grid[int(free_cell[0]), int(free_cell[1])] = int(0)
			free_cell = numpy.dot(rotMatrix,numpy.array([0, t*i])) + numpy.array([off_x,off_y])
			i = i+1;
			print "filling cell (", int(free_cell[0]), ", ", int(free_cell[1]), ")"

def callback_range(msg):
	# callback range
	global car_range
	car_range = msg.ranges[0]


# Subscribers
range_sub = rospy.Subscriber("/car/scan", LaserScan, callback_range)

# Publishers
occ_pub = rospy.Publisher("/car/map", OccupancyGrid, queue_size = 10)


# main function
if __name__ == '__main__':

	# set grid parameters
	if rospy.has_param("occupancy_rate"):
		rate = rospy.get_param("occupancy_rate")

	if rospy.has_param("grid_resolution"):
		resolution = rospy.get_param("grid_resolution")

	if rospy.has_param("grid_width"):
		width = rospy.get_param("grid_width")

	if rospy.has_param("grid_height"):
		height = rospy.get_param("grid_height")

	# initialize grid with -1 (unknown)
	grid = numpy.zeros((width, height), dtype=numpy.int)
	grid.fill(int(-1))

	map_msg.info.origin.position.x = - width // 2 * resolution
	map_msg.info.origin.position.y = - height // 2 * resolution

	loop_rate = rospy.Rate(rate)

	while not rospy.is_shutdown():

		try:
			t = car_pose.getLatestCommonTime("/car_base_link", "/world")
			position, quaternion = car_pose.lookupTransform("/world", "/car_base_link", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		try:
			t = car_pose.getLatestCommonTime("/car_base_link", "/sonar_link")
			position_sonar, quaternion_sonar = car_pose.lookupTransform("/car_base_link", "/sonar_link", t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		set_free_cells(grid, position, int(footprint//resolution))

		set_obstacle(grid, position, quaternion, position_sonar, quaternion_sonar, car_range)

		map_msg.header.frame_id = 'map'
		map_msg.header.stamp = rospy.Time.now()
		map_msg.info.resolution = resolution
		map_msg.info.width = width
		map_msg.info.height = height

		grid_reshaped = grid.reshape(1, width*height)
		map_msg.data = range(width*height)
		for i in range(width*height):
			map_msg.data[i] = grid_reshaped[0,i]
		occ_pub.publish(map_msg)

		loop_rate.sleep()
