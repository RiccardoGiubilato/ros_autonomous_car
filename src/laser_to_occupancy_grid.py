#!/usr/bin/env python

import rospy
import numpy
from tf import TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# initialize node
rospy.init_node('later_to_occupancy_grid_node')

# listener of transforms between the car_base_link and the world frame
car_pose = TransformListener()

# Occupancy grid message
map_msg = OccupancyGrid()
resolution = 0.01
width = 500
height = 500


# Car pose relative to world
x_car = 0.0
y_car = 0.0

# Map update rate (defaulted to 10 Hz)
rate = 10.0

# Range data
car_range = 0.0

def set_free_cells(grid, position, size):
	# set free the cells occupied by the car
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# size: 			r     radius of the footprint
	global resolution

	off_x = position[0] // resolution
	off_y = position[1] // resolution

	# set the roi to 1: known free positions
	for i in range(-size//2, size//2)
		for j in range(-size//2, size//2)
			grid[i + off_x, j + off_y] = 1

def set_obstacle(grid, position, orientation, car_range):
	# set the occupied cells when detecting an obstacle
	# grid:				ndarray [width,height]
	# position:			[x y] pose of the car
	# orientation:      quaternion, orientation of the car
	global resolution

	off_x = position[0] // resolution
	off_y = position[1] // resolution

	euler = tf.transformations.euler_from_quaternions(orientation)
	rotMatrix = numpy.array([[numpy.cos(math.pi/180*euler[1]), -numpy.sin(math.pi/180*euler[1])],
		                    [numpy.sin(math.pi/180*euler[1]),  numpy.cos(math.pi/180*euler[1])]])

	obstacle = numpy.dot(rotMatrix,numpy.array([car_range // resolution, 0])) + numpy.array([off_x,off_y])

	grid[obstacle[0], obstacle[1]] = 1

def callback_range(msg):
	# callback range
	global car_range
	car_range = msg.ranges(0)

# Subscribers
range_sub = rospy.Subscriber("/car/scan", LaserScan, callback_range)

# Publishers
occ_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 10)


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
	grid = numpy.zeros(width, height)
	grid.fill(-1)

	rate = rospy.Rate(rate)

	while not rospy.is_shutdown():

		t = car_pose.getLatestCommonTime("/car_base_link", "/world")
		position, quaternion = car_pose.lookupTransform("/car_base_link", "/world", t)

		set_free_cells(grid, position, 10)

		if not car_range
			set_obstacle(grid, position, orientation, car_range)

		map_msg.header.frame_id = 'map'
		map_msg.header.stamp = rospy.Time.now()
		map_msg.info.resolution = resolution
		map_msg.info.width = width
		map_msg.info.height = height
		map_msg.data = grid.reshape(1, width*height)
		occ_pub.publish(map_msg)

		rate.sleep()
