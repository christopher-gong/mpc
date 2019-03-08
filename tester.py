#!/usr/bin/env python
import os
import datetime
from sensor_msgs.msg import Imu

f = open("../Desktop/thisisatest.txt", "w+")
f.write("LOGGING SESSION BEGUN AT " + str(datetime.datetime.now()) + "\n\n")


print os.getcwd()

import rospy
from std_msgs.msg import String

def callback(data):
	f.write("header: \n")
	f.write("\t seq: " + str(data.header.seq) + "\n")
	f.write("\t stamp: \n")
	f.write("\t\t secs: " + str(data.header.stamp.secs) + "\n")
	f.write("\t\t nsecs: " + str(data.header.stamp.nsecs) + "\n")
	f.write("\t frame_id: " + str(data.header.frame_id) + "\n")
	f.write("orientation: \n")
	f.write("\t x: " + str(data.orientation.x) + "\n")
	f.write("\t y: " + str(data.orientation.y) + "\n")
	f.write("\t z: " + str(data.orientation.z) + "\n")
	f.write("\t w: " + str(data.orientation.w) + "\n")
	f.write("orientation_covariance: " + str(data.orientation_covariance) + "\n")
	f.write("angular_velocity: \n");
	f.write("\t x: " + str(data.angular_velocity.x) + "\n")
	f.write("\t y: " + str(data.angular_velocity.y) + "\n")
	f.write("\t z: " + str(data.angular_velocity.z) + "\n")
	f.write("angular_velocity_covariance: " + str(data.angular_velocity_covariance) + "\n")
	f.write("linear_acceleration: \n");
	f.write("\t x: " + str(data.linear_acceleration.x) + "\n")
	f.write("\t y: " + str(data.linear_acceleration.y) + "\n")
	f.write("\t z: " + str(data.linear_acceleration.z) + "\n")	
	f.write("linear_acceleration_covariance: " + str(data.linear_acceleration_covariance) + "\n")
	f.write("\n")

def listener():
	rospy.init_node("listener", anonymous=True)
	rospy.Subscriber("imu/data", Imu, callback)
	rospy.spin()

listener()
