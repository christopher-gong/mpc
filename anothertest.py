#!/usr/bin/env python
import os
import datetime
from sensor_msgs.msg import Imu
import rospy
from std_msgs.msg import String
import pandas as pd
import datetime
import sys

# to view prints, run command below , where 1234 is pid specified during roslaunch
# tail -f /proc/1234/fd/1
#f = open("../Desktop/thisisanotherstest.txt", "w+")
newrows = []

def callback(data):
	#print(int(datetime.datetime.now().strftime('%M:%S.%f')[:-4][6:]))
	if int(datetime.datetime.now().strftime('%M:%S.%f')[:-4][6:]) % 5 == 0:
		print(datetime.datetime.now())
		lay = data.linear_acceleration.y
		if lay < -2:
			print("Right")
		elif lay > 2:
			print("Left")
		else:
			print("Flat")
	
	#t = [str(datetime.datetime.now()), str(data.header.stamp.secs), str(data.header.stamp.nsecs), str(data.header.frame_id), str(data.orientation.x), str(data.orientation.y), str(data.orientation.z), str(data.orientation.w), str(data.orientation_covariance), str(data.angular_velocity.x), str(data.angular_velocity.y), str(data.angular_velocity.z), str(data.angular_velocity_covariance), str(data.linear_acceleration.x), str(data.linear_acceleration.y), str(data.linear_acceleration.z), str(data.linear_acceleration_covariance)]
	#newrows.append(t)
	#f.write(str(t))

def listener():
	rospy.init_node("listener", anonymous=True)
	rospy.Subscriber("imu/data", Imu, callback)
	i = 0
	while not rospy.core.is_shutdown():
		rospy.rostime.wallsleep(0.5)
		#if (i % 100 == 0):
			#i = 0
			#df = pd.DataFrame(newrows, columns=["time", "secs", "nsecs", "frame_id", "orient x", "orient y", "orient z", "orient w", "orient cov", "ang vel x", "ang vel y", "ang vel z", "ang vel cov", "lin acc x", "lin acc y", "lin acc z", "lin acc cov"])
			#df.to_csv('../info.csv')

listener()
