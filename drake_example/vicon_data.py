#!/usr/bin/env python

#rosrun teleop_twist_keyboard teleop_twist_keyboard.py

import numpy as np
import rospy
import os
import argparse

from geometry_msgs.msg import Twist, PoseStamped, Pose2D, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_inverse
from tf2_geometry_msgs import do_transform_vector3


g_pose = np.zeros(3)
msg_num = 0
ft2m = 1.0

def my_callback(msg):
	global msg_num
	global g_pose
	msg_num += 1
	
	# vicon - TransformStamped
	pose   = msg.transform.translation
	pose.x = pose.x*ft2m
	pose.y = pose.y*ft2m
	Q      = msg.transform.rotation
	angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])

	#self.linvel = self.TransformVectorToBody(msg.twist[i].linear, Q)
	#self.rotvel = msg.twist[i].angular

	# get euler angles to know heading
	angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])

	# if we move quadrant, unwrap from the previous angle
	theta = np.unwrap([g_pose[2], angles[2]])
	# store for other uses
	g_pose = np.array([pose.x, pose.y, theta[1]])
	print('Frame %d: x=%.3f\ty=%.3f\ttheta=%.3f' %(msg_num, g_pose[0], g_pose[1], g_pose[2]))

# rotate vectors from world frame to body frame
'''
def TransformVectorToBody(vect, q):
	v = Vector3Stamped()
	v.vector.x = vect.x
	v.vector.y = vect.y
	v.vector.z = vect.z

	t = TransformStamped()

	quaternion = np.array((q.x, q.y, q.z, q.w))
	quat_conj = np.array((-quaternion[0], -quaternion[1], \
				  -quaternion[2], quaternion[3]))
	quat_inv = quat_conj / np.dot(quaternion, quaternion)


	t.transform.rotation.x = quat_inv[0]
	t.transform.rotation.y = quat_inv[1]
	t.transform.rotation.z = quat_inv[2]
	t.transform.rotation.w = quat_inv[3]

	vt = do_transform_vector3(v, t)

	return vt.vector #np.array([vt.vector.x, vt.vector.y, vt.vector.z ])
'''

if __name__ == '__main__':
	try:
		parser = argparse.ArgumentParser()
		parser.add_argument("-i","--index", help="robot number")
		args = parser.parse_args()
		idx = int(args.index)
		os.environ['ROS_MASTER_URI'] = 'http://jackal%d:11311/'%idx
		
		rospy.init_node('guy_vicon_%d'%idx, anonymous=True)
		rate = rospy.Rate(10) # 10hz
		#idx = 3
		rospy.Subscriber("/vicon/jackal%d/jackal%d" %(idx,idx), TransformStamped, my_callback)
		#rospy.Subscriber("/vicon/jackal2/jackal2", TransformStamped, my_callback)
		#rospy.Subscriber("/vicon/hat1_1/hat1_1", TransformStamped, my_callback)
		#rospy.Subscriber("/vicon/Helmet/Helmet", TransformStamped, my_callback)

		while not rospy.is_shutdown():
			rate.sleep()

	except rospy.ROSInterruptException:
		pass

