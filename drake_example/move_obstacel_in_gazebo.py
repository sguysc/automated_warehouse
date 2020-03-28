#!/usr/bin/env python

import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#start_pos = [17.5300, 55.4300, 1.5708]
start_pos = [2.,   6.,    -0.0]
curr_pos  = start_pos
jackals_pos = np.array([[0.0, 0.0], [0.0, 0.0]])

JACKAL = 'person_standing'

def measurement_cb(msg):
	global curr_pos
	global jackals_pos
	#global start_pos
	#global JACKAL
	i    = msg.name.index(JACKAL)
	pose = msg.pose[i].position
	Q    = msg.pose[i].orientation
	angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
	curr_pos = [pose.x, pose.y, angles[2]]
	
	for j in range(2):
		try:
			i    = msg.name.index('jackal%d' %j) 
			pose = msg.pose[i].position
			jackals_pos[j] = np.array([pose.x, pose.y])
		except:
			pass
	
	#print(curr_pos)

if __name__ == '__main__':
	#global curr_pos
	#global jackals_pos
	#global start_pos
	#global JACKAL
	
	rospy.init_node('move_person_around')
	
	pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
	sensors  = rospy.Subscriber('/gazebo/model_states', ModelStates, measurement_cb)
	
	aa=ModelState()
	aa.model_name=JACKAL

	#import pdb; pdb.set_trace()
	dir = -1.0
	step_size = 0.1
	wait_counter = 0
	while not rospy.is_shutdown():
		if(np.linalg.norm( jackals_pos[0]-np.array([curr_pos[0], curr_pos[1]] ) ) <= 1. or \
		   np.linalg.norm( jackals_pos[1]-np.array([curr_pos[0], curr_pos[1]] ) ) <= 1. ):
			dir *= -1
			if(dir == 1):
				start_pos[2] = 3.14
			else:
				start_pos[2] = 0.0
			wait_counter = 1
		#import pdb; pdb.set_trace()
		if(curr_pos[1] <= 2.5):
			dir = 1.
			start_pos[2] = 3.14
			wait_counter = 4
		elif(curr_pos[1] >= 6.0):
			dir = -1.
			start_pos[2] = 0.0
			wait_counter = 4
		
		aa.pose.position.x    = start_pos[0]
		aa.pose.position.y    = curr_pos[1] + step_size*dir
		q = quaternion_from_euler(0.0, 0.0, start_pos[2])
		aa.pose.orientation.z = q[2]
		aa.pose.orientation.w = q[3]
				
		if(wait_counter > 0):
			q = quaternion_from_euler(0.0, 0.0, 0.0)
			aa.pose.orientation.z = q[2]
			aa.pose.orientation.w = q[3]
			pub.publish(aa)
			time.sleep(wait_counter)
			wait_counter = 0
		else:
			pub.publish(aa)
			time.sleep(0.1)
		#else:
		#	print('(%f,%f,%f)' %(curr_pos[0],curr_pos[1],curr_pos[2]))
	
	
		