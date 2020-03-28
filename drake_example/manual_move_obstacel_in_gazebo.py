#!/usr/bin/env python

import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String

#start_pos = [17.5300, 55.4300, 1.5708]
start_pos = [2.,   6.,    -0.0]
curr_pos  = start_pos
jackals_pos = np.array([[0.0, 0.0], [0.0, 0.0]])

JACKAL = 'person_standing'

key_mapping = { 'w': [ -.1, 0, -1.57], 'x': [.1, 0 , 1.57], 
                'a': [ 0, -.1, 0.], 'd': [0, .1, 3.14], 
                's': [ 0, 0, 0.0] }
g_last_cmd = None 

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
	#print(curr_pos)
	
def keys_cb(msg, pub):
	global g_last_cmd
	if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
		return # unknown key.
	vals = key_mapping[msg.data[0]]
	g_last_cmd.pose.position.x    = curr_pos[0]+vals[0]
	g_last_cmd.pose.position.y    = curr_pos[1]+vals[1]
	q = quaternion_from_euler(0.0, 0.0, vals[2])
	g_last_cmd.pose.orientation.z = q[2]
	g_last_cmd.pose.orientation.w = q[3]
	pub.publish(g_last_cmd)

if __name__ == '__main__':
	rospy.init_node('move_person_around')
	
	pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
	sensors  = rospy.Subscriber('/gazebo/model_states', ModelStates, measurement_cb)
	rospy.Subscriber('keys', String, keys_cb, pub)

	rate = rospy.Rate(10)
	g_last_cmd = ModelState()
	g_last_cmd.model_name=JACKAL
	
	while not rospy.is_shutdown():
		#pub.publish(g_last_cmd)
		rate.sleep()


		