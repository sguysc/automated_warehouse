#!/usr/bin/env python

import time

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#start_pos = [17.5300, 55.4300, 1.5708]
start_pos = [1.2192,   -2.1336,    1.5708]
curr_pos  = start_pos

JACKAL = 'jackal0'

def measurement_cb(msg):
	global curr_pos
	global start_pos
	global JACKAL
	i    = msg.name.index(JACKAL)
	pose = msg.pose[i].position
	Q    = msg.pose[i].orientation
	angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
	curr_pos = [pose.x, pose.y, angles[2]]
	#print(curr_pos)

if __name__ == '__main__':
	global curr_pos
	global start_pos
	global JACKAL
	
	rospy.init_node('stam2')
	
	pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
	sensors  = rospy.Subscriber('/gazebo/model_states', ModelStates, measurement_cb)
	
	aa=ModelState()
	aa.model_name=JACKAL
	aa.pose.position.x    = start_pos[0]
	aa.pose.position.y    = start_pos[1]
	q = quaternion_from_euler(0.0, 0.0, start_pos[2])
	aa.pose.orientation.z = q[2]
	aa.pose.orientation.w = q[3]

	#import pdb; pdb.set_trace()
	
	while not rospy.is_shutdown():
		time.sleep(5.)
		# only fix if we haven't started
		if(abs(curr_pos[2] - start_pos[2]) > 0.3 and \
		   abs(curr_pos[1] - start_pos[1]) < 0.2 and \
		   abs(curr_pos[0] - start_pos[0]) < 0.5 ):
			print('fixing ....')
			pub.publish(aa)
	
	
		