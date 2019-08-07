#!/usr/bin/env python
# BEGIN ALL
import argparse
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# BEGIN KEYMAP
key_mapping = { 'w': [ 0, 1], 'x': [0, -1], 
                'a': [-1, 0], 'd': [1,  0], 
                's': [ 0, 0] }
# END KEYMAP

def keys_cb(msg, twist_pub):
	# BEGIN CB
	if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
		return # unknown key.
	vels = key_mapping[msg.data[0]]
	# END CB
	t = Twist()
	t.angular.z = vels[0]
	t.linear.x  = vels[1]
	twist_pub.publish(t)

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Robot number to control.')
	parser.add_argument('--n', type=int, default=1,
					help='an integer for the robot number')
	args = parser.parse_args()
	print('Controller for Jackal%d' %args.n)

	rospy.init_node('keys_to_twist%d' %args.n)
	twist_pub = rospy.Publisher('/jackal%d/jackal_velocity_controller/cmd_vel' %args.n, Twist, queue_size=1)
	rospy.Subscriber('keys', String, keys_cb, twist_pub)
	rospy.spin()
	
	# END ALL
