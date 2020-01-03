#!/usr/bin/env python

import numpy as np
import rospy

from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from tf.transformations import euler_from_quaternion, quaternion_inverse
from tf2_geometry_msgs import do_transform_vector3


pose = np.zeros(3)
msg_num = 0

def my_callback(msg):
    global msg_num
    global pose
    msg_num += 1
    #pose   = msg.pose.position
    Q      = msg.pose.orientation

    #self.linvel = self.TransformVectorToBody(msg.twist[i].linear, Q)
    #self.rotvel = msg.twist[i].angular

    # get euler angles to know heading
    angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])

    # if we move quadrant, unwrap from the previous angle
    theta = np.unwrap([pose[2], angles[2]])
    # store for other uses
    pose = np.array([msg.pose.position.x, msg.pose.position.y, theta[1]])
    print('Frame %d: x=%.3f\ty=%.3f\ttheta=%.3f' %(msg_num, pose[0], pose[1], pose[2]))

# rotate vectors from world frame to body frame
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

if __name__ == '__main__':
    try:
        rospy.init_node('guy', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber("/mocap_node/Jackal1/pose", PoseStamped, my_callback)
        #rospy.Subscriber("/mocap_node/hat/pose", PoseStamped, my_callback)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

