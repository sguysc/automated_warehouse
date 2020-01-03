#!/usr/bin/env python

# general stuff
import numpy as np
import json
import argparse
import dill
import networkx as nx
import re
import logging # can't handle the ros logging :(

from warehouse_map import LoadMP, GetSpecificControl, find_nearest, GetRotmat, FL_L

# ROS stuff
import rospy
from geometry_msgs.msg import Twist, TransformStamped, Vector3Stamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_inverse
from tf2_geometry_msgs import do_transform_vector3

ROBOT_TYPE = 'JACKAL'  # original JACKAL run with 'roslaunch jackal_gazebo jackal_world.launch'
#ROBOT_TYPE = 'TURTLEBOT'

MAP = 'lab'
MAP_FILE = MAP + '.map'
SYNTH_AUTOMATA_FILE = MAP + '.json'
MP_MAP_FILE = MAP + '.pickle'
LABEL2BIT_FILE = MAP + '.label2bit'
pix2m = 1.0 #0.2 #[m]

#umax    = 2.6 * 1.6 * 1000.0 / 3600.0  # mph -> m/sec     5.0
#umax    = .3  # jackal m/sec     5.0
umax    = 0.5 #1.0  # jackal m/sec     5.0
delmax  = 80.0*np.pi/180.0  #rad   30.0 80
logger = None			

# class to handle a single robot comm.
class Jackal:
	def __init__(self, idx, map_file=MAP_FILE, aut_file=SYNTH_AUTOMATA_FILE, mp_file=MP_MAP_FILE, l2b_file=LABEL2BIT_FILE):
		self.all_topics_loaded = False
		self.Fs = 10 # main/control loop frequency
		self.idx = idx
		self.msg_num = 0
		self.pose   = np.array([14.0, 14.0, np.pi/2.0]) #None
		self.linvel = None
		self.rotvel = None
		#self.L = 0.42 # for the jackal
		self.L = FL_L #*15.0 #for the forklift
		self.do_calc = True
		self.u = np.array([0.0, 0.0]) # stores the last controls
		# integrators
		self.v_prev = 0.0
		self.delta_prev = 0.0
		
		# load the map properties
		aut = open(map_file, 'r')
		map_prop = json.load(aut)
		aut.close()
		#import pdb; pdb.set_trace()
		cell = map_prop['cell']
		bounds = np.array([ map_prop['workspace'][0]['x'],  map_prop['workspace'][0]['y'], \
								 map_prop['workspace'][0]['X'],  map_prop['workspace'][0]['Y']])
		self.W_xgrid = np.arange(bounds[0]+cell/2.0, bounds[2]-cell/2.0, cell)
		self.W_ygrid = np.arange(bounds[1]+cell/2.0, bounds[3]-cell/2.0, cell)

		# load the slugs solution
		aut = open(aut_file, 'r')
		self.automata = json.load(aut)
		aut.close()
		self.G = nx.read_gpickle(mp_file)
		self.mps = LoadMP()
		
		dbfile = open(l2b_file, 'rb')
		self.map_label_2_bit = dill.load(dbfile)
		dbfile.close()
			
		# the reverse dictionary is useful
		self.map_bit_2_label = dict((v, k) for k, v in self.map_label_2_bit.items())
		self.states, self.actions = GetSpecificControl(self.automata, self.map_bit_2_label, debug=False)
		
		for state in self.states:
			print('R%d (%s)' %(self.map_label_2_bit[state],state))
		
		self.curr_state = 0
		self.N_state  = len(self.states)
		self.N_ellipse = len(self.mps[ self.actions[self.curr_state] ]['V'])
		self.curr_ell = self.FindNextValidEllipse()
		self.K     = self.mps[self.actions[self.curr_state] ]['K'][self.curr_ell]
		self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.actions[self.curr_state] ]['xcenter'][self.curr_ell], \
										self.states[self.curr_state])
		self.x_ref_hires = self.mps[self.actions[self.curr_state] ]['xtraj']
		#import pdb; pdb.set_trace()
		self.u_ref = self.mps[self.actions[self.curr_state] ]['unom'][self.curr_ell]
		self.u_ref_hires = self.mps[self.actions[self.curr_state] ]['utraj']

		#import pdb; pdb.set_trace()
		#
		# handle the ROS topics
		# send commands
		if('JACKAL' in ROBOT_TYPE):
			self.control_pub = rospy.Publisher('/jackal%d/jackal_velocity_controller/cmd_vel' %self.idx, Twist, queue_size=1) #Jackal
			#self.control_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=1) #Jackal
		elif('TURTLEBOT' in ROBOT_TYPE):
			self.control_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1) #turtlebot
		# get data
		self.sensors_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.measurement_cb)
		#self.sensors_sub = rospy.Subscriber('/gazebo/default/pose/info', ModelStates, self.measurement_cb)
		
		if(False):
			self.xinterp, self.uinterp = self.GetClosestInterpPoint(self.x_ref_hires, self.states[self.curr_state])
		else:
			self.xinterp = self.x_ref
			self.uinterp = self.u_ref
			
		self.control(self.K, self.xinterp, self.uinterp, do_calc=False) #send a 0 for the first time!
		
		self.timer = -1E6
		self.r = rospy.Rate(self.Fs)
		

	#/gazebo/model_states. This runs in 1000Hz, so we basically just ignore most of the measurements
	def measurement_cb(self, msg):
		#if(self.all_topics_loaded == False):
		#	return
		self.all_topics_loaded = True
		#print(self.msg_num)
		self.msg_num += 1
			
		# decipher the msg comm. coming from gazebo (ground truth). later, switch to vicon in lab
		if('JACKAL' in ROBOT_TYPE):
			i      = msg.name.index('jackal%d' %(self.idx)) # Jackal
			#i      = msg.name.index('jackal') # Jackal
		elif('TURTLEBOT' in ROBOT_TYPE):
			i      = msg.name.index('mobile_base') # TurtleBot
			
		pose   = msg.pose[i].position
		Q = msg.pose[i].orientation
		
		self.linvel = self.TransformVectorToBody(msg.twist[i].linear, Q)
		#self.linvel = msg.twist[i].linear
		self.rotvel = msg.twist[i].angular
		
		# get euler angles to know heading
		angles = euler_from_quaternion([Q.x, Q.y, Q.z, Q.w])
		
		# if we move quadrant, unwrap from the previous angle
		theta = np.unwrap([self.pose[2], angles[2]])
		# store for other uses
		self.pose = np.array([pose.x, pose.y, theta[1]]) # (gazebo->mine axes match )
	
	# main loop
	def Run(self):
		while not self.all_topics_loaded:
			# wait till we start to get measurements
			self.r.sleep()
			
		while not rospy.is_shutdown():
			#if(self.msg_num > 120*1000):
			#	import pdb; pdb.set_trace()
			# do control and call the robot (you first execute the control for the given ellipse you're in,
			# only then, you see if you're at a new funnel/knot point and switch controls)
			if(True):
				self.xinterp, self.uinterp = self.GetClosestInterpPoint(self.x_ref_hires, self.states[self.curr_state])
			else:
				self.xinterp = self.x_ref
				self.uinterp = self.u_ref

			#if(self.msg_num - self.timer < 1000):
				#wait between funnel to funnel 1.0sec
				# need to check if there is a possibility to break, because this just continues with inertia
			#	self.control(self.K*0.0, self.xinterp, 0.0*self.uinterp, do_calc=self.do_calc)
			#else:
			self.control(self.K, self.xinterp, self.uinterp, do_calc=self.do_calc)
			self.do_calc = False

			# check if we're in the next ellipse on the same funnel
			if(self.curr_ell < self.N_ellipse - 1):
				# because sometimes we run too fast and we're going through the ellipses in a flash
				# this goes from the end of the funnel, backwards, to see the furthest away ellipse 
				# that it is in
				next_ell = self.N_ellipse - 1
				while( next_ell > self.curr_ell ):
					if(self.CheckEllipseCompletion(self.states[self.curr_state], \
												   self.actions[self.curr_state], next_ell) == True):
						self.K     = self.mps[self.actions[self.curr_state] ]['K'][next_ell]
						self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.actions[self.curr_state] ]['xcenter'][next_ell], \
													self.states[self.curr_state])
						self.u_ref = self.mps[self.actions[self.curr_state] ]['unom'][next_ell]
						# this doesn't change when we go through another ellipse
						#self.x_ref_hires = self.mps[ self.actions[self.curr_state] ]['xtraj']
						#self.u_ref_hires = self.mps[ self.actions[self.curr_state] ]['utraj']
						self.curr_ell = next_ell
						#self.do_calc = True
						#self.timer = self.msg_num
						break;
					next_ell -= 1

			#import pdb; pdb.set_trace()
			# check if we're in the next funnel (and overflow when in the final one)
			next_state = (self.curr_state + 1) if (self.curr_state < self.N_state - 1) else 0
			if (self.CheckFunnelCompletion(self.states[next_state], self.actions[next_state]) == True):
				rospy.loginfo('reached funnel %d (%s)' %(next_state, self.states[next_state]))
				#we've reached the beginning of a new funnel, so update the states
				self.curr_state = next_state
				# sometimes the first ellipse has zero speed and it causes trouble. skip ellipses ahead
				# it's fine because if it doesn't have controls, then it is also at the same place as first ellipse.
				# this is usually only one ellipse. Maybe should take care of it when creating the funnels or
				self.curr_ell = self.FindNextValidEllipse()
				# get how many points are in this new motion primitive
				self.N_ellipse = len(self.mps[ self.actions[self.curr_state] ]['V'])
				# update all the new motion parameters
				self.K     = self.mps[self.actions[self.curr_state] ]['K'][self.curr_ell]
				self.x_ref, __ = self.ConvertRelPos2Global(self.mps[self.actions[self.curr_state] ]['xcenter'][self.curr_ell], \
											self.states[self.curr_state])
				self.u_ref = self.mps[self.actions[self.curr_state] ]['unom'][self.curr_ell]
				self.x_ref_hires = self.mps[ self.actions[self.curr_state] ]['xtraj']
				self.u_ref_hires = self.mps[ self.actions[self.curr_state] ]['utraj']
				self.do_calc = True
				self.timer = self.msg_num

			# output the telemetry to file
			#print(':%d' %(self.msg_num))
			self.telemetry()
			self.r.sleep()
		#end of main loop, Fs [Hz]

	def telemetry(self):
		# telemetry
		now = rospy.get_rostime()
		t = now.secs + now.nsecs/1.0E9
		#rospy.logdebug
		try:
			logger.debug('%.3f;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%d;%d;%d;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;' \
					 '%.2f;%.2f;%.2f;%.2f;%.2f' \
					   %(t, self.msg_num, self.pose[0], self.pose[1], self.pose[2]*180.0/np.pi, \
						 self.x_ref[0], self.x_ref[1], self.x_ref[2]*180.0/np.pi, self.u_ref[0], self.u_ref[1], \
						self.curr_state, self.curr_ell, int(self.do_calc), self.actions[self.curr_state], \
						self.u[0], self.u[1], self.linvel.x, self.linvel.y, self.linvel.z,\
						self.rotvel.x, self.rotvel.y, self.rotvel.z, self.xinterp[0],self.xinterp[1],self.xinterp[2]*180.0/np.pi, \
						self.uinterp[0],self.uinterp[1]))
		except:
			pass
	
	# rotate vectors from world frame to body frame
	def TransformVectorToBody(self, vect, q):
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
	
	# y/x = 1/(tau*s+1) ==>  y(k) = T/(tau+T) * (x(k)+tau/T*y(k-1))
	def LPF(self, x, y_prev, tau=1.):
		T = 1./self.Fs
		y = T/(tau + T) * (x + tau/T*y_prev)
		return y

	# check if a point is in a predefined ellipse
	def CloseEnoughToNextPoint(self, funnel, action, ellipse_num, dist=0.2, ang=0.2):
		__, ellipse_pose = self.GetCoordAndEllipseFromLabel(funnel, action, ellipse_num)
		
		# create a circle/ellipse and use the current infrastructure to see if we're close enough to switch controls
		S = np.diag([dist**-2, dist**-2, ang**-2])
		
		if( self.InEllipse(S, ellipse_pose) ):
			return True
		
		return False
	
	# point is inside the ellipse iff (x-x0)'S(x-x0)<=0
	def InEllipse(self, S, x0, pose=None):
		# adjust both angles to be (-pi : +pi]
		#x0[2] = (( -x0[2] + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0
		if(pose):
			xbar = pose - x0
		else:
			xbar = self.pose - x0

		# find the smallest angle difference
		ang_diff = (xbar[2] + np.pi) % (2*np.pi) - np.pi
		xbar[2] = ang_diff
		#(x-x0)'S(x-x0)<=0
		ret = xbar.dot(S.dot(xbar))
			
		return (ret<=1)
	
	# sometimes the first ellipse has zero speed and it causes trouble. skip ellipses ahead
	def FindNextValidEllipse(self):
		find_non_zero = 0
		while True:
			if(np.linalg.norm(self.mps[self.actions[self.curr_state]]['unom'][find_non_zero]) < 0.01):
				find_non_zero += 1
				if(find_non_zero == self.N_ellipse):
					break
			else:
				break
		return find_non_zero
	
	# check if the pose of robot is in the ellipse
	def CheckEllipseCompletion(self, funnel, action, ellipse_num):
		# gets the S matrix already rotated to the right global orientation
		S, ellipse_pose = self.GetCoordAndEllipseFromLabel(funnel, action, ellipse_num)
		# check if the ellipse equation holds
		if( self.InEllipse(S, ellipse_pose) ):
			return True
		
		return False
	
	# check if the pose of robot is in the next funnel
	def CheckFunnelCompletion(self, funnel, action):
		# see if we are at the first ellipse of the new funnel
		find_non_zero = self.FindNextValidEllipse()
		# check if we are in the ellipse					
		return self.CheckEllipseCompletion(funnel, action, find_non_zero)

	# extract S (rotated to current pose) and center of ellipse from current mp (funnel&ellipse)
	def GetCoordAndEllipseFromLabel(self, funnel, action, ellipse_num):
		mp = self.mps[action]
		
		S  = mp['V'][ellipse_num]
		x0 = mp['xcenter'][ellipse_num]
		
		e_center, rotmat = self.ConvertRelPos2Global(x0, funnel)
		# rotate the ellipse to fit the global coordinate system
		# x'*V*x<1 & z=R*x ==> x=invR*x ==> z'*invR'*V*invR*z < 1
		rotmat = np.array([[rotmat[0,0], rotmat[0,1], 0.], [rotmat[1,0],rotmat[1,1],0.], [0.,0.,1.]]) # convert to 3D
		rotmat = np.linalg.inv(rotmat)
		S = rotmat.T.dot(S.dot(rotmat))

		return S, e_center

	# coordinates transfer from mp frame to global frame
	def ConvertRelPos2Global(self, rel_pose, funnel):
		orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', funnel)] # extract first ellipse pose
		xs = self.W_xgrid[xs]
		ys = self.W_ygrid[ys]
		
		rotmat = GetRotmat(orient)
		
		if(orient == 3):
			orient = -1
		
		theta = rel_pose[2]
		x_rel = rotmat.dot(rel_pose[:2]) # just x,y	
		
		pose = np.array([xs, ys]) + x_rel # location in the world of that ellipse
		pose = np.hstack((pose, theta + orient*np.pi/2.0 )) 
		
		return pose, rotmat
	
	# limits the commands in such a way as to preserve the ratio between them
	def LimitCmds(self, V, omega):
		
		if(V <= 0.0):
			V = 0.0
		else:	
			ratiov = np.abs(V / umax)
			ratiow = np.abs(omega / ( V/self.L *np.tan(delmax)))

			if(ratiow > ratiov and ratiow > 1.):
				omega = omega/ratiow
				V = V/ratiow
			elif(ratiov > ratiow and ratiov > 1.):
				omega = omega/ratiov
				V = V/ratiov
		
		return V, omega
	
	# gives look-ahead point to where to robot should go (and the commands it needs to take)
	def GetClosestInterpPoint(self, rel_poses, funnel):
		orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', funnel)] # extract first ellipse pose
		xs = self.W_xgrid[xs]
		ys = self.W_ygrid[ys]
		
		rotmat = GetRotmat(orient)
		
		if(orient == 3):
			orient = -1
		
		__, N = rel_poses.shape
		theta = rel_poses[2,:] 
		x_rel = np.dot(rotmat, rel_poses[:2,:])
		poses = np.array([[xs], [ys]]) + x_rel # location in the world of that ellipse
		#import pdb; pdb.set_trace()
		diff = self.pose[:2].reshape((2,1))-poses
		dist = np.linalg.norm( diff, axis=0 )

		index = np.argmin(dist, axis=0) + 0 #1 #2
		# skip the zero velocities ellipses
		while((index < N-1) and (np.abs(self.u_ref_hires[1,index]) < 0.05)):
			index += 1
		if(index > N-1):
			index = N-1
		
		pose_ref = np.hstack((poses[:,index], theta[index] + orient*np.pi/2.0 ))
		u_ref = self.u_ref_hires[:,index]

		return pose_ref, u_ref
	
	# compute the actual controls to the robot
	def control(self, K, x_0, u_0, do_calc=True):
		#if(do_calc):
		
		#print('x_0=(%f,%f,%f)\tu_0=(%f,%f)' %(x_0[0],x_0[1],x_0[2], u_0[0],u_0[1]))
		if(self.all_topics_loaded == False):
			return
		
		if(True): #do_calc == True):
			# error comes in global coordinates while the K computed in the optimization
			# assumes y-up x-right and we face right
			label = self.states[self.curr_state]
			orient, __, __ = [int(s) for s in re.findall(r'-?\d+\.?\d*', label)] # extract first ellipse pose
			rotmat = GetRotmat(orient)
			rotmat = np.linalg.inv(rotmat)
			err = self.pose - x_0
			# find the smallest angle difference
			ang_diff = (err[2] + np.pi) % (2*np.pi) - np.pi
			err_rel = rotmat.dot(err[:2])
			err_rel = np.hstack((err_rel, ang_diff )) # add the theta component
			
			#import pdb; pdb.set_trace()
			u = u_0 - K.dot(err_rel)
			#u[0] = self.LPF(u[0], self.delta_prev, tau=10.)
			#self.delta_prev = u[0]
			#if(do_calc == False):
			u[1] = self.LPF(u[1], self.u[1], tau=2.)
			self.v_prev = u[1]
			#else:
				# this is basically a reset when switching funnels
				#self.v_prev = 0.0
				#u[1] = 0.0
				
				
		
			#saturations
			if(u[1] > umax):
				u[1] = umax
			if(u[1] < 0):
				u[1] = 0.0
			if(u[0] > delmax):
				u[0] = delmax
			if(u[0] < -delmax):
				u[0] = -delmax
		
			self.u = u
			
		#else:
		#	u = self.u
		
		self.actuation(u)
	
	# send the commands to ROS
	def actuation(self, u):
		# Jackal accepts commands in linear velocity and angular velocity.
		# our model assumed u = [steering angle, linear velocity]
		# This is our model:
		# xdot = v*cos(teta)
		# ydot = v*sin(teta)
		# tetadot = v/L * tan( delta )
		# This is probably) Jackal's model:
		# xdot = v*cos(teta)
		# ydot = v*sin(teta)
		# tetadot = omega
		#
		# so we can easily see that we can get omega = v/L * tan( delta )
		t = Twist()
		#saturations
		#V,W = self.LimitCmds(u[1], u[1] / self.L * np.tan(u[0]))
		#self.u = np.array([W,V])
		#t.angular.z = W 
		t.angular.z = u[1] / self.L * np.tan(u[0]) # delta
		#t.linear.x  = V #u[1] # V
		t.linear.x  = u[1] # V
		#print('v = %f' %u[1])
		self.control_pub.publish(t)
			
if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Robot number to control.')
	parser.add_argument('--n', type=int, default=0,
					help='an integer for the robot number')
	args = parser.parse_args()
	print('Controller for Jackal%d' %args.n)

	# Create a custom logger
	log_file = 'telemetry.log'
	level = logging.DEBUG
	logger = logging.getLogger(__name__)
	#formatter = logging.Formatter('%(asctime)s - %(message)s')
	formatter = logging.Formatter('%(message)s')
	fileHandler = logging.FileHandler(log_file, mode='w')
	fileHandler.setFormatter(formatter)
	logger.setLevel(level)
	logger.addHandler(fileHandler)
	# set header of telemetry file
	logger.debug('t;frame;x;y;z;x_ref;y_ref;z_ref;delta_ref;u_ref;state;ellipse;control;action;delta;u;vx;vy;vz;wx;wy;wz;' \
				'xr;yr;zr;delr;ur')
				 
	rospy.init_node('run_jackal_%d' %args.n)#, log_level=rospy.DEBUG)
	J = Jackal(args.n)
			
	try:
		#rospy.spin()
		J.Run()
	except KeyboardInterrupt:
		print("Shutting down\n")
	
	# END ALL
