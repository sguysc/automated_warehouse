import numpy as np
import matplotlib.pyplot as plt
import math
import dill
from pydrake.all import (PiecewisePolynomial, Jacobian, Evaluate)
from pydrake.symbolic import Expression

from shapely.geometry import Polygon, box

from DubinsPlantCar import *


def uturn_motion(x0, xf, tf0, N):
	tguess = np.linspace(0.0, tf0, N)
	teta   = np.linspace(x0[2]-xf[2], xf[2]-x0[2], N)
	xcenter= ( (xf[0]-x0[0])/2.0, (xf[1]-x0[1])/2.0 )
	xguess = xcenter[0] + np.cos(teta)
	yguess = xcenter[1] + np.sin(teta)
	guess = []
	for k in range(N):
		guess.append( (xguess[k], yguess[k], teta[k]) )
	guess = np.column_stack(guess)

	return tguess, guess

# main function, finding the maximal funnel
def CreateFunnels():
	print('******\nCreating Motion primitives (Funnel algorithm) ...\n******')
	
	# (x,y,theta)
	# Jackal - Lab cell 0.4[m]
	MotionPrimitives = { \
			0: {'s': (0.0, 0.0, 0.0), 'e': (CELL_SIZE*4.,  0.00,  0.0), 'kp': 8, 'ex_kp': 8, 'traj': None}, \
			1: {'s': (0.0, 0.0, 0.0), 'e': (CELL_SIZE*2.,  0.00,  0.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			2: {'s': (0.0, 0.0, 0.0), 'e': (CELL_SIZE*1.,  0.00,  0.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			3: {'s': (0.0, 0.0, 0.0), 'e': (CELL_SIZE*3.,  CELL_SIZE,  0.0*math.pi/180.0), 'kp': 4, 'ex_kp': 5, 'traj': None}, \
			4: {'s': (0.0, 0.0, 0.0), 'e': (CELL_SIZE*3., -CELL_SIZE,  0.0*math.pi/180.0), 'kp': 4, 'ex_kp': 5, 'traj': None}, \
			5: {'s': (0.0, 0.0, 0.0), 'e': (CELL_SIZE*2.,  CELL_SIZE*2.,  90.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			6: {'s': (0.0, 0.0, 0.0), 'e': (CELL_SIZE*2., -CELL_SIZE*2., -90.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			7: {'s': (0.0, 0.0, 0.0), 'e': (0.00,  CELL_SIZE*3.,  180.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': uturn_motion }, \
			8: {'s': (0.0, 0.0, 0.0), 'e': (0.00, -CELL_SIZE*3., -180.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': uturn_motion }, \
	}
	''' # Jackal - Lab cell 0.4[m]
	MotionPrimitives = { \
			0: {'s': (0.0, 0.0, 0.0), 'e': (1.60,  0.00,  0.0), 'kp': 8, 'ex_kp': 8, 'traj': None}, \
			1: {'s': (0.0, 0.0, 0.0), 'e': (0.80,  0.00,  0.0), 'kp': 8, 'ex_kp': 8, 'traj': None}, \
			2: {'s': (0.0, 0.0, 0.0), 'e': (0.40,  0.00,  0.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			3: {'s': (0.0, 0.0, 0.0), 'e': (1.20,  0.40,  0.0*math.pi/180.0), 'kp': 3, 'ex_kp': 4, 'traj': None}, \
			4: {'s': (0.0, 0.0, 0.0), 'e': (1.20, -0.40,  0.0*math.pi/180.0), 'kp': 3, 'ex_kp': 4, 'traj': None}, \
			5: {'s': (0.0, 0.0, 0.0), 'e': (0.80,  0.80,  90.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			6: {'s': (0.0, 0.0, 0.0), 'e': (0.80, -0.80, -90.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			7: {'s': (0.0, 0.0, 0.0), 'e': (0.00,  0.80,  180.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': uturn_motion }, \
			8: {'s': (0.0, 0.0, 0.0), 'e': (0.00, -0.80, -180.0*math.pi/180.0), 'kp': 4, 'ex_kp': 4, 'traj': uturn_motion }, \
	}
	'''
	'''	# Jackal - Gazebo
	MotionPrimitives = { \
			0: {'s': (0.0, 0.0, 0.0), 'e': (5.00,  0.00,  0.0), 'kp': 12, 'ex_kp': 12, 'traj': None}, \
			1: {'s': (0.0, 0.0, 0.0), 'e': (2.50,  0.00,  0.0), 'kp': 8, 'ex_kp': 8, 'traj': None}, \
			2: {'s': (0.0, 0.0, 0.0), 'e': (1.25,  0.00,  0.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			3: {'s': (0.0, 0.0, 0.0), 'e': (3.75,  1.25,  0.0*math.pi/180.0), 'kp': 7, 'ex_kp': 7, 'traj': None}, \
			4: {'s': (0.0, 0.0, 0.0), 'e': (3.75, -1.25,  0.0*math.pi/180.0), 'kp': 6, 'ex_kp': 7, 'traj': None}, \
			5: {'s': (0.0, 0.0, 0.0), 'e': (2.50,  2.50,  90.0*math.pi/180.0), 'kp': 5, 'ex_kp': 5, 'traj': None}, \
			6: {'s': (0.0, 0.0, 0.0), 'e': (2.50, -2.50, -90.0*math.pi/180.0), 'kp': 5, 'ex_kp': 5, 'traj': None}, \
			7: {'s': (0.0, 0.0, 0.0), 'e': (0.00,  2.50,  180.0*math.pi/180.0), 'kp': 6, 'ex_kp': 6, 'traj': uturn_motion }, \
			8: {'s': (0.0, 0.0, 0.0), 'e': (0.00, -2.50, -180.0*math.pi/180.0), 'kp': 6, 'ex_kp': 6, 'traj': uturn_motion }, \
	}
	'''
	''' # forklift
	MotionPrimitives = { \
			0: {'s': (0.0, 0.0, 0.0), 'e': (5.00,  0.00,  0.0), 'kp': 12, 'ex_kp': 12, 'traj': None}, \
			1: {'s': (0.0, 0.0, 0.0), 'e': (2.50,  0.00,  0.0), 'kp': 8, 'ex_kp': 8, 'traj': None}, \
			2: {'s': (0.0, 0.0, 0.0), 'e': (1.25,  0.00,  0.0), 'kp': 4, 'ex_kp': 4, 'traj': None}, \
			3: {'s': (0.0, 0.0, 0.0), 'e': (3.75,  1.25,  0.0*math.pi/180.0), 'kp': 5, 'ex_kp': 6, 'traj': None}, \
			4: {'s': (0.0, 0.0, 0.0), 'e': (3.75, -1.25,  0.0*math.pi/180.0), 'kp': 6, 'ex_kp': 6, 'traj': None}, \
			5: {'s': (0.0, 0.0, 0.0), 'e': (2.50,  2.50,  90.0*math.pi/180.0), 'kp': 6, 'ex_kp': 6, 'traj': None}, \
			6: {'s': (0.0, 0.0, 0.0), 'e': (2.50, -2.50, -90.0*math.pi/180.0), 'kp': 6, 'ex_kp': 6, 'traj': None}, \
			7: {'s': (0.0, 0.0, 0.0), 'e': (0.00,  2.50,  180.0*math.pi/180.0), 'kp': 5, 'ex_kp': 7, 'traj': uturn_motion }, \
			8: {'s': (0.0, 0.0, 0.0), 'e': (0.00, -2.50, -180.0*math.pi/180.0), 'kp': 5, 'ex_kp': 7, 'traj': uturn_motion }, \
	}
	'''
	#dbfile = open('MPLibrary.lib', 'rb')
	#MotionPrimitives = dill.load(dbfile)
	#dbfile.close()
	'''
	MotionPrimitives = { \
			0: {'s': (0.0, 0.0, 0.0), 'e': (0.0,  4.0,  180.0*math.pi/180.0)}, \
	}
	'''
	
	#motion_library = []
					   
	#fig_traj, ax_traj = plt.subplots()
	fig1, ax1 = plt.subplots()
	#fig_traj.suptitle('Direct collocation trajectory optimization')
	#ax_traj[0].set(xlabel='x [m]', ylabel='y [m]')
	ax1.set_xlim([-1.0, CELL_SIZE*4.0+1.0])
	ax1.set_ylim([-CELL_SIZE*4, CELL_SIZE*4])
	ax1.set_xlabel('x')
	ax1.set_ylabel('y')
	fig1.suptitle('Car: Funnel for trajectory')

	plt.show(block = False)
	plt.pause(0.05)
	
    # Declare pendulum model
	plant = DubinsCarPlant_[float]() #None]  # Default instantiation
	
	for idx, mp in MotionPrimitives.items():
		print('Computing motion primitive %d/%d ...' %(idx+1, len(MotionPrimitives)))
		#import pdb; pdb.set_trace()
		# Trajectory optimization to get nominal trajectory
		x0 = mp['s']  #Initial state that trajectory should start from
		xf = mp['e']  #Final desired state
		plant.knotPoints = mp['kp']
		plant.ex_knots = mp['ex_kp']
		suggest_trajectory = mp['traj']
		dist = np.linalg.norm(np.array(xf)-np.array(x0))
		tf0  = dist/(plant.umax*0.8) # Guess for how long trajectory should take
		if(suggest_trajectory):
			tguess, guess = suggest_trajectory(x0, xf, tf0, mp['kp'])
			plant.initial_x_trajectory = \
				PiecewisePolynomial.FirstOrderHold(tguess, guess)

		
		utraj, xtraj = plant.runDircol(x0, xf, tf0)
		print('Trajectory takes %f[sec]' %(utraj.end_time()))
		print('Done\n******')
	
		V, K = plant.RegionOfAttraction(xtraj, utraj, debug=False, find_V=True)
		
		#times = xtraj.get_segment_times()
		times = np.linspace(utraj.start_time(), utraj.end_time(), len(V))
		xcenter = []
		unom    = []
		for i in range(len(times)):
			x0 = xtraj.value(times[i]).transpose()[0]
			xcenter.append(x0)
			unom.append(utraj.value(times[i]).transpose()[0])
			plant.my_plot_sublevelset_expression(ax1, V[i], x0, color=(0.1, idx/10.+.2,0.8))
		
		x = list(V[0].GetVariables())
		env = {a: 0 for a in x}
		V_matrix = []
		for roa in V:
			V_matrix.append( Evaluate(0.5*Jacobian(roa.Jacobian(x),x), env) )
		
		mp.update({'V': V_matrix})
		mp.update({'K': K})
		mp.update({'xcenter': xcenter})
		mp.update({'unom': unom})
		#mp.update({'utraj': utraj})
		#mp.update({'t': times})

		# resolve the trajectory piecewise polynomial structure
		N = 100 # number of points to sample
		times = np.linspace(utraj.start_time(), utraj.end_time(), N)
		#u_lookup = np.vectorize(utraj.value)
		#u_values = u_lookup(times)
		u_values = np.hstack([utraj.value(t) for t in times])
		xy_knots = np.hstack([xtraj.value(t) for t in times])
		
		mp.update({'xtraj': xy_knots})
		mp.update({'utraj': u_values})
		mp.update({'t'    : times})
		mp.pop('traj') # remove this function handles
		
		ax1.plot(xy_knots[0, :], xy_knots[1, :], 'k')
		plt.pause(0.05)
		#plt.show(block = False)

	# REMEMBER TO COMMENT THIS IF THE PRIMITIVES ARE GOOD TO BEGIN WITH!!!!
	MotionPrimitives = FixBadPrimitives(MotionPrimitives, 7, 8)
	MotionPrimitives = FixBadPrimitives(MotionPrimitives, 3, 4)
	
	import pdb; pdb.set_trace()
	dbfile = open('MPLibrary.lib', 'wb')
	dill.dump(MotionPrimitives, dbfile)
	dbfile.close()
	print('Done computing ...')
	plt.show(block = True)

def FixBadPrimitives(MotionPrimitives, fix_this, with_this):	
	#import pdb; pdb.set_trace()
	N = len(MotionPrimitives[fix_this]['xcenter'])
	for i in range(N):
		MotionPrimitives[fix_this]['xcenter'][i][0] = MotionPrimitives[with_this]['xcenter'][i][0] #x
		MotionPrimitives[fix_this]['xcenter'][i][1] = -1.0*MotionPrimitives[with_this]['xcenter'][i][1] #y
		MotionPrimitives[fix_this]['xcenter'][i][2] = -1.0*MotionPrimitives[with_this]['xcenter'][i][2] #theta
		MotionPrimitives[fix_this]['unom'][i][0]    = -1.0*MotionPrimitives[with_this]['xcenter'][i][0] #delta
		MotionPrimitives[fix_this]['unom'][i][1]    = MotionPrimitives[with_this]['xcenter'][i][1] #v
		# dirty rotation of the matrix about theta
		MotionPrimitives[fix_this]['V'][i][0][0]    = MotionPrimitives[with_this]['V'][i][0][0] #V
		MotionPrimitives[fix_this]['V'][i][0][1]    = -1.0*MotionPrimitives[with_this]['V'][i][0][1] #V
		MotionPrimitives[fix_this]['V'][i][0][2]    = -1.0*MotionPrimitives[with_this]['V'][i][0][2] #V
		MotionPrimitives[fix_this]['V'][i][1][0]    = -1.0*MotionPrimitives[with_this]['V'][i][1][0] #V
		MotionPrimitives[fix_this]['V'][i][1][1]    = MotionPrimitives[with_this]['V'][i][1][1] #V
		MotionPrimitives[fix_this]['V'][i][1][2]    = MotionPrimitives[with_this]['V'][i][1][2] #V
		MotionPrimitives[fix_this]['V'][i][2][0]    = -1.0*MotionPrimitives[with_this]['V'][i][2][0] #V
		MotionPrimitives[fix_this]['V'][i][2][1]    = MotionPrimitives[with_this]['V'][i][2][1] #V
		MotionPrimitives[fix_this]['V'][i][2][2]    = MotionPrimitives[with_this]['V'][i][2][2] #V
		
		MotionPrimitives[fix_this]['K'][i]          = MotionPrimitives[with_this]['K'][i] #K
		MotionPrimitives[fix_this]['K'][i][0][0]    = -MotionPrimitives[with_this]['K'][i][0][0] #K
		MotionPrimitives[fix_this]['K'][i][1][1]    = -MotionPrimitives[with_this]['K'][i][1][1] #K
		MotionPrimitives[fix_this]['K'][i][1][2]    = -MotionPrimitives[with_this]['K'][i][1][2] #K
	
	#import pdb; pdb.set_trace()
	__, N = MotionPrimitives[fix_this]['xtraj'].shape
	for i in range(N):
		MotionPrimitives[fix_this]['xtraj'][0][i] = MotionPrimitives[with_this]['xtraj'][0][i] #x
		MotionPrimitives[fix_this]['xtraj'][1][i] = -1.0*MotionPrimitives[with_this]['xtraj'][1][i] #y
		MotionPrimitives[fix_this]['xtraj'][2][i] = -1.0*MotionPrimitives[with_this]['xtraj'][2][i] #theta
		MotionPrimitives[fix_this]['utraj'][0][i] = -1.0*MotionPrimitives[with_this]['utraj'][0][i] #delta
		MotionPrimitives[fix_this]['utraj'][1][i] = MotionPrimitives[with_this]['utraj'][1][i] #v
		MotionPrimitives[fix_this]['t'][i] = MotionPrimitives[with_this]['t'][i] #t
		
	return MotionPrimitives

def PlotFunnels():
	dbfile = open('MPLibrary.lib', 'rb')
	MotionPrimitives = dill.load(dbfile)
	dbfile.close()
	
	fig1, ax1 = plt.subplots()
	#fig_traj.suptitle('Direct collocation trajectory optimization')
	#ax_traj[0].set(xlabel='x [m]', ylabel='y [m]')
	ax1.set_xlim([-1.0, CELL_SIZE*4.0+1.0])
	ax1.set_ylim([-CELL_SIZE*4, CELL_SIZE*4])
	ax1.set_xlabel('x')
	ax1.set_ylabel('y')
	fig1.suptitle('Car: Funnel for trajectory')

	plt.show(block = False)
	plt.pause(0.05)
	#import pdb; pdb.set_trace()
	# Declare pendulum model
	plant = DubinsCarPlant_[float]() # Default instantiation
	
	for idx, mp in MotionPrimitives.items():
		N = len(mp['xcenter'])
		vertices = 50
		for i in range(N):
			#plant.my_plot_sublevelset_expression(ax1, mp['V'][i], mp['xcenter'][i], color=(0.1, idx/10.+.2,0.8))
			color=(0.1, idx/10.+.2,0.8)
			x0 =  mp['xcenter'][i]
			# simple projection to 2D assuming we want to plot on (x1,x2)
			A = mp['V'][i][0:2,0:2]
			#Plots the 2D ellipse representing x'Ax + b'x + c <= 1, e.g.
    		#the one sub-level set of a quadratic form.
			H = .5*(A+A.T)
			xmin = np.linalg.solve(-2*H, np.zeros((2, 1)))
			fmin = -xmin.T.dot(H).dot(xmin)   # since b = -2*H*xmin
			assert fmin <= 1, "The minimum value is > 1; there is no sub-level set " \
                      "to plot"

			# To plot the contour at f = (x-xmin)'H(x-xmin) + fmin = 1,
			# we make a circle of values y, such that: y'y = 1-fmin,
			th = np.linspace(0, 2*np.pi, vertices)
			Y = np.sqrt(1-fmin)*np.vstack([np.sin(th), np.cos(th)])
			# then choose L'*(x - xmin) = y, where H = LL'.
			L = np.linalg.cholesky(H)
			X = np.tile(xmin, vertices) + np.linalg.inv(np.transpose(L)).dot(Y)

			ax1.fill(X[0, :]+x0[0], X[1, :]+x0[1], color=color)
			
	plt.show(block = True)

if __name__ == "__main__":
	CreateFunnels()
	#PlotFunnels()


