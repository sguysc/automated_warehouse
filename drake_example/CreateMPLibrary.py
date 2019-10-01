import numpy as np
import matplotlib.pyplot as plt
import math
import pickle

from DubinsPlantCar import *


# main function, finding the maximal funnel
def CreateFunnels():
	print('******\ncreating Motion primitives (Funnel algorithm) ...\n******')
	
	# (x,y,theta)
	
	MotionPrimitives = { \
			0: {'s': (0.0, 0.0, 0.0), 'e': (6.0,  0.0,  0.0)}, \
			1: {'s': (0.0, 0.0, 0.0), 'e': (3.0,  0.0,  0.0)}, \
			2: {'s': (0.0, 0.0, 0.0), 'e': (3.0,  2.0,  0.0*math.pi/180.0)}, \
			3: {'s': (0.0, 0.0, 0.0), 'e': (3.0, -2.0, -0.0*math.pi/180.0)}, \
			4: {'s': (0.0, 0.0, 0.0), 'e': (0.0,  4.0,  180.0*math.pi/180.0)}, \
			5: {'s': (0.0, 0.0, 0.0), 'e': (0.0, -4.0, -180.0*math.pi/180.0)}, \
	}
	#dbfile = open('MPLibrary.lib', 'rb')
	#MotionPrimitives = pickle.load(dbfile)
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
	ax1.set_xlim([-2.0, 5.0])
	ax1.set_ylim([-5.0, 5.0])
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
		tf0 = 4.0 # Guess for how long trajectory should take
		utraj, xtraj = plant.runDircol(x0, xf, tf0)
		print('Trajectory takes %f[sec]' %(utraj.end_time()))
		print('Done\n******')
		# resolve the trajectory piecewise polynomial structure
		N = 100 # number of points to sample
		times = np.linspace(utraj.start_time(), utraj.end_time(), N)
		#u_lookup = np.vectorize(utraj.value)
		#u_values = u_lookup(times)
		u_values = np.hstack([utraj.value(t) for t in times])
		xy_knots = np.hstack([xtraj.value(t) for t in times])
		#
		# plotting stuff
		
		#ax_traj[0].plot(xy_knots[0, :], xy_knots[1, :])
		#ax_traj[0].grid(True)
		#ax_traj[1].plot(times, u_values[0,:], 'green')
		#ax_traj[1].plot(times, u_values[1,:], 'red')
		#ax_traj[1].set(xlabel='t [sec]', ylabel='v [m/s] or omega [rad/sec]')
		#ax_traj[1].grid(True)
		#plt.pause(0.05)

		V, K = plant.RegionOfAttraction(xtraj, utraj, debug=False)
		
		times = xtraj.get_segment_times()
		for i in range(len(times)):
			x0 = xtraj.value(times[i]).transpose()[0]
			plant.my_plot_sublevelset_expression(ax1, V[i], x0, color=(0.1, idx/10.+.2,0.8))
			
		ax1.plot(xy_knots[0, :], xy_knots[1, :], 'w')
		plt.pause(0.05)
		#plt.show(block = False)
		#motion_library.append(V)
		mp.update({'V': V})
		mp.update({'K': K})
		mp.update({'xtraj': xtraj})
		mp.update({'utraj': utraj})
		mp.update({'t': times})

	import pdb; pdb.set_trace()
	dbfile = open('MPLibrary.lib', 'wb')
	pickle.dump(MotionPrimitives, dbfile)
	dbfile.close()
	
	plt.show(block = True)
	
if __name__ == "__main__":
	CreateFunnels()


