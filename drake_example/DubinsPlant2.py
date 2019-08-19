import numpy as np
import matplotlib.pyplot as plt
import math

from pydrake.systems.framework import VectorSystem
from pydrake.all import (DirectCollocation)

# Define the system.
class DubinsCar2(VectorSystem):
	def __init__(self):
		VectorSystem.__init__(self, 
            1,                           # 1 input.
            3)                           # 3 outputs.
		self.DeclareContinuousState(3)   # 3 state variable.
		
		# parameters 
		self.v = 1.0
		self.umax =  5.0

	# xdot(t) = -x(t) + x^3(t)
	def DoCalcVectorTimeDerivatives(self, context, u, x, xdot):
		theta = x[2]
		xdot[:] = np.array([self.v*np.cos(theta),  self.v*np.sin(theta), u[0]]) #.reshape(3,1) #u[0]

	# y(t) = x(t)
	def DoCalcVectorOutput(self, context, u, x, y):
		y[:] = x
		
	def runDircol(self, x0, xf, tf0):
		N = 11

		context = self.CreateDefaultContext()
		dircol  = DirectCollocation(self, context, num_time_samples=N,
						   minimum_timestep=0.1, maximum_timestep=1.0)
		u = dircol.input()
		dircol.AddEqualTimeIntervalsConstraints()
		dircol.AddConstraintToAllKnotPoints(u[0] <=  .5*self.umax)
		dircol.AddConstraintToAllKnotPoints(u[0] >= -.5*self.umax)
		dircol.AddBoundingBoxConstraint(x0, x0, dircol.initial_state())
		dircol.AddBoundingBoxConstraint(xf, xf, dircol.final_state())

		R = 10.0  # Cost on input "effort".
		dircol.AddRunningCost(R*u[0]**2)

		# Add a final cost equal to the total duration.
		dircol.AddFinalCost(dircol.time())

		initial_x_trajectory = \
			PiecewisePolynomial.FirstOrderHold([0., tf0], np.column_stack((x0, xf)))
		dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

		result = Solve(dircol)
		print(result.get_solver_id().name())
		print(result.get_solution_result())
		assert(result.is_success())

		#import pdb; pdb.set_trace()
		xtraj = dircol.ReconstructStateTrajectory(result)
		utraj = dircol.ReconstructInputTrajectory(result)

		return utraj,xtraj
	
	
def runFunnel():
	print('running Funnel algorithm ...')
	vconst = 2.0
	umax = 10000.0
	
    # Declare (Dubins car) model
	plant = DubinsCar2()  # Default instantiation
		
	# Trajectory optimization
	x0 = (0.0,0.0,-math.pi/4.0)  #Initial state that trajectory should start from
	xf = (1.5,0.0, math.pi/4.0)  #Final desired state
	tf0 = 0.5 # Guess for how long trajectory should take
	utraj, xtraj = plant.runDircol(x0, xf, tf0)

	# resolve the trajectory piecewise polynomial structure
	N = 100 # number of points to sample
	times = np.linspace(utraj.start_time(), utraj.end_time(), N)
	u_lookup = np.vectorize(utraj.value)
	u_values = u_lookup(times)
	xy_knots = np.hstack([xtraj.value(t) for t in times])

	# plotting stuff
	fig, ax = plt.subplots(3)
	fig.suptitle('Direct collocation trajectory optimization (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)' %(x0[0],x0[1],x0[2],xf[0],xf[1],xf[2]))
	ax[0].plot(xy_knots[0, :], xy_knots[1, :])
	ax[0].set(xlabel='x [m]', ylabel='y [m]')
	ax[0].grid(True)
	ax[1].plot(times, u_values, 'green')
	ax[1].set(xlabel='t [sec]', ylabel='u [rad/sec]')
	ax[1].grid(True)
	plot_sublevelset_expression(ax[2], V)
	plt.show()
	
	
if __name__ == "__main__":
	
	runFunnel()
	
