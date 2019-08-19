import numpy as np

from pydrake.systems.framework import VectorSystem
from pydrake.all import (DirectCollocation, PiecewisePolynomial, Solve)

# Define the system.
class ex1(VectorSystem):
	def __init__(self):
		VectorSystem.__init__(self, 
            1,                           # 1 input.
            2)                           # 2 outputs.
		self.DeclareContinuousState(2)   # 2 state variable.
		
	# xdot(t) = -x(t) - y(t); ydot(t) = -y(t) - x(t) + u
	def DoCalcVectorTimeDerivatives(self, context, u, x, xdot):
		xdot[:] = np.array([-x[0] - x[1],  -x[1] - x[0] + u[0]]) #.reshape(3,1) #u[0]

	# y(t) = x(t)
	def DoCalcVectorOutput(self, context, u, x, y):
		y[:] = x
		
	def runDircol(self, x0, xf, tf0):
		N = 11
		umax = 10.

		context = self.CreateDefaultContext()
		dircol  = DirectCollocation(self, context, num_time_samples=N,
						   minimum_timestep=0.1, maximum_timestep=1.0)
		u = dircol.input()
		dircol.AddEqualTimeIntervalsConstraints()
		dircol.AddConstraintToAllKnotPoints(u[0] <=  .5*umax)
		dircol.AddConstraintToAllKnotPoints(u[0] >= -.5*umax)
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
	
	
if __name__ == "__main__":
	# Declare model
	plant = ex1()  # Default instantiation
		
	# Trajectory optimization
	x0 = (0.0,0.0)  #Initial state that trajectory should start from
	xf = (1.0,1.0)  #Final desired state
	tf0 = 0.5       # Guess for how long trajectory should take
	utraj, xtraj = plant.runDircol(x0, xf, tf0)
	
