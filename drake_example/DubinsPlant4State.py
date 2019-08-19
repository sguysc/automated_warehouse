import numpy as np
import matplotlib.pyplot as plt
import math

from pydrake.all import (DirectCollocation, PiecewisePolynomial, BasicVector_, LeafSystem_, Solve, FirstOrderTaylorApproximation, LinearQuadraticRegulator, Linearize, Variables, MathematicalProgram, Evaluate, Jacobian, RealContinuousLyapunovEquation)
from pydrake.all import (plot_sublevelset_expression)
from pydrake.systems.scalar_conversion import TemplateSystem

@TemplateSystem.define("DubinsPlant_")
def DubinsPlant_(T):
	class Impl(LeafSystem_[T]):
		def _construct(self, converter=None):
			LeafSystem_[T].__init__(self, converter)
			# one inputs 
			# self.DeclareInputPort("u", PortDataType.kVectorValued, 1)
			self.DeclareVectorInputPort("u", BasicVector_[T](2))
			# three outputs (full state)
			self.DeclareVectorOutputPort("x", BasicVector_[T](4),
										 self.CopyStateOut)
			# three positions, no velocities
			self.DeclareContinuousState(4, 0, 0)

			# parameters 
			self.v = 1.0
			self.umax =  5.0
			self.omegamax =  5.0

		def _construct_copy(self, other, converter=None):
			Impl._construct(self, converter=converter)

		def CopyStateOut(self, context, output):
			x = context.get_continuous_state_vector().CopyToVector()
			output.SetFromVector(x) # = y 

		# X = [x y v theta]'
		def DoCalcTimeDerivatives(self, context, derivatives):
			x = context.get_continuous_state_vector().CopyToVector()
			u = self.EvalVectorInput(context, 0).CopyToVector()
			theta = x[3]
			v = x[2]
			
			qdot = np.array([v*np.cos(theta),  v*np.sin(theta), u[0], u[1]])#.reshape(3,1) #u[0]
			derivatives.get_mutable_vector().SetFromVector(qdot)
		
		def runDircol(self,x0,xf,tf0):
			N = np.int(tf0 * 10) # "10Hz" samples per second

			context = self.CreateDefaultContext()
			dircol  = DirectCollocation(self, context, num_time_samples=N,
							   minimum_timestep=0.1, maximum_timestep=1.0)
			u = dircol.input()
			
			dircol.AddEqualTimeIntervalsConstraints()
			
			dircol.AddConstraintToAllKnotPoints(u[0] <=  .5*self.umax)
			dircol.AddConstraintToAllKnotPoints(u[0] >= -.5*self.umax)
			dircol.AddConstraintToAllKnotPoints(u[1] <=  .5*self.omegamax)
			dircol.AddConstraintToAllKnotPoints(u[1] >= -.5*self.omegamax)
			
			eps = 0.0
			dircol.AddBoundingBoxConstraint(x0, x0, dircol.initial_state())
			dircol.AddBoundingBoxConstraint(xf-np.array([eps, eps, eps, eps]), xf+np.array([eps, eps, eps, eps]), dircol.final_state())

			R = np.diag([10.0, 10.0])  # Cost on input "effort".
			dircol.AddRunningCost( u.transpose().dot(R.dot(u)) ) #R*u[0]**2)

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
		
		# not implemented correctly yets
		def LQR(self, xtraj, utraj, Q, R, Qf):
			tspan = utraj.get_segment_times()
			
			context = self.CreateDefaultContext()
			nX = context.num_continuous_states()
			nU = self.GetInputPort('u').size()

			sym_system = self.ToSymbolic()
			sym_context = sym_system.CreateDefaultContext()
			prog = MathematicalProgram()
			x = prog.NewIndeterminates(nX,'x')
			ucon = prog.NewIndeterminates(nU,'u')
			#nY = self.GetOutputPort('x').size()
			#N = np.zeros([nX, nU])
			
			times = xtraj.get_segment_times()
			K = []
			
			import pdb; pdb.set_trace()
			for t in times:
				# option 1
				x0 = xtraj.value(t).transpose()[0]
				u0 = utraj.value(t).transpose()[0]
				
				sym_context.SetContinuousState(x0+x)
				sym_context.FixInputPort(0, u0+ucon )
				f = sym_system.EvalTimeDerivatives(sym_context).CopyToVector()
				A = Evaluate(Jacobian(f, x), dict(zip(x, x0)))
				B = Evaluate(Jacobian(f, ucon), dict(zip(ucon, u0)))
				K.append(LinearQuadraticRegulator(A, B, Q, R)) 
				
				# option 2
				#context.SetContinuousState(xtraj.value(t) )
				#context.FixInputPort(0, utraj.value(t) )
				#linearized_plant = Linearize(self, context)
				#K.append(LinearQuadraticRegulator(linearized_plant.A(),
                #                    	linearized_plant.B(),
                #                      	Q, R)) #self, context, Q, R)
				
		
			
			Kpp = PiecewisePolynomial.FirstOrderHold(times, K)
			
			return Kpp
		
		# not implemented correctly yet
		"""
		def findLyapunovFunctionSOS(self, x0, deg_V, deg_L):
			prog = MathematicalProgram()
			# Declare the "indeterminates", x. These are the variables which define the polynomials
			x = prog.NewIndeterminates(1, 'x')[0]
			y = prog.NewIndeterminates(1, 'y')[0]
			thetadot = prog.NewIndeterminates(1, 'thetadot')[0]
			X = np.array([s, c, thetadot])
			# Construct a polynomial V that contains all monomials with s,c,thetadot up to degree n.
			V = prog.NewFreePolynomial(Variables(X), deg_V).ToExpression()
			eps = 1e-4
			constraint1 = prog.AddSosConstraint(V - eps*(X-x0).dot(X-x0)) # constraint to enforce that V is strictly positive away from x0.
			Vdot = V.Jacobian(X).dot(f) # Construct the polynomial which is the time derivative of V
			L = prog.NewFreePolynomial(Variables(X), deg_L).ToExpression() # Construct a polynomial L representing the "Lagrange multiplier".
			constraint2 = prog.AddSosConstraint(-Vdot - L*(x**2+y**2-1) -
                                    eps*(X-x0).dot(X-x0)*y**2) # Add a constraint that Vdot is strictly negative away from x0
			# Add V(0) = 0 constraint
			constraint3 = prog.AddLinearConstraint(V.Substitute({y: 0, x: 1, thetadot: 0}) == 0)
			# Add V(theta=xxx) = 1, just to set the scale.
			constraint4 = prog.AddLinearConstraint(V.Substitute({y: 1, x: 0, thetadot: 0}) == 1)
			# Call the solver.
			result = Solve(prog)
			Vsol = Polynomial(result.GetSolution(V))
			return Vsol
		"""
		
		def EvalTimeDerivatives(self, context):
			#import pdb; pdb.set_trace()
			y = context.get_continuous_state_vector().Clone()
			x = context.get_continuous_state_vector().CopyToVector()
			
			# u = [0.0, 0.0] #self.EvalVectorInput(context, 0).CopyToVector()
			u = self.GetInputPort('u').EvalBasicVector(context).CopyToVector()
			
			# omega = [0.0]
			v = x[2]
			theta = x[3]
			
			qdot = np.array([v*np.cos(theta),  v*np.sin(theta), u[0], u[1]])#.reshape(3,1) #u[0]
			# import pdb; pdb.set_trace()
			y.SetFromVector(qdot)
			return y
		
		def isPositiveDefinite(self, M):
			assert np.size(M,0) == np.size(M, 1)

			for i in range(1, np.size(M,0)+1):
				if(np.linalg.det(M[0:i, 0:i]) <= 0):
					return False;
			return True

		def FixedLyapunovSearchRho(self, prog, x, V, Vdot, multiplier_degree=None):
			'''
			Assumes V>0.
			V <= rho => Vdot <= 0 via
				find Lambda subject to
				  -Vdot + Lambda*(V - rho) is SOS, 
				  Lambda is SOS.
			'''

			def CheckLevelSet(prev_x, prev_V, prev_Vdot, rho, multiplier_degree):
				prog = MathematicalProgram()
				x = prog.NewIndeterminates(len(prev_x),'x')
				V = prev_V.Substitute(dict(zip(prev_x, x)))
				Vdot = prev_Vdot.Substitute(dict(zip(prev_x, x)))
				slack = prog.NewContinuousVariables(1)[0]

				Lambda = prog.NewSosPolynomial(Variables(x), multiplier_degree)[0].ToExpression()

				prog.AddSosConstraint(-Vdot + Lambda*(V - rho) - slack*V)
				prog.AddCost(-slack)

				result = Solve(prog)
				assert result.is_success()
				return result.GetSolution(slack)


			if multiplier_degree is None:
				# There are no guarantees... this is a reasonable guess:
				multiplier_degree = Polynomial(Vdot).TotalDegree()
				print "Using a degree " + str(multiplier_degree) + " multiplier for the S-procedure"

			rhomin = 0.
			rhomax = 1.

			# First bracket the solution
			while CheckLevelSet(x, V, Vdot, rhomax, multiplier_degree) > 0:
				rhomin = rhomax
				rhomax = 1.2*rhomax

			# TODO(russt): Could use a continuous (black-box) optimization (e.g. we used fzero in MATLAB)
			# TODO(russt): Make the tolerance an option?
			tolerance = 1e-4
			while rhomax - rhomin > tolerance:
				rho = (rhomin + rhomax)/2
				if CheckLevelSet(x, V, Vdot, rho, multiplier_degree) >= 0:
					rhomin = rho
				else:
					rhomax = rho

			rho = (rhomin + rhomax)/2
			return V/rho

		def FixedLyapunovMaximizeLevelSet(self, prog, x, V, Vdot, multiplier_degree=None):
			'''
			Assumes V>0.
			Vdot >= 0 => V >= rho (or x=0) via 
				maximize rho subject to
				  (V-rho)*x'*x - Lambda*Vdot is SOS, 
				  Lambda is SOS.
			'''

			if multiplier_degree is None:
				# There are no guarantees... this is a reasonable guess:
				multiplier_degree = Polynomial(Vdot).TotalDegree()
				print "Using a degree " + str(multiplier_degree) + " multiplier for the S-procedure"

			# TODO(russt): implement numerical "balancing" from matlab version.
			Lambda = prog.NewSosPolynomial(Variables(x), multiplier_degree)[0].ToExpression()

			rho = prog.NewContinuousVariables(1, "rho")[0]
			prog.AddSosConstraint((V-rho)*(x.dot(x)) - Lambda*Vdot)

			prog.AddCost(-rho)

		#    mosek = MosekSolver()
		#    mosek.set_stream_logging(True, 'mosek.out')
		#    result = mosek.Solve(prog, None, None)
			result = Solve(prog)

			print "Using " + result.get_solver_id().name()

			assert result.is_success()
			assert result.GetSolution(rho) > 0, "Optimization failed (rho <= 0)."

			return V/result.GetSolution(rho)


		def RegionOfAttraction(self, context, V=None):
			x0 = context.get_continuous_state_vector().CopyToVector()

			# Check that x0 is a fixed point.
			xdot0 = self.EvalTimeDerivatives(context).CopyToVector()
			assert np.allclose(xdot0, 0*xdot0), "context does not describe a fixed point."

			import pdb; pdb.set_trace()
			sym_system = self.ToSymbolic()
			sym_context = sym_system.CreateDefaultContext()

			prog = MathematicalProgram()
			x = prog.NewIndeterminates(sym_context.num_continuous_states(),'x')

			# Evaluate the dynamics (in relative coordinates)
			sym_context.SetContinuousState(x0+x)
			f = sym_system.EvalTimeDerivatives(sym_context).CopyToVector()

			if V is None:
				# Solve a Lyapunov equation to find the Lyapunov candidate.
				A = Evaluate(Jacobian(f, x), dict(zip(x, x0)))
				Q = np.eye(sym_context.num_continuous_states())
				import pdb; pdb.set_trace()
				P = RealContinuousLyapunovEquation(A, Q)
				V = x.dot(P.dot(x))

			Vdot = V.Jacobian(x).dot(f)

			# Check Hessian of Vdot at origin
			H = Evaluate(0.5*Jacobian(Vdot.Jacobian(x),x), dict(zip(x, x0)))
			assert isPositiveDefinite(-H), "Vdot is not negative definite at the fixed point."

			#V = FixedLyapunovMaximizeLevelSet(prog, x, V, Vdot)
			V = self.FixedLyapunovSearchRho(prog, x, V, Vdot)

			# Put V back into global coordinates
			V = V.Substitute(dict(zip(x,x-x0)))
			return V

	return Impl

	
def runFunnel():
	print('running Funnel algorithm ...')
	vconst = 2.0
	umax = 10000.0
	
    # Declare (Dubins car) model
	plant = DubinsPlant_[float]() #None]  # Default instantiation
		
	# Trajectory optimization
	x0 = (0.0, 0.0, 0.0, -math.pi/4.0)  #Initial state that trajectory should start from
	xf = (1.5, 0.0, 0.0, -math.pi/4.0)  #Final desired state
	tf0 = 5.0 # Guess for how long trajectory should take
	utraj, xtraj = plant.runDircol(x0, xf, tf0)

	# region of attraction:
	#context = plant.CreateDefaultContext()
	#context.SetContinuousState(xf)
	#context.FixInputPort(0, [0.0, 0.0])
	#V = plant.RegionOfAttraction(context)
	
	#print('V=')
	#print(V)
	
	# Do tvlqr
	Q = 3.0*np.eye(len(x0))
	R = 1.0*np.eye(2)
	Qf = 1.0*Q
	K = plant.LQR(xtraj, utraj, Q, R, Qf)  # c, V 
	# poly = FirstOrderTaylorApproximation(feedback(p,c),xtraj,[],3)
	# ts = xtraj.getBreaks()

	
	## simulation for debugging
	#sim = Simulator(plant)
	#context = sim.get_mutable_context()
	#context.SetTime(0.)
	#context.SetContinuousState(x0)  #np.column_stack([0.9]))
	#sim.StepTo(10) #AdvanceTo
	#xend = context.get_continuous_state().get_vector().CopyToVector()
	#print('x = %f' % (xend[0]))
	
	# resolve the trajectory piecewise polynomial structure
	N = 100 # number of points to sample
	times = np.linspace(utraj.start_time(), utraj.end_time(), N)
	#u_lookup = np.vectorize(utraj.value)
	#import pdb; pdb.set_trace()
	#u_values = u_lookup(times)
	u_values = np.hstack([utraj.value(t) for t in times])
	xy_knots = np.hstack([xtraj.value(t) for t in times])

	# plotting stuff
	fig, ax = plt.subplots(3)
	fig.suptitle('Direct collocation trajectory optimization (%.2f,%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f,%.2f)' %\
				 (x0[0],x0[1],x0[2],x0[3],xf[0],xf[1],xf[2],xf[3]))
	ax[0].plot(xy_knots[0, :], xy_knots[1, :])
	ax[0].set(xlabel='x [m]', ylabel='y [m]')
	ax[0].grid(True)
	ax[1].plot(times, u_values[0,:], 'green')
	ax[1].plot(times, u_values[1,:], 'red')
	ax[1].set(xlabel='t [sec]', ylabel='v [m/s] or omega [rad/sec]')
	ax[1].grid(True)
	#plot_sublevelset_expression(ax[2], V)
	plt.show()
	
	
if __name__ == "__main__":
	
	runFunnel()
	
	
