import numpy as np
import matplotlib.pyplot as plt
import math

from pydrake.all import (DirectCollocation, PiecewisePolynomial, BasicVector_, LeafSystem_, Simulator, Solve, FirstOrderTaylorApproximation)
from pydrake.all import (DiagramBuilder, SignalLogger, Saturation, PortDataType, plot_sublevelset_expression)
from pydrake.systems.scalar_conversion import TemplateSystem

@TemplateSystem.define("DubinsPlant_")
def DubinsPlant_(T):
	class Impl(LeafSystem_[T]):
		def _construct(self, converter=None):
			LeafSystem_[T].__init__(self, converter)
			# one inputs 
			self.DeclareInputPort("u", PortDataType.kVectorValued, 1)
			# self.DeclareVectorInputPort("u", BasicVector_[T](1))
			# three outputs (full state)
			self.DeclareVectorOutputPort("x", BasicVector_[T](3),
										 self.CopyStateOut)
			# three positions, no velocities
			self.DeclareContinuousState(3, 0, 0)

			# parameters 
			self.v = 1.0
			self.umax =  5.0

		def _construct_copy(self, other, converter=None):
			Impl._construct(self, converter=converter)

		def CopyStateOut(self, context, output):
			x = context.get_continuous_state_vector().CopyToVector()
			output.SetFromVector(x) # = y 

		def DoCalcTimeDerivatives(self, context, derivatives):
			x = context.get_continuous_state_vector().CopyToVector()
			omega = self.EvalVectorInput(context, 0).CopyToVector()
			theta = x[2]
			
			qdot = np.array([self.v*np.cos(theta),  self.v*np.sin(theta), 1.0*omega[0]])#.reshape(3,1) #u[0]
			derivatives.get_mutable_vector().SetFromVector(qdot)
		
		def runDircol(self,x0,xf,tf0):
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
		
		def EvalTimeDerivatives(self, context):
			#import pdb; pdb.set_trace()
			y = context.get_continuous_state_vector().Clone()
			x = context.get_continuous_state_vector().CopyToVector()
			omega = [0.0]
			v = 0.0
			theta = x[2]
			
			qdot = np.array([v*np.cos(theta),  v*np.sin(theta), omega[0]])#.reshape(3,1) #u[0]
			y.SetFromVector(qdot)
			return y
		
		#def CreateDefaultContext(self):
		#	return LeafSystem_[T]().CreateDefaultContext()
		
		
	return Impl

def isPositiveDefinite(self, M):
			assert np.size(M,0) == np.size(M, 1)

			for i in range(1, np.size(M,0)+1):
				if(np.linalg.det(M[0:i, 0:i]) <= 0):
					return False;
			return True

def FixedLyapunovSearchRho(prog, x, V, Vdot, multiplier_degree=None):
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
    
def FixedLyapunovMaximizeLevelSet(prog, x, V, Vdot, multiplier_degree=None):
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
    

def RegionOfAttraction(system, context, V=None):
    # TODO(russt): Add python binding for has_only_continuous_state
    # assert(context.has_only_continuous_state())
    # TODO(russt): Handle more general cases.

	x0 = context.get_continuous_state_vector().CopyToVector()
	
	#import pdb; pdb.set_trace()
	# Check that x0 is a fixed point.
	xdot0 = system.EvalTimeDerivatives(DubinsPlant_[None](), context).CopyToVector()
	assert np.allclose(xdot0, 0*xdot0), "context does not describe a fixed point."

	sym_system = system.ToSymbolic()
	sym_context = sym_system.CreateDefaultContext()
    
	prog = MathematicalProgram()
	x = prog.NewIndeterminates(sym_context.num_continuous_states(),'x')

	# Evaluate the dynamics (in relative coordinates)
	sym_context.SetContinuousState(x0+x)
	f = sym_system.EvalTimeDerivatives(sym_context).get_vector().CopyToVector()

	if V is None:
		# Solve a Lyapunov equation to find the Lyapunov candidate.
		A = Evaluate(Jacobian(f, x), dict(zip(x, x0)))
		Q = np.eye(sym_context.num_continuous_states())
		P = RealContinuousLyapunovEquation(A, Q)
		V = x.dot(P.dot(x))
        
	Vdot = V.Jacobian(x).dot(f)
    
	# Check Hessian of Vdot at origin
	H = Evaluate(0.5*Jacobian(Vdot.Jacobian(x),x), dict(zip(x, x0)))
	assert isPositiveDefinite(-H), "Vdot is not negative definite at the fixed point."

	#V = FixedLyapunovMaximizeLevelSet(prog, x, V, Vdot)
	V = FixedLyapunovSearchRho(prog, x, V, Vdot)
    
	# Put V back into global coordinates
	V = V.Substitute(dict(zip(x,x-x0)))
	return V


	
def runFunnel():
	print('running Funnel algorithm ...')
	vconst = 2.0
	umax = 10000.0
	
    # Declare (Dubins car) model
	plant = DubinsPlant_[None]  # Default instantiation
		
	# Trajectory optimization
	x0 = (0.0,0.0,-math.pi/4.0)  #Initial state that trajectory should start from
	xf = (1.5,0.0, math.pi/4.0)  #Final desired state
	tf0 = 0.5 # Guess for how long trajectory should take
	utraj, xtraj = plant().runDircol(x0, xf, tf0)

	# region of attraction:
	#context = plant.CreateDefaultContext(DubinsPlant_[None]())
	context = plant().CreateDefaultContext()
	context.SetContinuousState(xf)
	V = RegionOfAttraction(plant, context)
	
	print('V=')
	print(V)
	
	# Do tvlqr
	Q = np.eye(3)
	R = 1
	Qf = 1*Q
	# c, V = tvlqr(plant, xtraj, utraj, Q, R, Qf)
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
	
	
	
	
"""
class DubinsPlant(LeafSystem): 
	def __init__(self, v, umax):
		LeafSystem.__init__(self)
		self.DeclareVectorInputPort("u", BasicVector(1) ) 
		self.DeclareVectorOutputPort("state", BasicVector(3), self.CopyStateOut)
		self.DeclareContinuousState(3);
		self.v = v
		self.umax =  umax
		

	def	get_input_port(self, i):
		return LeafSystem.get_input_port(self, i)
	
	def	get_output_port(self, i):
		return LeafSystem.get_output_port(self, i)
	
	def DoCalcTimeDerivatives(self, context, output):
		x = context.get_continuous_state().get_vector().CopyToVector()

		u1 = context.get_time()
		theta = x[2];
		xdot = np.array([self.v*np.cos(theta),  self.v*np.sin(theta), u1]).reshape(3,1) #u[0]
		
		#import pdb; pdb.set_trace()
		output.get_mutable_vector().SetFromVector(xdot) #SetAtIndex(0,xdot)
	
	def CopyStateOut(self,context, bla): #output(self,t,x,u)
		x = context.get_continuous_state()
		return x
	
		
	""
	def __init__(self, v, umax, x0, xf):
		obj = LeafSystem.__init__(self, 3.0,0.0,1.0,3.0,0.0,1.0)
		obj = LeafSystem.setOutputFrame(obj, LeafSystem.getStateFrame(obj) )
		self.v = v
		self.umax =  umax
		self.x0 = x0
		self.xf = xf
		
		return obj
	
	# X = [x,y,theta]'
	def dynamics(self,t,x,u):
		theta = x[2];
		# xdot = np.array([self.v*cos(theta),  self.v*sin(theta), u[0]]).reshape(3,1)
		xdot = np.column_stack([self.v*cos(theta),  self.v*sin(theta), u[0]])
		        
		df,d2f,d3f = dynamicsGradients(self,t,x,u,3)
		return xdot, df, d2f, d3f
	
	def dynamicsGradients(self, a1, a2, a3, a4, order):
		# Symbol table:
		a3_3=a3[2]
		a4_1=a4[0]
		v=a1.v
		# Compute Gradients:
		df = np.zeros([3,5])
		df[0,3] = -v*np.sin(a3_3)
		df[1,3] = v*np.cos(a3_3)
		df[2,4] = 1

		# d2f
		if (order>=2):
			d2f = np.zeros([3,25])
			d2f[0,18] = -v*np.cos(a3_3)
			d2f[1,18] = -v*np.sin(a3_3)
		else:
			d2f=np.array([])
		
		# d3f
		if (order>=3):
			d3f = np.zeros([3,125])
			d3f[0,93] = v*np.sin(a3_3)
			d3f[1,93] = -v*np.cos(a3_3)
		else:
			d3f=np.array([])
			
		return df, d2f, d3f
	""
	
	def getInitialState(self):
		return np.column_stack([0,0,0]) # np.zeros(3).reshape(3,1)

	def runDircol(self,x0,xf,tf0):
		N = 21
		
		context = self.CreateDefaultContext()
		dircol  = DirectCollocation(self, context, num_time_samples=N,
                           minimum_timestep=0.1, maximum_timestep=1.0)
		u = dircol.input()
		dircol.AddEqualTimeIntervalsConstraints()
		dircol.AddConstraintToAllKnotPoints(u[0] <=  .5*self.umax)
		dircol.AddConstraintToAllKnotPoints(u[0] >= -.5*self.umax)
		initial_state = (0., 0., 0.)
		dircol.AddBoundingBoxConstraint(initial_state, initial_state,
                                		dircol.initial_state())
		final_state = (1.5, 0., 0.)
		dircol.AddBoundingBoxConstraint(final_state, final_state,
										dircol.final_state())
		
		R = 10  # Cost on input "effort".
		dircol.AddRunningCost(R*u[0]**2)

		# Add a final cost equal to the total duration.
		dircol.AddFinalCost(dircol.time())

		initial_x_trajectory = \
			PiecewisePolynomial.FirstOrderHold([0., tf0],
											   np.column_stack((initial_state,
																final_state)))
		dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

		result = Solve(dircol)
		xtraj,utraj = dircol.ReconstructStateTrajectory(result)


		#traj_init.x = PPTrajectory(FirstOrderHold([0,tf0],[x0,xf]))
		#info = 0
		#while (info is not 1):
		#	xtraj,utraj,z,F,info = prog.solveTraj(tf0)
			
		return utraj,xtraj
	"""	