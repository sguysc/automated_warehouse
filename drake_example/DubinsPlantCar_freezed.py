import numpy as np
import matplotlib.pyplot as plt
import math

from pydrake.all import (DirectCollocation, PiecewisePolynomial, BasicVector_, LeafSystem_, Solve, FirstOrderTaylorApproximation, LinearQuadraticRegulator, Linearize, Variables, MathematicalProgram, Evaluate, Jacobian, RealContinuousLyapunovEquation, Substitute)
from pydrake.all import (plot_sublevelset_quadratic)
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake import symbolic
from pydrake.symbolic import Polynomial
from pydrake.solvers.scs import ScsSolver
from pydrake.solvers.ipopt import IpoptSolver
from pydrake.solvers.mosek import MosekSolver
#from pydrake.solvers.csdp import CsdpSolver

# to solve the differential ricatti eqn
from scipy.integrate import solve_ivp

@TemplateSystem.define("DubinsPlant_")
def DubinsPlant_(T):
	class Impl(LeafSystem_[T]):
		def _construct(self, converter=None):
			# car model:
			# ==========
			# X = [x y theta]';    U = [delta v]'   [steer ang, fwd vel]'
			# xdot = v*cos(teta)
			# ydot = v*sin(teta)
			# tetadot = v/L * tan( delta )
			LeafSystem_[T].__init__(self, converter)
			self.nX = 3
			self.nU = 2
			# two inputs (delta steering angle, u forward velocity)
			self.DeclareVectorInputPort("u", BasicVector_[T](self.nU))
			# three outputs (full state)
			self.DeclareVectorOutputPort("x", BasicVector_[T](self.nX),
										 self.CopyStateOut)
			# three positions, no velocities
			self.DeclareContinuousState(self.nX, 0, 0)

			# parameters and limits
			self.L = 1.0 #length of car
			self.umax =  5.0  #m/sec
			self.delmax =  30.0*math.pi/180.0  #rad

		def _construct_copy(self, other, converter=None):
			Impl._construct(self, converter=converter)

		def CopyStateOut(self, context, output):
			x = context.get_continuous_state_vector().CopyToVector()
			output.SetFromVector(x) # = y 

		# X = [x y theta]';    U = [delta v]'   [steer ang, fwd vel]'
		def DoCalcTimeDerivatives(self, context, derivatives):
			x = context.get_continuous_state_vector().CopyToVector()
			u = self.EvalVectorInput(context, 0).CopyToVector()
			theta = x[2]
			
			qdot = np.array([ u[1]*np.cos(theta), \
							  u[1]*np.sin(theta), \
							  u[1]*np.tan(u[0])/self.L ])
			derivatives.get_mutable_vector().SetFromVector(qdot)
		
		def runDircol(self,x0,xf,tf0):
			N = 21 # constant
			#N = np.int(tf0 * 10) # "10Hz" / samples per second

			context = self.CreateDefaultContext()
			dircol  = DirectCollocation(self, context, num_time_samples=N,
							   minimum_timestep=0.05, maximum_timestep=1.0)
			u = dircol.input()
			# set some constraints on inputs
			dircol.AddEqualTimeIntervalsConstraints()
			
			dircol.AddConstraintToAllKnotPoints(u[0] <=  self.delmax)
			dircol.AddConstraintToAllKnotPoints(u[0] >= -self.delmax)
			dircol.AddConstraintToAllKnotPoints(u[1] <=  self.umax)
			dircol.AddConstraintToAllKnotPoints(u[1] >= -self.umax)
			
			# set some constraints on start and final pose
			eps = 0.0 # relaxing factor
			dircol.AddBoundingBoxConstraint(x0, x0, dircol.initial_state())
			dircol.AddBoundingBoxConstraint(xf-np.array([eps, eps, eps]), xf+np.array([eps, eps, eps]), dircol.final_state())

			R = 1.0*np.eye(self.nU)  # Cost on input "effort".
			dircol.AddRunningCost( u.transpose().dot(R.dot(u)) ) 

			# Add a final cost equal to the total duration.
			dircol.AddFinalCost(dircol.time())

			# guess initial trajectory
			initial_x_trajectory = \
				PiecewisePolynomial.FirstOrderHold([0., tf0], np.column_stack((x0, xf)))
			dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

			# optimize
			result = Solve(dircol)
			print('******\nRunning trajectory optimization:')
			print('w/ solver %s' %(result.get_solver_id().name()))
			print(result.get_solution_result())
			assert(result.is_success())

			xtraj = dircol.ReconstructStateTrajectory(result)
			utraj = dircol.ReconstructInputTrajectory(result)

			# return nominal trajectory
			return utraj,xtraj
		
		# Continuous Differential Riccati Equation (solved reverse in time)
		def Cdre(self, t, S, Q, Rinv, xtraj, utraj):
			x0 = xtraj.value(t).transpose()[0]
			u0 = utraj.value(t).transpose()[0]
			x0d = xtraj.derivative(1).value(t).transpose()[0]
			
			A, B = self.PlantDerivatives(x0, u0)
			s1 = S[0:(self.nX*self.nX)].reshape(self.nX, self.nX) # comes in as vector, switch to matrix
			#s2 = S[(self.nX*self.nX):12].reshape(self.nX, 1)
			#s3 = S[-1].reshape(1)
			#c = np.vstack((0.,0.,0.))  #xd-x0d
			#rs = B.transpose().dot(s2)/2.0
			S1dot = -(A.transpose().dot(s1) + s1.dot(A) - s1.dot(B).dot(Rinv).dot(B.transpose()).dot(s1) + Q)
  			S2dot = np.array([]) #-(-2.0*(s1.dot(B)).dot(Rinv).dot(rs) + A.transpose().dot(s2) + 2.0*s1.dot(c))
  			S3dot = np.array([[]]) #-(-rs.transpose().dot(Rinv).dot(rs) + s2.transpose().dot(c))
			Sdot = np.concatenate((S1dot.ravel(),S2dot.ravel(),S3dot[0]))
			return Sdot # return as vector
		
		def PlantDerivatives(self, x, u):
			# this was done manually for now (speed in mind, but never actually checked that symbolically is slow),
			# but basically could also be automatically derived using symbolic system
			A = np.array([[0., 0., -u[1]*np.sin(x[2])], \
         	     		  [0., 0.,  u[1]*np.cos(x[2])], \
         		 		  [0., 0., 0.]])
			B = np.array([[0., np.cos(x[2])], \
         		 		  [0., np.sin(x[2])], \
         		 		  [u[1]*(1+np.tan(u[0])**2)/self.L, np.tan(u[0])/self.L ]])
			return A, B
		#
		def TVLQR(self, xtraj, utraj, Q, R, Qf):
			context = self.CreateDefaultContext()
			sym_system = self.ToSymbolic()
			sym_context = sym_system.CreateDefaultContext()
			
			prog = MathematicalProgram()
			x = prog.NewIndeterminates(self.nX,'x')
			ucon = prog.NewIndeterminates(self.nU,'u')
			
			times = xtraj.get_segment_times()
			t = times[-1]

			# final state goal
			x0 = xtraj.value(t).transpose()[0]
			u0 = utraj.value(t).transpose()[0]

			sym_context.SetContinuousState(x) #x0+
			sym_context.FixInputPort(0, ucon ) #u0+
			
			# xdot=f(x,u)==>xhdot=f(xh+x0,uh+u0)-x0dot ~= df/dx*xh + df/du*uh
			f = sym_system.EvalTimeDerivatives(sym_context).CopyToVector() 
			mapping = dict(zip(x, x0))
			mapping.update(dict(zip(ucon, u0)))
			# get the final condition, according to Tedrake10' Qf should be last S(LTI)
			#A_sim = Jacobian(f, x)
			#B_sim = Jacobian(f, ucon)
			#A = Evaluate(A_sim, mapping)
			#B = Evaluate(B_sim, mapping)
			#k,s = LinearQuadraticRegulator(A, B, Q, R)  # get the Qf = S_lti
			#Qf = s
			# now, run backwards in time to solve for S for associated Riccati equation
			Rinv = np.linalg.inv(R)
			#S(3x3),s1(3,1),s2(1x1)
			#S0 = np.array(Qf.ravel().tolist()+[0.0,0.0,0.0,0.0])
			S0 = Qf.ravel()
			sol = solve_ivp(lambda t,S: self.Cdre(t,S,Q,Rinv,xtraj,utraj), \
							[times[-1], times[0]], S0, t_eval=times[::-1])
			K = []
			S = []
			Ssol = np.fliplr(sol.y)  # flip is because we switch back to normal time progress
			for i in range(sol.y.shape[1]):
				x0 = xtraj.value(times[i]).transpose()[0]
				u0 = utraj.value(times[i]).transpose()[0]
				A, B = self.PlantDerivatives(x0, u0)
				#import pdb; pdb.set_trace()
				#s = Ssol[0:self.nX*self.nX,i].reshape(Qf.shape)
				#s2 = Ssol[self.nX*self.nX:12,i].reshape(self.nx,1)
				#s3 = Ssol[-1,i].reshape(1)
				s = Ssol[:,i].reshape(Qf.shape)
				k = Rinv.dot(B.transpose()).dot(s)
				if(len(K) == 0):
					K = np.ravel(k).reshape(self.nX*self.nU,1) 
					S = np.ravel(s).reshape(self.nX*self.nX,1) 
				else:
					K = np.hstack( (K, np.ravel(k).reshape(self.nX*self.nU,1)) )
					S = np.hstack( (S, np.ravel(s).reshape(self.nX*self.nX,1)) )
			
			Kpp = PiecewisePolynomial.FirstOrderHold(times, K)
			Spp = PiecewisePolynomial.FirstOrderHold(times, S)
			#import pdb; pdb.set_trace()
			return Kpp,Spp
		
		# get system dynamics symbolically
		def EvalTimeDerivatives(self, context):
			y = context.get_continuous_state_vector().Clone()
			x = context.get_continuous_state_vector().CopyToVector()
			
			u = self.GetInputPort('u').EvalBasicVector(context).CopyToVector()
			
			theta = x[2]
			
			qdot = np.array([u[1]*np.cos(theta), \
							 u[1]*np.sin(theta), \
							 u[1]*np.tan(u[0])/self.L]) #
			
			y.SetFromVector(qdot)
			return y
		
		# get the polynomial representation of the system using Taylor expansion on each non-polynomial expression
		# this was done manually and on the trigonometric expressions only
		def EvalTimeDerivativesTaylor(self, context):
			y = context.get_continuous_state_vector().Clone()
			x = context.get_continuous_state_vector().CopyToVector()
			
			u = self.GetInputPort('u').EvalBasicVector(context).CopyToVector()
			theta = x[2]
			
			qdot = np.array([u[1]*self.taylor_cos(theta), \
							 u[1]*self.taylor_sin(theta), \
							 u[1]*self.taylor_tan(u[0])/self.L]) #

			y.SetFromVector(qdot)
			return y
		
		# expansion of the trigonometric functions to whatever order we want
		def taylor_cos(self, x):
			return (1.0 - (x**2)/2.0 + (x**4)/24.0)
		
		def taylor_sin(self, x):
			return (x - (x**3)/6.0)  # + (x**5)/120.0)
		
		def taylor_tan(self, x):
			return (x + (x**3)/3.0) # + (2*x**5)/15.0  )
		
		# evaluate the closed loop dynamics with the LQR feedback controller
		#  xhdot = f(x0+xh, u0-Kxh)  where xh=x-x0
		def EvalClosedLoopDynamics(self, x, ucon, f, x_d, u_d, K):
			#import pdb; pdb.set_trace()
			U = u_d - K.dot(x-x_d)  # feedback law
			delta_fb = U[0]
			u_fb = U[1]
			mapping = dict(zip(ucon, [delta_fb, u_fb])) #now replace u with u0-K*xh
			f_fb = Substitute(f, mapping)  # now xdot=f(x), autonomous system
			f_fb = np.array([ f_fb[0][0], f_fb[1][0], f_fb[2][0]]) # gets rid of the extra array
			
			return f_fb

		"""
		# trying to numerically find a Lyapunov function using an SOS approach rather than using the S from the LQR alg.
		# this was done in the beginning when I had my own controller (feedback linearization), but this was never able to
		# find a solution
		def findLyapunovFunctionSOS(self, xtraj, utraj, deg_V, deg_L):
			times = xtraj.get_segment_times()
			
			prog = MathematicalProgram()
			
			# Declare the "indeterminates", x. These are the variables which define the polynomials
			x = prog.NewIndeterminates(self.nX, 'x')
			ucon = prog.NewIndeterminates(self.nU, 'u')
			sym_system = self.ToSymbolic()
			sym_context = sym_system.CreateDefaultContext()
			sym_context.SetContinuousState(x)
			sym_context.FixInputPort(0, ucon )
			f = sym_system.EvalTimeDerivativesTaylor(sym_context).CopyToVector() # - dztrajdt.value(t).transpose()
			
			for t in times:
				x0 = xtraj.value(t).transpose()[0]
				u0 = utraj.value(t).transpose()[0]
				
				f_fb = self.EvalClosedLoopDynamics(x, ucon, f, x0, u0)
				
				# Construct a polynomial V that contains all monomials with s,c,thetadot up to degree n.
				V = prog.NewFreePolynomial(Variables(x), deg_V).ToExpression()
				eps = 1e-4
				constraint1 = prog.AddSosConstraint(V - eps*(x-x0).dot(x-x0)) # constraint to enforce that V is strictly positive away from x0.
				Vdot = V.Jacobian(x).dot(f_fb) # Construct the polynomial which is the time derivative of V
				#L = prog.NewFreePolynomial(Variables(x), deg_L).ToExpression() # Construct a polynomial L representing the 
																			   # "Lagrange multiplier".
				# Add a constraint that Vdot is strictly negative away from x0
				constraint2 = prog.AddSosConstraint(-Vdot[0] - eps*(x-x0).dot(x-x0)) 
				#import pdb; pdb.set_trace()
				# Add V(0) = 0 constraint
				constraint3 = prog.AddLinearConstraint(V.Substitute({x[0]: 0.0, x[1]: 1.0, x[2]: 0.0}) == 1.0)
				# Add V(theta=xxx) = 1, just to set the scale.
				constraint4 = prog.AddLinearConstraint(V.Substitute({x[0]: 1.0, x[1]: 0.0, x[2]: 0.0}) == 1.0)
				# Call the solver (default).
				#result = Solve(prog)
				# Call the solver (specific).
				#solver = CsdpSolver() 
				solver = ScsSolver()
				#solver = IpoptSolver()
				result = solver.Solve(prog, None, None)
				# print out the result.
				print("Success? ", result.is_success())
				print(result.get_solver_id().name())
				
				Vsol = Polynomial(result.GetSolution(V))
				
				print('Time t=%f:\nV=' %(t) )
				print(Vsol.RemoveTermsWithSmallCoefficients(1e-6))
				
			return Vsol, Vsol.Jacobian(x),dot(f_fb)
		"""
		
		# just a helper function
		def isPositiveDefinite(self, M):
			assert np.size(M,0) == np.size(M, 1)

			for i in range(1, np.size(M,0)+1):
				if(np.linalg.det(M[0:i, 0:i]) <= 0):
					return False;
			return True
		
		#
		def minimumV(self, prev_x, Vs):
			N = len(prev_x)
			Vmin = np.zeros(len(Vs))
			
			for i in range(len(Vs)):
				''' option 1: solve SOS programs (it's not so bad)  
					to find minimval value of V, we get a lower bound with slack variable
						maximize slack
							s.t. V-slack is SOS
				'''
				prog = MathematicalProgram()
				x = prog.NewIndeterminates(N,'x')
				V = Vs[i].Substitute(dict(zip(prev_x, x)))
				slack = prog.NewContinuousVariables(1,'g')[0] 
				prog.AddSosConstraint( -slack + V )  #slack*x.transpose().dot(x)
				#import pdb; pdb.set_trace()
				prog.AddConstraint( slack >= 1.0E-12 )
				prog.AddCost(-slack)
				result = Solve(prog)
				assert result.is_success()
				Vmin[i] = result.GetSolution(slack)
				""" option 2: do algerbraic manipulations to find
					minimum value of this V = x'Sx   (not fully implemented yet)
				H = np.zeros((N,N))
				for j in range(N):
					for k in range(N):
						H[j][k] = V[i].Differentiate(x[j]).Differentiate(x[k])
				
				H = 0.5*diff(diff(V,x).transpose(),x)
    			b = -0.5*(H / (subs(diff(V,x),x,0*x).transpose()))
				Vmin[i] = subs(V,x,b)
				"""
   
			return Vmin

		# Attempts to find the largest funnel, defined by the time-varying
		# one-level set of V, which verifies (using SOS over state at finite 
		# sample points in time) initial conditions to end inside the one-level set
		# of the goal region G at time ts(end).  
		# from the paper "Invariant Funnels around Trajectories usingSum-of-Squares Programming", Tobenkin et al.
		def TimeVaryingLyapunovSearchRho(self, prog, prev_x, Vs, Vdots, times, rho_f, multiplier_degree=None):
			N = len(times)-1
			Vmin = self.minimumV(prev_x, Vs) #0.05*np.ones((1, len(times))) # need to do something here instead of this!! 
			print(Vmin)
			dt = np.diff(times)
			rho = np.flipud(rho_f*np.exp(-2.0*(np.array(times)-times[0])/(times[-1]-times[0])))+ np.max(Vmin) 
			#rho = np.linspace(np.max(Vmin), rho_f, num=len(times)) # just heuristics for now
			rhodot = np.diff(rho)/dt
			# sampleCheck()
			Lambda_vec = []
			#slack_vec = []
			#import pdb; pdb.set_trace()
			#fix rho, optimize Lagrange multipliers
			for i in range(N):
				prog = MathematicalProgram()
				x = prog.NewIndeterminates(len(prev_x),'x')
				V = Vs[i].Substitute(dict(zip(prev_x, x)))
				Vdot = Vdots[i].Substitute(dict(zip(prev_x, x)))
				Lambda = prog.NewSosPolynomial(Variables(x), multiplier_degree)[0].ToExpression()
				gamma = prog.NewContinuousVariables(1,'g')[0] 
				# Jdot-rhodot+Lambda*(rho-J) < -gamma
				prog.AddSosConstraint( -gamma*x.transpose().dot(x) - (Vdot-rhodot[i] + Lambda*(rho[i]-V)) ) #gamma*x.transpose().dot(x)
				prog.AddConstraint( gamma >= 1.0E-6 )
				prog.AddCost(-gamma) #maximize gamma
				result = Solve(prog)
				assert result.is_success()
				
				Lambda_vec.append(result.GetSolution(Lambda))
				slack = result.GetSolution(gamma)
				if(slack[-1]>0.0001):
					print('Something is not great with searching for Lambda[%d] :(' %(i))
					assert (slack < 0.0001)
			
			# fix Lagrange multipliers, maximize rho
			rhointegral = 0.0
			prog = MathematicalProgram()
			x = prog.NewIndeterminates(len(prev_x),'x')
			t = prog.NewContinuousVariables(N,'t')
			rho = np.array([t, rho_f]) + Vmin
			for i in range(N):
				prog.AddConstraint(t[i]>=0.0)  # does this mean [prog,rho] = new(prog,N,'pos'); in matlab??
				rhodot = (rho[i+1]-rho[i])/dt[i]
				rhointegral = rhointegral+rho[i]*dt[i] + 0.5*rhod_vec[i]*(dt[i]**2)
				
				V    = Vs[i].Substitute(dict(zip(prev_x, x)))
				Vdot = Vdots[i].Substitute(dict(zip(prev_x, x)))
				L    = Lambda_vec[i].Substitute(dict(zip(prev_x, x)))
				prog.AddSosConstraint( -(Vdot - rhodot + L * ( rho[i]-V ) ) )
				
			prog.AddCost(-rhointegral)
			result = Solve(prog)
			assert result.is_success()
			rho = result.GetSolution(rho)
			rhointegral = result.GetSolution(rhointegral)
			rhodot = np.diff(rho)/dt
			# end of iterations if we want
			V = 1.0/rho * Vs
			return V
			
		"""	
		def FixedLyapunovSearchRho(self, prog, x, V, Vdot, multiplier_degree=None):
			#'''
			Assumes V>0.
			V <= rho => Vdot <= 0 via
				find Lambda subject to
				  -Vdot + Lambda*(V - rho) is SOS, 
				  Lambda is SOS.
			#'''

			def CheckLevelSet(prev_x, prev_V, prev_Vdot, rho, multiplier_degree):
				prog = MathematicalProgram()
				x = prog.NewIndeterminates(len(prev_x),'x')
				V = prev_V.Substitute(dict(zip(prev_x, x)))
				Vdot = prev_Vdot.Substitute(dict(zip(prev_x, x)))
				slack = prog.NewContinuousVariables(1,'b')[0]  
				
				#import pdb; pdb.set_trace()
				Lambda = prog.NewSosPolynomial(Variables(x), multiplier_degree)[0].ToExpression()
				print('Vdot degree: %d' %(Polynomial(Vdot).TotalDegree()))
				print('SOS degree: %d' %(Polynomial(-Vdot + Lambda*(V - rho) - slack*V).TotalDegree()))
				
				prog.AddSosConstraint(-Vdot + Lambda*(V - rho) - slack*V)
				prog.AddCost(-slack)
				#e1=prog.AddConstraint(slack, 0.0, 10.0)
				#cstmsol = MosekSolver()
				#result = cstmsol.Solve(prog, None, None)
				#cstmsol = ScsSolver()
				#result = cstmsol.Solve(prog, None, None)
				result = Solve(prog)
				#import pdb; pdb.set_trace()
				print(result.get_solution_result() )
				print('****** Running solver w/ %s' %(result.get_solver_id().name()))
				assert result.is_success()
				return result.GetSolution(slack)

			#import pdb; pdb.set_trace()
			if multiplier_degree is None:
				# There are no guarantees... this is a reasonable guess:
				multiplier_degree = Polynomial(Vdot).TotalDegree()
				print ("Using a degree " + str(multiplier_degree) + " multiplier for the S-procedure")

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
				print ("Using a degree " + str(multiplier_degree) + " multiplier for the S-procedure")

			# TODO(russt): implement numerical "balancing" from matlab version.
			Lambda = prog.NewSosPolynomial(Variables(x), multiplier_degree)[0].ToExpression()

			rho = prog.NewContinuousVariables(1, "rho")[0]
			prog.AddSosConstraint((V-rho)*(x.dot(x)) - Lambda*Vdot)

			prog.AddCost(-rho)

		#    mosek = MosekSolver()
		#    mosek.set_stream_logging(True, 'mosek.out')
		#    result = mosek.Solve(prog, None, None)
			result = Solve(prog)

			print ("Using " + result.get_solver_id().name())

			assert result.is_success()
			assert result.GetSolution(rho) > 0, "Optimization failed (rho <= 0)."

			return V/result.GetSolution(rho)
		"""
		
		def RegionOfAttraction(self, context, xtraj, utraj, V=None):
			# Construct a polynomial V that contains all monomials with s,c,thetadot up to degree 2.
			#deg_V = 2
			# Construct a polynomial L representing the "Lagrange multiplier".
			deg_L = 4
			Q  = 3.0*np.eye(self.nX)
			R  = 1.0*np.eye(self.nU)
			Qf = 3.0*np.eye(self.nX)
			# set some constraints on inputs
			#context.SetContinuousState(xtraj.value(xtraj.end_time()))
			#context.FixInputPort(0, utraj.value(utraj.end_time()))

			#x0 = context.get_continuous_state_vector().CopyToVector()
			#if(xdotraj is None):
			#	xdotraj = xtraj.Derivative(1)
			
			# Check that x0 is a "fixed point" (on the trajectory).
			#xdot0 = self.EvalTimeDerivatives(context).CopyToVector()
			#assert np.allclose(xdot0, xdotraj), "context does not describe valid path."   #0*xdot0), 
			
			prog = MathematicalProgram()
			x = prog.NewIndeterminates(self.nX, 'x')
			ucon = prog.NewIndeterminates(self.nU, 'u')
			sym_system = self.ToSymbolic()
			sym_context = sym_system.CreateDefaultContext()
			sym_context.SetContinuousState(x)
			sym_context.FixInputPort(0, ucon )
			f = sym_system.EvalTimeDerivativesTaylor(sym_context).CopyToVector() 
			#f = sym_system.EvalTimeDerivatives(sym_context).CopyToVector() 
			
			times = xtraj.get_segment_times()
			all_V = []
			all_Vd=[]
			all_fcl=[]
			fig, ax = plt.subplots()
			#ax.plot(x[0,:], x[1,:], color='k', linewidth=2)
			#ax.set_xlim([-2.5, 2.5])
			#ax.set_ylim([-3, 3])
			ax.set_xlabel('x')
			ax.set_ylabel('y')
			plt.draw()
			
			if V is None:
				# Do optimization to find the Lyapunov candidate.
				#print('******\nRunning SOS to find Lyapunov function ...') 
				#V, Vdot = self.findLyapunovFunctionSOS(xtraj, utraj, deg_V, deg_L)
				# or Do tvlqr to get S
				print('******\nRunning TVLQR ...')
				K, S  = self.TVLQR(xtraj, utraj, Q, R, Qf) 
				#S0 = S.value(times[-1]).reshape(self.nX, self.nX)
				#print(Polynomial(x.dot(S0.dot(x))).RemoveTermsWithSmallCoefficients(1e-6))
				#import pdb; pdb.set_trace()
				print('Done\n******')
				for t in times:
					x0 = xtraj.value(t).transpose()[0]
					u0 = utraj.value(t).transpose()[0]
					K0 = K.value(t).reshape(self.nU, self.nX)
					S0 = S.value(t).reshape(self.nX, self.nX)
					S0d = S.derivative(1).value(t).reshape(self.nX, self.nX) # Sdot
					# make sure Lyapunov candidate is pd
					if (self.isPositiveDefinite(S0)==False):
						assert False, '******\nS is not PD for t=%f\n******' %(t)
					# for debugging
					#S0 = np.eye(self.nX)
					V = x.dot(S0.dot(x))
					# get a polynomial representation of f_closedloop, xdot = f_cl(x)
					f_cl = self.EvalClosedLoopDynamics(x, ucon, f, x0, u0, K0)
					# vdot = x'*Sdot*x + 2*x'*S*fcl_poly
					#Vdot = (x.transpose().dot(S0d)).dot(x) + 2*(x.transpose().dot(S0)).dot(f_cl) 
					Vdot = (x.transpose().dot(S0d)).dot(x) + V.Jacobian(x).dot(f_cl) 
					# store it for later use
					all_V.append(V)
					all_fcl.append(f_cl)
					all_Vd.append(Vdot)
			
			#import pdb; pdb.set_trace()
			#V_pp = PiecewisePolynomial.FirstOrderHold(times, all_V)
			#Vdot_pp = V_pp.derivative(1)
			#for t in times:
			#	Vdot = V_pp.value(t).Jacobian(x).transpose().dot(all_fcl.value(t)) + Vdot_pp.value(t)
			#	all_Vd.append(Vdot)
			
			rho_f = 0.5
			# time-varying PolynomialLyapunovFunction who's one-level set defines the verified invariant region
			V = self.TimeVaryingLyapunovSearchRho(prog, x, all_V, all_Vd, times, rho_f, multiplier_degree=4)
			# Check Hessian of Vdot at origin
			#H = Evaluate(0.5*Jacobian(Vdot.Jacobian(x),x), dict(zip(x, x0)))
			#assert isPositiveDefinite(-H), "Vdot is not negative definite at the fixed point."
			return V
		
		def my_plot_sublevelset_expression(self, ax, e, x0, vertices=51, **kwargs):
			p = Polynomial(e)
			assert p.TotalDegree() == 2

			x = list(e.GetVariables())
			env = {a: 0 for a in x}
			c = e.Evaluate(env)
			e1 = e.Jacobian(x)
			b = Evaluate(e1, env)
			e2 = Jacobian(e1, x)
			A = 0.5*Evaluate(e2, env)
			# simple projection to 2D assuming we want to plot on (x1,x2)
			A = A[0:2,0:2]
			b = b[0:2]*0.0
			c = 0.0
			#Plots the 2D ellipse representing x'Ax + b'x + c <= 1, e.g.
    		#the one sub-level set of a quadratic form.
			H = .5*(A+A.T)
			xmin = np.linalg.solve(-2*H, np.reshape(b, (2, 1)))
			fmin = -xmin.T.dot(H).dot(xmin) + c  # since b = -2*H*xmin
			assert fmin <= 1, "The minimum value is > 1; there is no sub-level set " \
                      "to plot"

			# To plot the contour at f = (x-xmin)'H(x-xmin) + fmin = 1,
			# we make a circle of values y, such that: y'y = 1-fmin,
			th = np.linspace(0, 2*np.pi, vertices)
			Y = np.sqrt(1-fmin)*np.vstack([np.sin(th), np.cos(th)])
			# then choose L'*(x - xmin) = y, where H = LL'.
			L = np.linalg.cholesky(H)
			X = np.tile(xmin, vertices) + np.linalg.inv(np.transpose(L)).dot(Y)

			return ax.fill(X[0, :]+x0[0], X[1, :]+x0[1], **kwargs)
			#return plot_sublevelset_quadratic(ax, A, b, c, vertices, **kwargs)

	return Impl


# main function, finding the maximal funnel
def runFunnel():
	print('******\nrunning Funnel algorithm ...\n******')
	
    # Declare car model
	plant = DubinsPlant_[float]() #None]  # Default instantiation
		
	# Trajectory optimization to get nominal trajectory
	x0 = (0.0, 0.0, -math.pi/4.0)  #Initial state that trajectory should start from
	xf = (1.5, 0.0, math.pi/4.0)  #Final desired state
	tf0 = 1.0 # Guess for how long trajectory should take
	utraj, xtraj = plant.runDircol(x0, xf, tf0)
	print('Trajectory takes %f[sec]' %(utraj.end_time()))
	print('Done\n******')
	#import pdb; pdb.set_trace()
	# region of attraction:
	context = plant.CreateDefaultContext()
	V = plant.RegionOfAttraction(context, xtraj, utraj)
	#print('V=')
	#print(V)
		
	# resolve the trajectory piecewise polynomial structure
	N = 100 # number of points to sample
	times = np.linspace(utraj.start_time(), utraj.end_time(), N)
	#u_lookup = np.vectorize(utraj.value)
	#u_values = u_lookup(times)
	u_values = np.hstack([utraj.value(t) for t in times])
	xy_knots = np.hstack([xtraj.value(t) for t in times])

	# plotting stuff
	fig, ax = plt.subplots(3)
	fig.suptitle('Direct collocation trajectory optimization (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)' %\
				 (x0[0],x0[1],x0[2],xf[0],xf[1],xf[2]))
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
	
	
