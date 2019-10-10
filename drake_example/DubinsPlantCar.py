import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.special import factorial
import itertools

from pydrake.all import (DirectCollocation, PiecewisePolynomial, BasicVector_, LeafSystem_, Solve, FirstOrderTaylorApproximation, LinearQuadraticRegulator, Linearize, Variables, MathematicalProgram, Evaluate, Jacobian, RealContinuousLyapunovEquation, Substitute)
from pydrake.all import (Simulator, DiagramBuilder, VectorSystem)
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

@TemplateSystem.define("DubinsCarPlant_")
def DubinsCarPlant_(T):
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

			# parameters and limits from 
			# "Tow tractor and Center rider Vehicle Limits Rev1.docx"
			self.L = 1.0 #length of car, for a 36inch length fork
			self.umax =  2.6 * 1.6 * 1000.0 / 3600.0  # mph -> m/sec     5.0
			self.delmax =  80.0*math.pi/180.0  #rad   30.0
			# acceleration
			#Vehicle acceleration, unloaded 0.8 m/s2
			#Vehicle deceleration, unloaded 1.6 m/s2
			#Steer slew rate 60 deg / sec 

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
			
			#distance = np.linalg.norm(np.array(xf)-np.array(x0))
			#est_time_to_travel = distance/self.umax
			#N = np.int(est_time_to_travel * self.umax + 1.) # "1Hz" / samples per second
			#import pdb; pdb.set_trace() #(xf-x0)
			#N = 5#8 # constant
			#N = np.int(tf0 * 10) # "10Hz" / samples per second
			N = 31 #12
			#print('N=%d' %(N) )
			context = self.CreateDefaultContext()
			dircol  = DirectCollocation(self, context, num_time_samples=N,
							   minimum_timestep=0.05, maximum_timestep=2.0)
			u = dircol.input()
			# set some constraints on inputs
			#import pdb; pdb.set_trace()
			dircol.AddEqualTimeIntervalsConstraints()
			
			dircol.AddConstraintToAllKnotPoints(u[0] <=  self.delmax)
			dircol.AddConstraintToAllKnotPoints(u[0] >= -self.delmax)
			dircol.AddConstraintToAllKnotPoints(u[1] <=  self.umax)
			dircol.AddConstraintToAllKnotPoints(u[1] >= -0.*self.umax)
			
			# constrain the last input to be zero (at least for the u input)
			#dv = dircol.decision_variables()
			#final_u_decision_var = dv[self.nX*N + self.nU*N - 1] #u[t_end]
			#dircol.AddLinearEqualityConstraint(final_u_decision_var, 0.0)
			#first_u_decision_var = dv[self.nX*N + 1 ] #u[t_0]
			#dircol.AddLinearEqualityConstraint(first_u_decision_var, 0.0)
			
			# set some constraints on start and final pose
			eps = .1 * np.ones(self.nX) # relaxing factor  np.array([0.0, 0.0, 0.0])
			#eps[-1] = 0.0
			#eps = np.array([0.0, 0.0, 0.0])
			dircol.AddBoundingBoxConstraint(x0, x0, dircol.initial_state())
			dircol.AddBoundingBoxConstraint(xf-eps, \
											xf+eps, dircol.final_state())

			#R = 1.0*np.eye(self.nU)  # Cost on input "effort".
			R = np.diag((20., 1.))  # More cost on input of steering angle "effort" to make it curvey.
			dircol.AddRunningCost( u.transpose().dot(R.dot(u)) ) 

			# Add a final cost equal to the total duration.
			dircol.AddFinalCost(dircol.time()/10.0) #10.0

			# guess initial trajectory
			initial_x_trajectory = \
				PiecewisePolynomial.FirstOrderHold([0., tf0], np.column_stack((x0, xf)))
			dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_x_trajectory)

			# optimize
			result = Solve(dircol)
			print('******\nRunning trajectory optimization:')
			print('w/ solver %s' %(result.get_solver_id().name()))
			print(result.get_solution_result())
			if(result.is_success() == False):
				import pdb; pdb.set_trace()
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
		
		# Continuous Differential Riccati Equation for the closed loop (solved reverse in time)
		def Cdre_CL(self, t, S, Q, Rinv, xtraj, utraj, x, ucon, K):
			x0 = xtraj.value(t).transpose()[0]
			u0 = utraj.value(t).transpose()[0]
			x0d = xtraj.derivative(1).value(t).transpose()[0]
			Kf = K.value(t).reshape(self.nU, self.nX)
						
			zero_map = dict(zip(x,np.zeros(self.nX)))
			f_Lcl    = self.EvalClosedLoopDynamics(x, ucon, x0, u0, x0d, Kf, order=1) # linearization CL
			Af       = Evaluate(Jacobian(f_Lcl, x), zero_map) # because this is rel. coord.
					
			s1 = S.reshape(self.nX, self.nX) # comes in as vector, switch to matrix
			S1dot = -(Af.transpose().dot(s1) + s1.dot(Af) + Q)

			return S1dot.ravel() # return as vector
		
		def PlantDerivatives(self, x, u):
			# this was done manually for now (speed in mind, but never actually checked that symbolically is slow),
			# but basically could also be automatically derived using symbolic system
			A = np.array([[0., 0., -u[1]*np.sin(x[2])], \
         	     		  [0., 0.,  u[1]*np.cos(x[2])], \
         		 		  [0., 0.,  0.]])
			B = np.array([[0., np.cos(x[2])], \
         		 		  [0., np.sin(x[2])], \
         		 		  [u[1]*(1+np.tan(u[0])**2)/self.L, np.tan(u[0])/self.L ]])
			return A, B
		#
		def TVLQR(self, xtraj, utraj, Q, R, Qf):
			#context = self.CreateDefaultContext()
			#sym_system = self.ToSymbolic()
			#sym_context = sym_system.CreateDefaultContext()
			
			#prog = MathematicalProgram()
			#x = prog.NewIndeterminates(self.nX,'x')
			#ucon = prog.NewIndeterminates(self.nU,'u')
			
			times = xtraj.get_segment_times()
			
			# final state goal
			t = times[-1]
			x0 = xtraj.value(t).transpose()[0]
			u0 = utraj.value(t).transpose()[0]

			#sym_context.SetContinuousState(x) #x0+
			#sym_context.FixInputPort(0, ucon ) #u0+
			
			# xdot=f(x,u)==>xhdot=f(xh+x0,uh+u0)-x0dot ~= df/dx*xh + df/du*uh
			#f = sym_system.EvalTimeDerivatives(sym_context).CopyToVector() 
			#mapping = dict(zip(x, x0))
			#mapping.update(dict(zip(ucon, u0)))
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
			#import pdb; pdb.set_trace()
			sol = solve_ivp(lambda t,S: self.Cdre(t,S,Q,Rinv,xtraj,utraj), \
							[times[-1], times[0]], S0, t_eval=times[::-1])
			K = []
			S = []
			S_debug = []
			Ssol = np.fliplr(sol.y)  # flip is because we switch back to normal time progress
			for i in range(sol.y.shape[1]):
				x0 = xtraj.value(times[i]).transpose()[0]
				u0 = utraj.value(times[i]).transpose()[0]
				A, B = self.PlantDerivatives(x0, u0)
				
				s = Ssol[:,i].reshape(Qf.shape)
				k = Rinv.dot(B.transpose()).dot(s)
				if(len(K) == 0):
					K = np.ravel(k).reshape(self.nX*self.nU,1) 
					S_debug = np.ravel(s).reshape(self.nX*self.nX,1) 
				else:
					K = np.hstack( (K, np.ravel(k).reshape(self.nX*self.nU,1)) )
					S_debug = np.hstack( (S_debug, np.ravel(s).reshape(self.nX*self.nX,1)) )
			
			Kpp = PiecewisePolynomial.FirstOrderHold(times, K)
			Spp_debug = PiecewisePolynomial.FirstOrderHold(times, S_debug)
			
			# now that it's an autonomous system, get the Lyapunov function		
			prog = MathematicalProgram()
			x = prog.NewIndeterminates(self.nX, 'x')
			ucon = prog.NewIndeterminates(self.nU, 'u')
			
			sol = solve_ivp(lambda t,S: self.Cdre_CL(t,S,Q,Rinv,xtraj,utraj,x,ucon,Kpp), \
							[times[-1], times[0]], S0, t_eval=times[::-1])
			Ssol = np.fliplr(sol.y)  # flip is because we switch back to normal time progress
			for i in range(sol.y.shape[1]):
				s = Ssol[:,i].reshape(Qf.shape)
				if(len(S) == 0):
					S = np.ravel(s).reshape(self.nX*self.nX,1) 
				else:
					S = np.hstack( (S, np.ravel(s).reshape(self.nX*self.nX,1)) )	
				
			Spp = PiecewisePolynomial.FirstOrderHold(times, S)
			#import pdb; pdb.set_trace()
			return Kpp,Spp_debug #Spp
		
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
				
		def SystemTaylorExpansion(self, f, xstate, ucon, x0state, u0, x0dot, order=3):
			# get polynomial dynamics around (xd,ud)
			#import pdb; pdb.set_trace()
			
			x = np.append(xstate, ucon)
			x0 = np.append(x0state, u0)
			x_vars = range(len(x))
			mapping = dict(zip(xstate, x0state))
			mapping.update(dict(zip(ucon, u0)))
			df = 0.0*f
			
			for i in range(len(f)):
				f_sym = f[i]
				df[i] = f_sym.Substitute(mapping) # f(x0,u0)
				#func_expression = 'f%d = f(0) + '%(i)
				for n in range(1,order+1):
					p1 = itertools.product(x_vars, repeat=n) # combinations_with_replacement(x_vars, r=n)
					for pair in p1:
						df_n = 1.0
						df_sym = f_sym
						#df_sym_expr = 'df'
						#df_n_expr = ''
						for comp in pair:
							df_sym = df_sym.Differentiate(x[comp])
							#df_sym_expr = df_sym_expr + '_dx[%d]'%(comp)
							df_n = df_n * (x[comp]-x0[comp])
							#df_n_expr = df_n_expr + '*(x[%d]-x0[%d])'%(comp, comp)
						#func_expression = func_expression + str(df_sym_expr) + str(df_n_expr) + '/%d! + '%(n)
						df_n = df_n*df_sym.Evaluate(mapping)/factorial(n)
						df[i] = df[i] + df_n
					#func_expression = func_expression + '\n'
				#print(func_expression)
			#import pdb; pdb.set_trace()
			return df
		
		# evaluate the closed loop dynamics with the LQR feedback controller
		#  xhdot = f(x0+xh, u0-Kxh)  where xh=x-x0
		def EvalClosedLoopDynamics(self, x, ucon, x_d, u_d, x_d_dot, K, order=5):
			#import pdb; pdb.set_trace()
		
			sym_system = self.ToSymbolic()
			sym_context = sym_system.CreateDefaultContext()
			sym_context.SetContinuousState(x)
			sym_context.FixInputPort(0, ucon )
			f = sym_system.EvalTimeDerivatives(sym_context).CopyToVector() 
			if(True):
				#1) do polynomial system over the feedback system
				U = u_d - K.dot(x-x_d)  # feedback law
				fb_map = dict(zip(ucon, U))
				for i in range(len(f)):
					f[i] = f[i].Substitute(fb_map) - x_d_dot[i]
			
				f_fb = self.SystemTaylorExpansion(f, x, ucon, x_d, u_d, x_d_dot, order)
			else:
				#2) do feedback over the polynomial system
				f_fb = self.SystemTaylorExpansion(f, x, ucon, x_d, u_d, x_d_dot, order)
				U = u_d - K.dot(x-x_d)  # feedback law
				fb_map = dict(zip(ucon, U))
				for i in range(len(f_fb)):
					f_fb[i] = f_fb[i].Substitute(fb_map) - x_d_dot[i]
			
			return f_fb
		
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
				"""
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
				"""
				""" option 2: do algerbraic manipulations to find
					minimum value of this V = x'Sx   
				"""
				xmin, vmin = self.SampleCheck(prev_x, Vs[i])
				Vmin[i] = vmin
				#print('knot (%d): derivatives/optimization: %g/%g; ' %(i, vmin, Vmin[i]))
				#import pdb; pdb.set_trace()
   
			return Vmin

		def SampleCheck(self, x, V, Vdot=None):
			b = np.zeros((self.nX,1))
			H = np.zeros((self.nX,self.nX))
			
			for i in range(self.nX):
				b[i] = -V.Jacobian(x)[i].Evaluate(dict(zip(x,np.zeros(self.nX))))
				for j in range(self.nX):
					H[i,j] = V.Jacobian(x)[i].Jacobian(x)[j].Evaluate(dict(zip(x,np.zeros(self.nX))))
			
			min_x = np.linalg.solve(H, b)
			mapping = dict(zip(x, min_x))
			
			if Vdot is None:
				return min_x, V.Evaluate(mapping)
			else:
				return min_x, V.Evaluate(mapping), Vdot.Evaluate(mapping)
		
		def balanceQuadraticForm(self, S, P):
			V = np.linalg.inv(np.linalg.cholesky(S)).transpose() # Tests if S positive def. for us.
			U,l, __ = np.linalg.svd(V.transpose().dot(P.dot(V)))
			T = V.dot( U.dot( np.diag(l**(-0.25)) )  )
			#D = np.diag(l**(-0.5))
			return T #,D
			
		def CheckLevelSet(self, prev_x, x0, Vs, Vsdot, rho, multiplier_degree):
			prog = MathematicalProgram()
			x = prog.NewIndeterminates(len(prev_x),'x')
			V = Vs.Substitute(dict(zip(prev_x, x)))
			Vdot = Vsdot.Substitute(dict(zip(prev_x, x)))
			slack = prog.NewContinuousVariables(1,'a')[0]  
			#mapping = dict(zip(x, np.ones(len(x))))
			#V_norm = 0.0*V
			#for i in range(len(x)):
			#	basis = np.ones(len(x))
			#	V_norm = V_norm + V.Substitute(dict(zip(x, basis)))
			#V = V/V_norm
			#Vdot = Vdot/V_norm
			#prog.AddConstraint(V_norm == 0)

			# in relative state (lambda(xbar)
			Lambda = prog.NewSosPolynomial(Variables(x), multiplier_degree)[0].ToExpression() 
			Lambda = Lambda.Substitute(dict(zip(x, x-x0))) # switch to relative state (lambda(xbar)
			prog.AddSosConstraint(-Vdot + Lambda*(V - rho) - slack*V)
			prog.AddCost(-slack)
			#import pdb; pdb.set_trace()
			result = Solve(prog)
			if(not result.is_success()):
				print('%s, %s' %(result.get_solver_id().name(),result.get_solution_result()) )
				print('slack = %f' %(result.GetSolution(slack)) )
				print('Rho = %f' %(rho))
				#assert result.is_success()
				return -1.0
			
			return result.GetSolution(slack)
		'''
		# Attempts to find the largest funnel, defined by the time-varying
		# one-level set of V, which verifies (using SOS over state at finite 
		# sample points in time) initial conditions to end inside the one-level set
		# of the goal region G at time ts(end).  
		# from the paper "Invariant Funnels around Trajectories using Sum-of-Squares Programming", Tobenkin et al.
		def TimeVaryingLyapunovSearch(self, prev_x, Vs, Vdots, Ts, times, xtraj, utraj, \
										 rho_f, multiplier_degree=None, debug=True):
			
			deg_V = 2
			tries = 40
			prev_rhointegral = 0.
			N = len(times)-1
			
			dt = np.diff(times)
			rho = np.linspace(rho_f/2.0, rho_f, N+1)
			
			for idx in range(tries):
				# start of number of iterations if we want
				rhodot = np.diff(rho)/dt
				# sampleCheck()
				Lambda_vec = []
				x_vec = []
				
				#fix rho, optimize Lagrange multipliers
				for i in range(N):
					Vdot = Vdots[i].Substitute(dict(zip(prev_x, x)))
					x0 = xtraj.value(times[i]).transpose()[0]
					Ttrans = np.linalg.inv(Ts[i])
					x0 = Ttrans.dot(x0)
					
					prog = MathematicalProgram()
					x = prog.NewIndeterminates(len(prev_x),'x')
					switch_x = dict(zip(prev_x, x))

					# Construct a polynomial V that contains all monomials with x,y,theta up to degree 
					V = prog.NewFreePolynomial(Variables(x), deg_V).ToExpression()

					# Add a constraint to enforce that V is strictly positive away from x0.
					# (Note that because our coordinate system is sine and cosine, V is also zero
					# at theta=2pi, etc).
					eps = 1e-4
					constraint1 = prog.AddSosConstraint(V - eps*(x-x0).dot(x-x0))

					# Construct the polynomial which is the time derivative of V.
					Vdot = V.Jacobian(x).dot(f_cl)

					# Construct a polynomial L representing the "Lagrange multiplier".
					L = prog.NewFreePolynomial(Variables(x), multiplier_degree).ToExpression()
					L = L.Substitute(dict(zip(x, x-x0))) # switch to relative state (lambda(xbar)

					# Add a constraint that Vdot is strictly negative 
					gamma = prog.NewContinuousVariables(1,'g')[0] 
					constraint2 = prog.AddSosConstraint( -gamma*V - (Vdot-rhodot[i] + Lambda*(rho[i]-V)) ) 
					prog.AddCost(-gamma) #maximize gamma

					# exception for the upright fixed point by multipling by s^2).
					#constraint2 = prog.AddSosConstraint(-Vdot - L*(s**2+c**2-1) -
					#									eps*(x-x0).dot(x-x0)*s**2)

					# Add V(0) = 0 constraint
					constraint3 = prog.AddLinearConstraint( \
						V.Substitute(dict(zip(x, x0+1.0*np.ones(self.nX) ))) == 1)

					# Add V(theta=pi) = mgl, just to set the scale.
					#constraint4 = prog.AddLinearConstraint( \
					#	V.Substitute({s: 1, c: 0, thetadot: 0}) == p.mass()*p.gravity()*p.length())

					# Call the solver.
					result = Solve(prog)
					assert(result.is_success())
					Lambda_vec.append(result.GetSolution(Lambda))
					slack = result.GetSolution(gamma)
					if(debug):
						print('Slack #%d = %f' %(idx, slack))
					x_vec.append(x)
		'''
		
		# Attempts to find the largest funnel, defined by the time-varying
		# one-level set of V, which verifies (using SOS over state at finite 
		# sample points in time) initial conditions to end inside the one-level set
		# of the goal region G at time ts(end).  
		# from the paper "Invariant Funnels around Trajectories using Sum-of-Squares Programming", Tobenkin et al.
		def TimeVaryingLyapunovSearchRho(self, prev_x, Vs_orig, Vdots_orig, Ts, all_fcl, times, xtraj, utraj, \
										 rho_f, multiplier_degree=None, debug=True, find_V=False):
			C = 1.0 #8.0
			#rho_f = 3.0
			tries = 3 #40
			prev_rhointegral = 0.
			N = len(times)-1
			#Vmin = self.minimumV(prev_x, Vs) #0.05*np.ones((1, len(times))) # need todo something here instead of this!! 
			dt = np.diff(times)
			rho = np.flipud(rho_f*np.exp(-C*(np.array(times)-times[0])/(times[-1]-times[0])))# + np.max(Vmin) 
			#rho = np.linspace(0.1, rho_f, N+1)
			#rho = np.linspace(rho_f/2.0, rho_f, N+1)
			#rho = np.linspace(rho_f, rho_f, N+1)
			#import pdb; pdb.set_trace()
			Vs    = Vs_orig
			Vdots = Vdots_orig
			
			if(debug):
				fig, ax = plt.subplots()
				fig.suptitle('Rho progression')
				ax.set(xlabel='index', ylabel='rho')
				ax.grid(True)
				plt.show(block = False)
				
			need_to_break = False
			# iterations of bilinear optimizations
			for idx in range(tries):
				# start of number of iterations if we want
				if(debug):
					print('starting iteration #%d with rho=' %(idx))
					print(rho)
					ax.plot(rho)
					plt.pause(0.05)
					
				rhodot = np.diff(rho)/dt
				Lambda_vec = [] # store the lamda's for this iteration
				x_vec      = [] # store the prog indeterminates 

				#fix rho and V, optimize Lagrange multipliers
				for i in range(N):
					prog = MathematicalProgram()
					x = prog.NewIndeterminates(len(prev_x),'y')
					if(find_V):
						# because I'm switching the prev_x from previous iteration to be consistent
						V = Vs[i].Substitute(dict(zip(prev_x, x)))
						Vdot = Vdots[i].Substitute(dict(zip(prev_x, x)))
					else:
						V = Vs[i].Substitute(dict(zip(prev_x, x)))
						Vdot = Vdots[i].Substitute(dict(zip(prev_x, x)))
						
					x0 = xtraj.value(times[i]).transpose()[0]
					Ttrans = np.linalg.inv(Ts[i])
					x0 = Ttrans.dot(x0)
					#xmin, vmin, vdmin = self.SampleCheck(x, V, Vdot)
					#if(vdmin > rhodot[i]):
					#	print('Vdot is greater than rhodot!')
					
					# this is too restrictive
					#Lambda = prog.NewSosPolynomial(Variables(x), multiplier_degree)[0].ToExpression()
					Lambda = prog.NewFreePolynomial(Variables(x), multiplier_degree).ToExpression()
					Lambda = Lambda.Substitute(dict(zip(x, x-x0))) # switch to relative state lambda(xbar)
					gamma  = prog.NewContinuousVariables(1,'g')[0] # the slack variable to optimize
					# Jdot-rhodot+Lambda*(rho-J) < -gamma
					prog.AddSosConstraint( -gamma*V - (Vdot-rhodot[i] + Lambda*(rho[i]-V)) ) 
					prog.AddCost(-gamma) #maximize gamma
					result = Solve(prog)
					if result.is_success() == False:
						need_to_break = True
						print('Solver could not solve anymore')
						#import pdb; pdb.set_trace()
						break
					else:
						# we're good, keep lambdas and x's for sencond optimization
						Lambda_vec.append(result.GetSolution(Lambda))
						slack = result.GetSolution(gamma)
						if(debug):
							print('Slack #%d = %f' %(idx, slack))
						x_vec.append(x)
						if(slack < 0.0):
							print('In iter#%d, found negative slack so going to end prematurely... :(' %(idx))
							need_to_break = True
				
				# break if couldn't find a solution or slack is not positive anymore
				if(need_to_break == True):
					break;
				
				#import pdb; pdb.set_trace()
				# now, fix Lagrange multipliers and maximize rho and find a better V
				rhointegral = 0.0
				prog = MathematicalProgram()
				xx   = prog.NewIndeterminates(len(x),'z')
				t    = prog.NewContinuousVariables(N,'r')
				#import pdb; pdb.set_trace()
				rho_x = np.concatenate((t,[rho[-1]])) #+ rho 
				V        = list(Vs)
				V[-1]    = V[-1].Substitute(dict(zip(prev_x, xx)))
				Vdot     = list(Vdots)
				Vdot[-1] = Vdot[-1].Substitute(dict(zip(prev_x, xx)))
				
				#
				# loop over the whole trajectory to find the integral. go backwards to get dV/dt
				for i in range(N-1, 4, -1):
					print(i)
					if(i==5):
						import pdb; pdb.set_trace()
					#prog.AddConstraint(t[i]>=0.0)  # does this mean [prog,rho] = new(prog,N,'pos'); in matlab??
					rhod_x = (rho_x[i+1]-rho_x[i])/dt[i]
					#prog.AddConstraint(rhod_x<=0.0)
					rhointegral = rhointegral + rho_x[i]*dt[i] + 0.5*rhod_x*(dt[i]**2)
					x0 = xtraj.value(times[i]).transpose()[0]
					Ttrans = np.linalg.inv(Ts[i])
					x0 = Ttrans.dot(x0)
					
					if(find_V):
						# find a new V
						f_cl = all_fcl[i] # just to "allocate" the same variable
						for j in range(self.nX):
							f_cl[j] = all_fcl[i][j].Substitute(dict(zip(prev_x, xx)))
						# Construct a polynomial V that contains all monomials with x,y,theta up to degree 2
						deg_V = 2
						eps   = 1e-4
						# run over the previous, except the last V, and make a decision-variable of coeff.
						V[i] = prog.NewFreePolynomial(Variables(xx), deg_V).ToExpression() 
						V[i] = V[i].Substitute(dict(zip(xx, xx-x0)))
						# Add a constraint to enforce that V is strictly positive away from x0.
						constraint1 = prog.AddSosConstraint(V[i] - eps*(xx-x0).dot(xx-x0))
						# Construct the polynomial which is the time derivative of V.
						Vdot[i] = V[i].Jacobian(xx).dot(f_cl) + (V[i+1]-V[i])/dt[i] 
						# Add V(1,1,...) = V_guess(1,1,...) constraint. give it some sort of scaling.
						x_add = np.ones(self.nX)
						#x_add[-1] = 0.0 # the teta state
						constraint3 = prog.AddLinearConstraint(
    						V[i].Substitute(dict(zip(xx, x0 + x_add))) == 
						    Vs_orig[i].Substitute(dict(zip(prev_x, x0 + x_add)))  )

						#x0   = xtraj.value(times[i]).transpose()[0]
						L1   = Lambda_vec[i].Substitute(dict(zip(x_vec[i], xx)))
						#Vdot = Vdot*rho_x[i] - V*rhod_x
						prog.AddSosConstraint( -(Vdot[i] - rhod_x + L1 * ( rho_x[i]-V[i] ) ) )

					else:
						# use the V provided by the LQR routine
						V    = Vs[i].Substitute(dict(zip(prev_x, xx)))
						Vdot = Vdots[i].Substitute(dict(zip(prev_x, xx)))
						#x0   = xtraj.value(times[i]).transpose()[0]
						L1   = Lambda_vec[i].Substitute(dict(zip(x_vec[i], xx)))
						#Vdot = Vdot*rho_x[i] - V*rhod_x
						prog.AddSosConstraint( -(Vdot - rhod_x + L1 * ( rho_x[i]-V ) ) )

				prog.AddCost(-rhointegral)
				result = Solve(prog)
				if (result.is_success() == False):
					import pdb; pdb.set_trace()
				rhos = result.GetSolution(rho_x)
				
				# set it back to the original x's for next iteration
				if(find_V):
					for i in range(N-1,-1,-1):
						Vsol = result.GetSolution(V[i])
						Vs[i] = Vsol.Substitute(dict(zip(xx, prev_x)))
						f_cl = all_fcl[i] # just to "allocate" the same variable
						#for j in range(self.nX):
						#	f_cl[j] = all_fcl[i][j].Substitute(dict(zip(prev_x, xx)))
						Vdots[i] = Vs[i].Jacobian(prev_x).dot(f_cl) + (Vs[i+1]-Vs[i])/dt[i] #Vsoldot
						#Vdots[i] = Vsoldot.Substitute(dict(zip(xx, prev_x)))
						
				rho = []
				for r in rhos:
					rho.append(r[0].Evaluate())
				
				rhointegral = result.GetSolution(rhointegral).Evaluate()
				if( (rhointegral-prev_rhointegral)/rhointegral < 1E-5): # 0.1%
					print('Rho integral converged')
					need_to_break = True
					break;
				else:	
					prev_rhointegral = rhointegral
					print('End of iteration #%d: rhointegral=%f' %(idx, rhointegral))
					if(need_to_break == True):
						print('In iter#%d, found negative slack so ending prematurely... :(' %(idx))
						break;
				
				# end of iterations if we want
			if(debug):
				ax.plot(rho)
				plt.pause(0.05)
			print('done computing funnel.\nFinal rho= ')
			print(rho)
			#import pdb; pdb.set_trace()
			# because in this case, we parameterize with V=V/rho
			#if(find_V == False):
			for i in range(len(rho)):
				Vs[i] = Vs[i]/rho[i]
					
			return Vs
		
		def FindInitialSet(self, x, ucon, xtraj, xdotraj, utraj, times, Q, R, zero_map, \
						   do_balancing=False, find_V=False):
			do_normalization = False
			deg_L = 4
			# get the final ROA to be the initial condition (of t=end) of the S from Ricatti)
			tf = times[-1]
			xf = xtraj.value(tf).transpose()[0]
			xdf = xdotraj.value(tf).transpose()[0]
			uf = utraj.value(tf).transpose()[0]
			Af, Bf = self.PlantDerivatives(xf, uf) #0.0*uf)
			Kf, __ = LinearQuadraticRegulator(Af, Bf, Q, R)
			f_cl   = self.EvalClosedLoopDynamics(x, ucon, xf, uf, xdf, Kf, order=3) # for dynamics CL
			# get a polynomial representation of f_closedloop, xdot = f_cl(x)
			# where x is actually in rel. coord.
			f_Lcl   = self.EvalClosedLoopDynamics(x, ucon, xf, uf, xdf, Kf, order=1) # linearization CL
			Af     = Evaluate(Jacobian(f_Lcl, x), zero_map) # because this is rel. coord.
			Qf 	   = RealContinuousLyapunovEquation(Af, Q)
			# make sure Lyapunov candidate is pd
			if (self.isPositiveDefinite(Qf)==False):
				assert False, '******\nQf is not PD for t=%f\n******' %(tf)
			Vf = (x-xf).transpose().dot(Qf.dot((x-xf)))

			#import pdb; pdb.set_trace()
			if(do_normalization):
				#coeffs = Polynomial(Vf).monomial_to_coefficient_map().values()
				#sumV = 0.
				#for coeff in coeffs:
				#	sumV = sumV + np.abs(coeff.Evaluate())
				sumV = Vf.Evaluate(dict(zip(x, xf+np.ones(self.nX)))) #do: V(1,1,...)=1
				Vf = Vf / sumV  #normalize coefficient sum to one
				Qf = Qf / sumV
			# this part is not giving good results so I temporarilly disable it
			find_V = False
			if(find_V == True):
				deg_V = 2
				eps   = 1e-4
				
				f_cl_save = f_cl.copy()
				rho = 1. # guess
				
				# do bilinear optimization
				for i in range(2):
					# fix V, rho
					prog1 = MathematicalProgram()
					xx   = prog1.NewIndeterminates(self.nX, 'y')
					Vf = Vf.Substitute(dict(zip(x, xx)))
					for j in range(self.nX):
						f_cl[j] = f_cl_save[j].Substitute(dict(zip(x, xx)))
					Vfdot = Vf.Jacobian(xx).dot(f_cl)
					Lambda = prog1.NewFreePolynomial(Variables(xx), deg_L).ToExpression()
					Lambda = Lambda.Substitute(dict(zip(xx, xx-xf))) # switch to relative state lambda(xbar)
					gamma  = prog1.NewContinuousVariables(1,'g')[0] # the slack variable to optimize
					prog1.AddSosConstraint( -gamma*Vf - (Vfdot + Lambda*(rho-Vf)) ) 
					prog1.AddCost(-gamma) #maximize gamma
					#
					result = Solve(prog1)
					Lambda = result.GetSolution(Lambda)
					slack  = result.GetSolution(gamma)
					# fix L, optimize rho and V	
					prog2 = MathematicalProgram()
					xxx   = prog2.NewIndeterminates(self.nX, 'z')
					rho   = prog2.NewContinuousVariables(1, 'r')[0]
					Vfnew = prog2.NewFreePolynomial(Variables(xxx), deg_V).ToExpression()
					Vfnew = Vfnew.Substitute(dict(zip(xxx, xxx-xf)))
					# Add a constraint to enforce that V is strictly positive away from xf.
					prog2.AddSosConstraint(Vfnew - eps*(xxx-xf).dot(xxx-xf))
					# Construct the polynomial which is the time derivative of V.
					for j in range(self.nX):
						f_cl[j] = f_cl_save[j].Substitute(dict(zip(x, xxx)))
					Vfdot = Vfnew.Jacobian(xxx).dot(f_cl)
					# Construct a polynomial L representing the "Lagrange multiplier".
					Lambda = Lambda.Substitute(dict(zip(xx, xxx)))
					# Add a constraint that Vdot is strictly negative away from x0
					prog2.AddSosConstraint( -Vfdot + Lambda*(Vfnew-rho) )
					# just some scaling/normalization
					prog2.AddLinearConstraint(
						Vfnew.Substitute(dict(zip(xxx, xf + np.ones(self.nX)))) == 
						Vf.Substitute(dict(zip(xx,     xf + np.ones(self.nX)))) )

					prog2.AddCost(-rho)
					# Call the solver.
					result = Solve(prog2)
					assert(result.is_success())
					Vf = result.GetSolution(Vfnew)
					Vf = Vf.Substitute(dict(zip(xxx, x)))
					rho = result.GetSolution(rho)
				#import pdb; pdb.set_trace()
				Qf = Evaluate(0.5*Jacobian(Vf.Jacobian(x),x), zero_map)
				rho_f = rho
				print('Qf')
				print(Qf)
				print('rho=')
				print(rho)
			else:
				Vfdot = Vf.Jacobian(x).dot(f_cl) # we're just doing the static final point to get Rho_f
				#H = Evaluate(0.5*Jacobian(Vfdot.Jacobian(x),x), zero_map)
				#if (self.isPositiveDefinite(-H)==False):
				#	assert False, '******\nVdot is not ND at the end point, for t=%f\n******' %(tf)

				if(do_balancing):
					#import pdb; pdb.set_trace()
					S1 = Evaluate(0.5*Jacobian(Vf.Jacobian(x),x), zero_map)
					S2 = Evaluate(0.5*Jacobian(Vfdot.Jacobian(x),x), zero_map)
					T = self.balanceQuadraticForm(S1, S2)
					balanced_x = T.dot(x)
					balance_map = dict(zip(x, balanced_x))
					Vf = Vf.Substitute(balance_map)
					Vfdot = Vfdot.Substitute(balance_map)
					for i in range(len(f_cl)):
						f_cl[i] = f_cl[i].Substitute(balance_map)
					xf = np.linalg.inv(T).dot(xf) #the new coordinates of the equilibrium point

				rhomin = 0.0
				rhomax = 1.0

				#import pdb; pdb.set_trace()
				#deg_L = Polynomial(Vfdot).TotalDegree()
				# First bracket the solution
				while self.CheckLevelSet(x, xf, Vf, Vfdot, rhomax, multiplier_degree=deg_L) > 0:
					rhomin = rhomax
					rhomax = 1.2*rhomax

				#print('Rho_max = %f' %(rhomax))
				tolerance = 1e-4
				slack = -1.0
				while rhomax - rhomin > tolerance:
					rho_f = (rhomin + rhomax)/2
					slack = self.CheckLevelSet(x, xf, Vf, Vfdot, rho_f, multiplier_degree=deg_L)
					if  slack >= 0:
						rhomin = rho_f
					else:
						rhomax = rho_f

				rho_f = (rhomin + rhomax)/2
				rho_f = rho_f*0.8 # just to be on the safe-side. REMOVE WHEN WORKING
				print('Rho_final(t=%f) = %f; slack=%f' %(tf, rho_f, slack))
				#import pdb; pdb.set_trace()
				
			return Qf, rho_f
		
		def RegionOfAttraction(self, xtraj, utraj, V=None, debug=True, find_V=False):
			# Construct a polynomial V that contains all monomials with s,c,thetadot up to degree 2.
			#deg_V = 2
			do_balancing     = False #True
			do_use_Slti      = True
			# Construct a polynomial L representing the "Lagrange multiplier".
			deg_L = 4
			Q  = 10.0*np.eye(self.nX)
			R  = 1.0*np.eye(self.nU)
			Qf = 1.0*np.eye(self.nX)
			rho_f = 1.0
			xdotraj = xtraj.derivative(1)
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
			#sym_system = self.ToSymbolic()
			#sym_context = sym_system.CreateDefaultContext()
			#sym_context.SetContinuousState(x)
			#sym_context.FixInputPort(0, ucon )
			#f = sym_system.EvalTimeDerivativesTaylor(sym_context).CopyToVector() 
						
			times   = xtraj.get_segment_times()
			all_V   = [] #might be transformed
			all_Vd  = [] #might be transformed
			all_Vd2 = [] 
			all_fcl = [] #might be transformed
			all_T   = [] #might be transformed
			all_x0  = [] #might be transformed
			all_K   = []
			sumV = 0.0
			zero_map = dict(zip(x,np.zeros(self.nX)))
			if(do_use_Slti):
				Qf, rho_f = self.FindInitialSet(x, ucon, xtraj, xdotraj, utraj, times, Q, R, zero_map, \
												do_balancing=do_balancing, find_V=find_V)
				
			#import pdb; pdb.set_trace()
			# end of getting initial conditions
			if V is None:
				# Do optimization to find the Lyapunov candidate.
				#print('******\nRunning SOS to find Lyapunov function ...') 
				#V, Vdot = self.findLyapunovFunctionSOS(xtraj, utraj, deg_V, deg_L)
				# or Do tvlqr to get S
				print('******\nRunning TVLQR ...')
				K, S  = self.TVLQR(xtraj, utraj, Q, R, Qf) 
				#S0 = S.value(times[-1]).reshape(self.nX, self.nX)
				#print(Polynomial(x.dot(S0.dot(x))).RemoveTermsWithSmallCoefficients(1e-6))
				print('Done\n******')
				for t in times:
					x0 = xtraj.value(t).transpose()[0]
					xd0 = xdotraj.value(t).transpose()[0]
					u0 = utraj.value(t).transpose()[0]
					K0 = K.value(t).reshape(self.nU, self.nX)
					all_K.append(K0)
					S0 = S.value(t).reshape(self.nX, self.nX)
					S0d = S.derivative(1).value(t).reshape(self.nX, self.nX) # Sdot
					# make sure Lyapunov candidate is pd
					if (self.isPositiveDefinite(S0)==False):
						assert False, '******\nS is not PD for t=%f\n******' %(t)
					# for debugging
					#S0 = np.eye(self.nX)
					#V = x.dot(S0.dot(x))
					V = (x-x0).transpose().dot(S0.dot((x-x0)))
					# normalization of the lyapunov function
					#if(do_normalization):
						#coeffs = Polynomial(V).monomial_to_coefficient_map().values()
						#sumV = 0.
						#for coeff in coeffs:
						#	sumV = sumV + np.abs(coeff.Evaluate())
						#sumV = V.Evaluate(dict(zip(x, x0+np.ones(self.nX)))) #do: V(1,1,...)=1
						#V = V / sumV  #normalize coefficient sum to one
					# get a polynomial representation of f_closedloop, xdot = f_cl(x)
					f_cl = self.EvalClosedLoopDynamics(x, ucon, x0, u0, xd0, K0, order=3)
					#import pdb; pdb.set_trace()
					# vdot = x'*Sdot*x + dV/dx*fcl_poly
					#Vdot = (x.transpose().dot(S0d)).dot(x) + V.Jacobian(x).dot(f_cl(xbar)) 
					Vdot = ((x-x0).transpose().dot(S0d)).dot((x-x0)) + V.Jacobian(x).dot(f_cl) 
					#deg_L = np.max([Polynomial(Vdot).TotalDegree(), deg_L])
					if(do_balancing):
						#import pdb; pdb.set_trace()
						S1 = Evaluate(0.5*Jacobian(V.Jacobian(x),x), zero_map)
						S2 = Evaluate(0.5*Jacobian(Vdot.Jacobian(x),x), zero_map)
						T = self.balanceQuadraticForm(S1, S2)
						balanced_x = T.dot(x)
						balance_map = dict(zip(x, balanced_x))
						V = V.Substitute(balance_map)
						Vdot = Vdot.Substitute(balance_map)
						for i in range(len(f_cl)):
							f_cl[i] = f_cl[i].Substitute(balance_map)
						x0 = np.linalg.inv(T).dot(x0) #the new coordinates of the equilibrium point
					else:
						T = np.eye(self.nX)
					
					# store it for later use
					all_V.append(V)
					all_fcl.append(f_cl)
					all_Vd.append(Vdot)
					all_T.append(T)
					all_x0.append(x0)
			
			for i in range(len(times)-1):
				xd0 = xdotraj.value(times[i]).transpose()[0]
				Vdot = (all_V[i+1]-all_V[i])/(times[i+1]-times[i]) + all_V[i].Jacobian(x).dot(all_fcl[i]) 
				all_Vd2.append(Vdot)
			#import pdb; pdb.set_trace()
			#rho_f = 1.0
			# time-varying PolynomialLyapunovFunction who's one-level set defines the verified invariant region
			V = self.TimeVaryingLyapunovSearchRho(x, all_V, all_Vd, all_T, all_fcl, times, xtraj, utraj, rho_f, \
												  multiplier_degree=deg_L, debug=debug, find_V=find_V)
			# Check Hessian of Vdot at origin
			#H = Evaluate(0.5*Jacobian(Vdot.Jacobian(x),x), dict(zip(x, x0)))
			#assert isPositiveDefinite(-H), "Vdot is not negative definite at the fixed point."
			return V, all_K
		
		def my_plot_sublevelset_expression(self, ax, e, x0, vertices=51, color=(0.1,0.5,0.8)): #, **kwargs):
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

			return ax.fill(X[0, :]+x0[0], X[1, :]+x0[1], color=color)
			#return plot_sublevelset_quadratic(ax, A, b, c, vertices, **kwargs)

	return Impl


# main function, finding the maximal funnel
def runFunnel():
	print('******\nrunning Funnel algorithm ...\n******')
	
    # Declare pendulum model
	plant = DubinsCarPlant_[float]() #None]  # Default instantiation
		
	# Trajectory optimization to get nominal trajectory
	#x0 = (0.0, 0.0, -math.pi/4.0)  #Initial state that trajectory should start from
	#xf = (3.0, 0.0, math.pi/4.0)  #Final desired state
	x0 = (0.0, 0.0, 0.0)
	xf = (3.0, 2.0, 0.0*math.pi/180.0)
	dist = np.linalg.norm(np.array(xf)-np.array(x0))
	tf0 = dist/(plant.umax*0.8) # Guess for how long trajectory should take
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
	#import pdb; pdb.set_trace()
	# plotting stuff
	fig, ax = plt.subplots(2)
	fig.suptitle('Direct collocation trajectory optimization (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)' %\
				 (x0[0],x0[1],x0[2],xf[0],xf[1],xf[2]))
	ax[0].plot(xy_knots[0, :], xy_knots[1, :])
	ax[0].set(xlabel='x [m]', ylabel='y [m]')
	ax[0].grid(True)
	ax[1].plot(times, u_values[0,:], 'green')
	ax[1].plot(times, u_values[1,:], 'red')
	ax[1].set(xlabel='t [sec]', ylabel='v [m/s] or omega [rad/sec]')
	ax[1].grid(True)
	plt.show(block = False)
	#import pdb; pdb.set_trace()
	#prog = MathematicalProgram()
	#x = prog.NewIndeterminates(plant.nX, 'x')
	#ucon = prog.NewIndeterminates(plant.nU, 'u')
	#times = xtraj.get_segment_times()
	#x0 = xtraj.value(times[-1]).transpose()[0]
	#u0 = [0.0] #utraj.value(times[-1]).transpose()[0]
	#Af, Bf = plant.PlantDerivatives(x0, u0)
	#Q = 10.*np.eye(plant.nX)
	#R = 1.*np.eye(plant.nU)
	#Kf, Qf = LinearQuadraticRegulator(Af, Bf, Q, R)
	#sym_system = plant.ToSymbolic()
	#sym_context = sym_system.CreateDefaultContext()
	#sym_context.SetContinuousState(x)
	#sym_context.FixInputPort(0, ucon )
	#f = sym_system.EvalTimeDerivatives(sym_context).CopyToVector() 
	#f_poly = plant.SystemTaylorExpansion(f, x, ucon, xf, u0, np.zeros((2,1)), order=3)
	# region of attraction:
	#context = plant.CreateDefaultContext()
	V, __ = plant.RegionOfAttraction(xtraj, utraj, debug=True, find_V=True)
	
	fig1, ax1 = plt.subplots()
	ax1.set_xlim([-5.0, 5.0])
	ax1.set_ylim([-5.0, 5.0])
	ax1.set_xlabel('x')
	ax1.set_ylabel('y')
	fig1.suptitle('Car: Funnel for trajectory')
	times = xtraj.get_segment_times()
	for i in range(len(times)):
		x0 = xtraj.value(times[i]).transpose()[0]
		plant.my_plot_sublevelset_expression(ax1, V[i], x0)
		plt.pause(0.05)
		#import pdb; pdb.set_trace()
	ax1.plot(xy_knots[0, :], xy_knots[1, :], 'k')
	plt.show(block = True)	

	
	
if __name__ == "__main__":
	runFunnel()
	
	
