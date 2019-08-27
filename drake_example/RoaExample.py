import numpy as np
import matplotlib.pyplot as plt

from pydrake.examples.van_der_pol import VanDerPolOscillator

x = VanDerPolOscillator.CalcLimitCycle()

fig, ax = plt.subplots()
ax.plot(x[0,:], x[1,:], color='k', linewidth=2)
ax.set_xlim([-2.5, 2.5])
ax.set_ylim([-3, 3])
ax.set_xlabel('q')
ax.set_ylabel('qdot')

# TODO(russt): Consider putting this into drake.math?
def isPositiveDefinite(M):
    assert np.size(M,0) == np.size(M, 1)

    for i in range(1, np.size(M,0)+1):
      if(np.linalg.det(M[0:i, 0:i]) <= 0):
        return False;
                   
    return True

from pydrake.all import (Evaluate, Jacobian, MathematicalProgram, Polynomial, 
                         RealContinuousLyapunovEquation,
                         Solve, Variables)
from pydrake.all import MosekSolver

# https://github.com/RobotLocomotion/drake/blob/last_sha_with_original_matlab/drake/matlab/systems/%40PolynomialSystem/regionOfAttraction.m

def FixedLyapunovSearchRho(prog, x, V, Vdot, multiplier_degree=None):
    '''
    Assumes V>0.
    V <= rho => Vdot <= 0 via
        find Lambda subject to
          -Vdot + Lambda*(V - rho) is SOS, 
          Lambda is SOS.
    '''

    def CheckLevelSet(prev_x, prev_V, prev_Vdot, rho, multiplier_degree):
        #import pdb; pdb.set_trace()
        prog = MathematicalProgram()
        x = prog.NewIndeterminates(len(prev_x),'x')
        V = prev_V.Substitute(dict(zip(prev_x, x)))
        Vdot = prev_Vdot.Substitute(dict(zip(prev_x, x)))
        slack = prog.NewContinuousVariables(1,'a')[0]

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
    # Check that x0 is a fixed point.
    xdot0 = system.EvalTimeDerivatives(context).get_vector().CopyToVector()
    assert np.allclose(xdot0, 0*xdot0), "context does not describe a fixed point."

    sym_system = system.ToSymbolic()
    sym_context = sym_system.CreateDefaultContext()
    
    prog = MathematicalProgram()
    x = prog.NewIndeterminates(sym_context.num_continuous_states(),'x')

    # Evaluate the dynamics (in relative coordinates)
    sym_context.SetContinuousState(x0+x)
    f = sym_system.EvalTimeDerivatives(sym_context).get_vector().CopyToVector()
    #import pdb; pdb.set_trace()
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

from pydrake.all import SymbolicVectorSystem, Variable, plot_sublevelset_expression

# Time-reversed van der Pol oscillator
mu = 1;
q = Variable('q')
qdot = Variable('qdot')
vdp = SymbolicVectorSystem(state=[q,qdot], dynamics=[-qdot, mu*(q*q - 1)*qdot + q])
context = vdp.CreateDefaultContext()
context.SetContinuousState([0, 0])

V = RegionOfAttraction(vdp, context)

plot_sublevelset_expression(ax, V)
plt.show()