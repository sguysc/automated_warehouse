import numpy as np
import matplotlib.pyplot as plt
import math

from pydrake.all import (DirectCollocation, PiecewisePolynomial, BasicVector_, LeafSystem_, Solve, FirstOrderTaylorApproximation, LinearQuadraticRegulator, Linearize, Variables, MathematicalProgram, Evaluate, Jacobian, RealContinuousLyapunovEquation, Substitute)
from pydrake.all import (Simulator, DiagramBuilder, VectorSystem)
from pydrake.all import (plot_sublevelset_quadratic)
from pydrake.systems.scalar_conversion import TemplateSystem
from pydrake import symbolic
from pydrake.symbolic import Polynomial
from pydrake.symbolic import Expression
from pydrake.solvers.scs import ScsSolver
from pydrake.solvers.ipopt import IpoptSolver
from pydrake.solvers.mosek import MosekSolver

def sublevelset(ax, e, x0, vertices=51, color=(0.1,0.5,0.8)): #, **kwargs):
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
		
prog = MathematicalProgram()
x    = prog.NewIndeterminates(2, 'x')  
V1   = prog.NewSosPolynomial(Variables(x), 2)[0].ToExpression()
V2   = prog.NewSosPolynomial(Variables(x), 2)[0].ToExpression()

a = 0.5*Jacobian(V1.Jacobian(x),x) 
b = 0.5*Jacobian(V2.Jacobian(x),x) 

pxi = np.array([[-0.5,-0.5], [-0.5, 0.5], [0.5,0.5], [0.5,-0.5]])
for pt in pxi:
	prog.AddConstraint( pt.T.dot(a).dot(pt) <= 1.0)
	prog.AddConstraint( pt.T.dot(b).dot(pt) <= 1.0)
pxi = np.array([[0.0, 1.0] ])
prog.AddConstraint( pxi[-1].T.dot(b).dot(pxi[-1]) <= 1.0)

prog.AddMaximizeLogDeterminantSymmetricMatrixCost(a)
prog.AddMaximizeLogDeterminantSymmetricMatrixCost(b)

result = Solve(prog)
assert(result.is_success())
V1 = result.GetSolution(V1)
V2 = result.GetSolution(V2)

fig1, ax1 = plt.subplots()
ax1.set_xlim([-5.0, 5.0])
ax1.set_ylim([-5.0, 5.0])
ax1.set_xlabel('x')
ax1.set_ylabel('y')
	
sublevelset(ax1, V2, np.array([0,0]), vertices=51, color=(0.1,0.8,0.8))
sublevelset(ax1, V1, np.array([0,0]), vertices=51, color=(0.1,0.5,0.8))
plt.show(block = True)


