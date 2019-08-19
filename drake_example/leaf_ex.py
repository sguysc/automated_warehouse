import numpy as np
import matplotlib.pyplot as plt
import math

from pydrake.all import (DirectCollocation, PiecewisePolynomial, BasicVector_, LeafSystem_, Simulator, Solve, FirstOrderTaylorApproximation, LinearQuadraticRegulator, Linearize, Variables, MathematicalProgram)
from pydrake.all import (DiagramBuilder, SignalLogger, Saturation, PortDataType, plot_sublevelset_expression)
from pydrake.systems.scalar_conversion import TemplateSystem

@TemplateSystem.define("ex1_")
def ex1_(T):
	class Impl(LeafSystem_[T]):
		def _construct(self, converter=None):
			LeafSystem_[T].__init__(self, converter)
			# one inputs 
			self.DeclareVectorInputPort("u", BasicVector_[T](1))
			# two outputs (full state)
			self.DeclareVectorOutputPort("x", BasicVector_[T](2), self.CopyStateOut)
			# two positions, no velocities
			self.DeclareContinuousState(2, 0, 0)

		def _construct_copy(self, other, converter=None):
			Impl._construct(self, converter=converter)

		def CopyStateOut(self, context, output):
			x = context.get_continuous_state_vector().CopyToVector()
			output.SetFromVector(x) # = y 

		def DoCalcTimeDerivatives(self, context, derivatives):
			x = context.get_continuous_state_vector().CopyToVector()
			u = self.EvalVectorInput(context, 0).CopyToVector()
			
			xdot[:] = np.array([-x[0] - x[1],  -x[1] - x[0] + u[0]]) #.reshape(3,1) #u[0]
			derivatives.get_mutable_vector().SetFromVector(xdot)

	
	return Impl
	
if __name__ == "__main__":
    # Declare model
	plant = ex1_[None]  # Default instantiation
		
	#context = plant.CreateDefaultContext(DubinsPlant_[None]())
	context = plant().CreateDefaultContext()
	#import pdb; pdb.set_trace()
	sym_system = plant.ToSymbolic()
	
	
	
