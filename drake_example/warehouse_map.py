import numpy as np
import matplotlib.pyplot as plt
import math
import dill
#from DubinsPlantCar import *
#from pydrake.symbolic import Expression
#from pydrake.all import PiecewisePolynomial

from shapely.geometry import Polygon, box
import networkx as nx
from numpy import linalg as LA
import GeometryFunctions as gf

'''
def GetMap():
	#in the future, read a region file
	G1 = box( 23.0, 13.0, 24.0, 14.0 ) #goal 1
	G2 = box(  6.0,  6.0,  7.0,  7.0 ) #goal 2
	
	OBS1 = box( 5.0, 7.0, 25.0, 13.0  ) #obstacle
	
	Free = box( 0.0, 0.0, 30.0, 20.0 ) #free space boundary
    
	coord = list(Free.exterior.coords)
	
	x = np.arange(0., 30., 0.5)
	y = np.arange(0., 20., 0.5)
	xx, yy = np.meshgrid(x, y, sparse=True)
'''
	
ft2m     = 0.3048
W_Height = 233.0 * ft2m # [m]
W_Width  = 434.0 * ft2m # [m]
cell     = 1.25 # [m]

#plant = DubinsCarPlant_[float]() # Default instantiation
#FL_WB = plant.L      # wheel base, for a 36inch length fork, dimension C in spec.
#FL_W  = plant.TotalW # width of car, for all forklifts, dimension G in spec.
#FL_L  = plant.TotalL # length of car, for a 36inch length fork, dimension B in spec.
FL_WB      = 1.882 # wheel base, for a 36inch length fork, dimension C in spec.
FL_W = 0.902 # width of car, for all forklifts, dimension G in spec.
FL_L = 2.619 # length of car, for a 36inch length fork, dimension B in spec.
			
def LoadMP(fName='MPLibrary.lib'):
	dbfile = open(fName, 'rb')
	MotionPrimitives = dill.load(dbfile)
	dbfile.close()
	return MotionPrimitives
	
def PopulateMapWithMP(MotionPrimitives, workspace, obs, cell_h=1.25, cell_w=1.25):
	bounds = np.array(list(workspace.bounds))
	pix2m  = W_Width/(bounds[3]-bounds[1])
	bounds = bounds * pix2m
	
	X = np.arange(bounds[0]+cell, bounds[2]-cell, cell)
	Y = np.arange(bounds[1]+cell, bounds[3]-cell, cell)
	#	XV, YV = np.meshgrid(X, Y)
	nX = len(X)
	nY = len(Y)
	
	#possible_orientations = ('N','E','S','W', 'N', 'E')
	
	total_count = 0
	G = nx.DiGraph()
	for orient in range(4): #corresponds to (E, N, W, S)
		# rotate the motion primitives according to initial position
		if(orient == 0):
			rotmat = np.array([[0.,-1.], [1.,0.]])
		elif(orient == 1):
			rotmat = np.array([[-1.,0.], [0.,-1.]])
		elif(orient == 2):
			rotmat = np.array([[0.,1.], [-1.,0.]])
		else:
			rotmat = np.array([[1.,0.], [0.,1.]])

		for i, x in enumerate(X):
			for j, y in enumerate(Y):
				for key, mp in MotionPrimitives.items():
					#import pdb; pdb.set_trace()
					#if(i==1 and j==1):
					#	import pdb; pdb.set_trace()
					connect2  = np.array([[ mp['e'][0]-mp['s'][0] ], \
								          [ mp['e'][1]-mp['s'][1] ]])
					connect2  = rotmat.dot( connect2 ) # fixed to true orientation
					toLoc     =  np.array([[x], [y]]) + connect2

					if(IsPathFree(workspace, mp, obs, x, y, toLoc[0], toLoc[1], X[0], X[-1], Y[0], Y[-1] )):
						toRot    = (mp['e'][2]-mp['s'][2]) / (90.0*math.pi/180.0)
						toRot    = (orient+toRot)%4
						connect2 = connect2/cell
						G.add_edge( 'H' + str(int(orient)) + 'X' + str(i) + 'Y' + str(j), \
								    'H' + str(int(toRot)) + 'X' + str(i+int(connect2[0][0])) + 'Y' + str(j+int(connect2[1][0])), \
								    weight=LA.norm(connect2), motion=key, index=total_count )
						total_count += 1
	#import pdb; pdb.set_trace()
	return G

# function that decides if an obstacle-free path exist between start and end point
# taking into account the width of the funnels
def IsPathFree(workspace, mp, obstacles, xs, ys, xe, ye, xmin, xmax, ymin, ymax):
	# check if you're not going out of bounds (doesn't account for a U-turn)
	if( (xe < xmin) or (xe > xmax) ):
		return False
	if( (ye < ymin) or (ye > ymax) ):
		return False
	
	for S in mp['V']:
		e = gf.Ellipse(mp['x0'], S)
		for obs in obstacles:
			v = np.array(obs.exterior.coords[:])
			b = gf.Box(v)
			overlaps = gf.TestIntersectionBoxEllipse(b, e)
			if(overlaps == True):
				return False
	
	return True

def ReplicateMap():
	# manually done from the warehouse map we got from Raymond
	rack_w = 12.0
	#shapely.geometry.box(minx, miny, maxx, maxy, ccw=True)
	workspace = box( 27.0, 40.0, 690.0, 1269.0 )
	obs = []
	obs.append( box( 115.0,  46.0, 630.0,  46.0+rack_w*1.0 ) ) # rack, in pixels
	obs.append( box( 115.0,  71.0, 630.0,  71.0+rack_w*1.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 100.0, 610.0, 100.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 140.0, 630.0, 140.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 181.0, 610.0, 181.0+rack_w*1.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 201.0, 630.0, 201.0+rack_w*1.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 229.0, 610.0, 229.0+rack_w*1.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 255.0, 630.0, 255.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 295.0, 610.0, 295.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 333.0, 630.0, 333.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 373.0, 610.0, 373.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 412.0, 610.0, 412.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 467.0, 469.0, 467.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 510.0, 469.0, 510.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 549.0, 469.0, 549.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 588.0, 469.0, 588.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 629.0, 469.0, 629.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 667.0, 469.0, 667.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 706.0, 469.0, 706.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 766.0, 492.0, 766.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 805.0, 515.0, 805.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 844.0, 515.0, 844.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 884.0, 515.0, 884.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 924.0, 515.0, 924.0+rack_w*2.0 ) ) # rack, in pixels
	obs.append( box( 186.0, 963.0, 515.0, 963.0+rack_w*1.0 ) ) # rack, in pixels
	
	obs.append( box(  27.0,   95.0, 145.0, 1070.0 ) ) # don't enter zone, in pixels
	obs.append( box( 455.0, 1113.0, 564.0, 1269.0 ) ) # don't enter zone, in pixels
	obs.append( box( 564.0,  830.0, 690.0, 1032.0 ) ) # don't enter zone, in pixels
	obs.append( box( 537.0,  464.0, 690.0,  723.0 ) ) # don't enter zone, in pixels
	import pdb; pdb.set_trace()
	
	return workspace, obs
	
	def PlotGraph(G):
		pos=nx.spring_layout(G, iterations=10)
		nx.draw(G,pos,edgelist=G.edges(),node_size=50,with_labels=False)
		plt.show()

if __name__ == "__main__":
	MP = LoadMP(fName='MPLibrary.lib')
	workspace, obs = ReplicateMap()
	DiGraph = PopulateMapWithMP(MP, workspace, obs, cell_h=cell, cell_w=cell)
	
	
	
	
