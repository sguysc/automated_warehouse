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
from itertools import combinations


ft2m     = 0.3048
W_Height = 0.0 #233.0 * ft2m # [m]
W_Width  = 0.0 #434.0 * ft2m # [m]
cell     = 1.25 # [m]

#plant = DubinsCarPlant_[float]() # Default instantiation
#FL_WB = plant.L      # wheel base, for a 36inch length fork, dimension C in spec.
#FL_W  = plant.TotalW # width of car, for all forklifts, dimension G in spec.
#FL_L  = plant.TotalL # length of car, for a 36inch length fork, dimension B in spec.
FL_WB  = 1.882 # wheel base, for a 36inch length fork, dimension C in spec.
FL_W   = 0.902 # width of car, for all forklifts, dimension G in spec.
FL_L   = 2.619 # length of car, for a 36inch length fork, dimension B in spec.
			
def LoadMP(fName='MPLibrary.lib'):
	dbfile = open(fName, 'rb')
	MotionPrimitives = dill.load(dbfile)
	dbfile.close()
	return MotionPrimitives
	
def PopulateMapWithMP(MotionPrimitives, workspace, obs, cell_h=1.25, cell_w=1.25):
	global W_Width
	
	bounds = np.array(list(workspace.bounds))
	pix2m  = W_Width/(bounds[3]-bounds[1])
	bounds = bounds * pix2m
	
	# plot obstacles & total workable space
	ax = plot_map(workspace, obs, pix2m)
		
	X = np.arange(bounds[0]+cell, bounds[2]-cell, cell)
	Y = np.arange(bounds[1]+cell, bounds[3]-cell, cell)
	#	XV, YV = np.meshgrid(X, Y)
	nX = len(X)
	nY = len(Y)
	
	#possible_orientations = {0: 0.*math.pi/180.0, 1: 90.*math.pi/180.0, 2: 180.*math.pi/180.0, 3: 270.*math.pi/180.0}
	
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

		#import pdb; pdb.set_trace()
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
					# check if funnel is in bounds and does not collide with any obstacle
					if(IsPathFree(workspace, mp, obs, rotmat, orient, x, y, toLoc[0], toLoc[1], \
								  X[0], X[-1], Y[0], Y[-1], pix2m, ax )):
						toRot    = (mp['e'][2]-mp['s'][2]) / (90.0*math.pi/180.0)
						toRot    = (orient+toRot)%4
						connect2 = connect2/cell
						G.add_edge( 'H' + str(int(orient)) + 'X' + str(i) + 'Y' + str(j), \
								    'H' + str(int(toRot)) + 'X' + str(i+int(connect2[0][0])) + 'Y' + str(j+int(connect2[1][0])), \
								    weight=LA.norm(connect2), motion=key, index=total_count )
						total_count += 1
		plt.pause(0.05)
		print('Done computing transition map for orientation (%d/4).' %(orient+1))
	import pdb; pdb.set_trace()
	return G

# function that decides if an obstacle-free path exist between start and end point
# taking into account the width of the funnels
def IsPathFree(workspace, mp, obstacles, rotmat, orient, xs, ys, xe, ye, xmin, xmax, ymin, ymax, pix2m, ax):
	# check if you're not going out of bounds (doesn't account for a U-turn!)
	if( (xe < xmin) or (xe > xmax) ):
		return False
	if( (ye < ymin) or (ye > ymax) ):
		return False
	
	if(orient == 3):
		orient = -1
		
	for i, S in enumerate(mp['V']):
		#import pdb; pdb.set_trace()
		# in motion primitive's normalized coordinates
		x_rel = mp['xcenter'][i]
		# move it to account for current orientation
		theta = x_rel[2]
		x_rel = rotmat.dot(x_rel[0:2]) # just x,y
		# calculate each ellipsoid's center in the funnel
		e_center = np.array([xs, ys]) + x_rel
		e_center = np.hstack((e_center, theta + orient*90.0*math.pi/180.0))
		# create the ellipsoid object
		e = gf.Ellipse(e_center, S)
		# iterate through all obstacles to see if any one of them
		# touches any of the ellipsoids (funnel). This does not take
		# into account the funnel in between any two ellipsoids
		for obs in obstacles:
			v = pix2m * np.array(obs.exterior.coords[:])
			b = gf.Box(v)
			overlaps = gf.TestIntersectionBoxEllipse(b, e)
			if(overlaps == True):
				return False
		
	# if we made it thus far, the funnel is ok
	if(np.random.rand() > 0.9): #True): 
		#import pdb; pdb.set_trace()
		for i, S in enumerate(mp['V']):
			x_rel = mp['xcenter'][i]
			theta = x_rel[2]
			x_rel = rotmat.dot(x_rel[0:2]) # just x,y
			e_center = np.array([xs, ys]) + x_rel
			e_center = np.hstack((e_center, theta + orient*90.0*math.pi/180.0))
			'''
			# create the ellipsoid object
			e = gf.Ellipse(e_center, S)
			plot_ellipsoid(ax, e, orient)
			'''
			# plot the 2-D ellipse 
			plot_ellipse(ax, S, e_center, orient)
			
	return True

def ReplicateMap(map_kind = 'none'):
	global W_Height
	global W_Width
	global ft2m
	
	if(map_kind.lower() == 'raymond'):
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
		
		W_Height = 233.0 * ft2m # [m]
		W_Width  = 434.0 * ft2m # [m]

	elif(map_kind.lower() == 'map1'):
		workspace = box( 0.0, 0.0, 100.0, 200.0 )
		obs = []
		obs.append( box( 15., 50.0, 25.0,  150.0 ) ) # rack, in pixels
		obs.append( box( 45., 50.0, 55.0,  150.0 ) ) # rack, in pixels
		obs.append( box( 75., 50.0, 85.0,  130.0 ) ) # rack, in pixels
		
		W_Height = 30.0 # [m]
		W_Width  = 60.0 # [m]
	
	elif(map_kind.lower() == 'none'):
		workspace = box( 0.0, 0.0, 100.0, 200.0 )
		obs = []
		obs.append( box( 30.0, 60.0, 50.0,  140.0 ) ) # rack, in pixels
		
		W_Height = 10.0 # [m]
		W_Width  = 20.0 # [m]
		
	#import pdb; pdb.set_trace()
	
	return workspace, obs
	
def PlotGraph(G):
	pos=nx.spring_layout(G, iterations=10)
	nx.draw(G,pos,edgelist=G.edges(),node_size=50,with_labels=False)
	plt.show()

def plot_map(workspace, obstacles, pix2m): 
	# plot map, obstacles, ...
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	v = pix2m * np.array(workspace.exterior.coords[:])
	wksp = gf.Box(v) 
	#plot workspace
	for s, e in combinations(wksp.vertices, 2):
		if(np.sum(s==e) == 2): # works only when parallel!!
			ax.plot3D(*zip(s, e), color="b")

	for obs in obstacles:
		v = pix2m * np.array(obs.exterior.coords[:])
		box = gf.Box(v) 
		#plot box
		for s, e in combinations(box.vertices, 2):
			if(np.sum(s==e) == 2): # works only when parallel!!
				ax.plot3D(*zip(s, e), color="b")

	ax.set_xlabel('X [m]')
	ax.set_ylabel('Y [m]')
	ax.set_zlabel('$\Theta$ [Rad]')

	ax.set_aspect("equal")
	plt.show(block = False)
	plt.pause(0.05)

	return ax

def plot_ellipsoid(ax, ellipse, orient): 
	A = ellipse.M
	center = ellipse.center

	# find the rotation matrix and radii of the axes
	U, s, rotation = np.linalg.svd(A)
	radii = 1.0/np.sqrt(s)

	# sometimes the optimization gives a really small number
	# in the S[2,2] coefficient
	if(np.abs(radii[2]) > math.pi):
		radii[2] = 45.0*math.pi/180.0
		
	# create ellipse in spherical coordinates
	u = np.linspace(0.0, 2.0 * np.pi, 100)
	v = np.linspace(0.0, np.pi, 100)
	x = radii[0] * np.outer(np.cos(u), np.sin(v))
	y = radii[1] * np.outer(np.sin(u), np.sin(v))
	z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
	for i in range(len(x)):
		for j in range(len(x)):
			[x[i,j],y[i,j],z[i,j]] = np.dot([x[i,j],y[i,j],z[i,j]], rotation) + center

	if(orient == 0):
		clr = (0.1,0.5,0.8)
	elif(orient == 1):
		clr = (0.3,0.5,0.8)
	elif(orient == 2):
		clr = (0.5,0.5,0.8)
	elif(orient == -1 or orient == 3):
		clr = (0.7,0.5,0.8)
	# plot ellipse
	ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color=clr, alpha=0.2)

	#plt.show() #block = False)
	#plt.pause(0.05)	

def plot_ellipse(ax, A, x0, orient):
	# simple projection to 2D assuming we want to plot on (x1,x2)
	A = A[0:2,0:2]
	b = b[0:2]*0.0  # check this!!!! GUY
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

	if(orient == 0):
		clr = (0.1,0.5,0.8)
	elif(orient == 1):
		clr = (0.3,0.5,0.8)
	elif(orient == 2):
		clr = (0.5,0.5,0.8)
	elif(orient == -1 or orient == 3):
		clr = (0.7,0.5,0.8)

	ax.fill(X[0, :]+x0[0], X[1, :]+x0[1], color=clr)

if __name__ == "__main__":
	MP = LoadMP(fName='MPLibrary.lib')
	workspace, obs = ReplicateMap(map_kind='none')
	
	DiGraph = PopulateMapWithMP(MP, workspace, obs, cell_h=cell, cell_w=cell)
	
	
	
	
