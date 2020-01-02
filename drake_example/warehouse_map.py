import numpy as np
import matplotlib.pyplot as plt
import math
import dill
import re
import glob
import sys
import resource
from sets import Set
import curses, sys, subprocess
import json
from itertools import combinations
from timeit import default_timer as timer
import argparse

import networkx as nx

from numpy import linalg as LA
from shapely.geometry import Polygon, box
#from mayavi import mlab

from StructuredSlugsParser import compiler as slugscomp

from DubinsPlantCar import CELL_SIZE

import GeometryFunctions as gf
import ROSUtilities as RU

ft2m     = 0.3048
W_Height = 0.0 #233.0 * ft2m # [m]
W_Width  = 0.0 #434.0 * ft2m # [m]
cell     = CELL_SIZE #1.25 # [m]
pix2m    = 0.0
W_xgrid  = []
W_ygrid  = []
	
#plant = DubinsCarPlant_[float]() # Default instantiation
#FL_WB = plant.L      # wheel base, for a 36inch length fork, dimension C in spec.
#FL_W  = plant.TotalW # width of car, for all forklifts, dimension G in spec.
#FL_L  = plant.TotalL # length of car, for a 36inch length fork, dimension B in spec.
FL_WB  = 1.882 # wheel base, for a 36inch length fork, dimension C in spec.
FL_W   = 0.902 # width of car, for all forklifts, dimension G in spec.
#FL_L   = 2.619 # length of car, for a 36inch length fork, dimension B in spec.
FL_L   = 0.42 # length of jackal
			
def LoadMP(fName='MPLibrary.lib'):
	dbfile = open(fName, 'rb')
	MotionPrimitives = dill.load(dbfile)
	dbfile.close()
	return MotionPrimitives

def GetRotmat(orient):
	# axes are as follows: X positive = south, Y positive = East
	#  X |  Y -->
	#    V
	# orient corresponds to (S, E, N, W)
	if(orient == 0):
		rotmat = np.array([[1.,0.], [0.,1.]])
	elif(orient == 1):
		rotmat = np.array([[0.,-1.], [1.,0.]])
	elif(orient == 2):
		rotmat = np.array([[-1.,0.], [0.,-1.]])
	elif((orient == 3) or (orient == -1)):
		rotmat = np.array([[0.,1.], [-1.,0.]])
	else:
		print('bad orientation parameter')
			
	return rotmat

# add connections between grid points using motion primitives if they do not collide with any obstacle
def PopulateMapWithMP(MotionPrimitives, workspace, obs, no_enter, map_kind, cell_h=1.25, cell_w=1.25, force=False):
	global W_Width
	global W_xgrid
	global W_ygrid
	global pix2m
	
	tic = timer()

	bounds = np.array(list(workspace.bounds))
	
	#pix2m  = W_Width/(bounds[3]-bounds[1])
	bounds = bounds * pix2m
	
	# plot obstacles & total workable space
	ax = plot_map(workspace, obs)
		
	X = np.arange(bounds[0]+cell/2.0, bounds[2]-cell/2.0, cell)
	W_xgrid = X.copy()
	Y = np.arange(bounds[1]+cell/2.0, bounds[3]-cell/2.0, cell)
	W_ygrid = Y.copy()
	#	XV, YV = np.meshgrid(X, Y)
	nX = len(X)
	nY = len(Y)
	
	obstacles = []
	for obstacle in obs:
		v = pix2m * np.array(obstacle.exterior.bounds)
		obstacles.append(v)
	# create the world map for gazebo simulation
	RU.CreateCustomMapWorld(map_kind, bounds, obstacles)
	#import 	pdb; pdb.set_trace()
	
	if((force == False) and (glob.glob(map_kind + '.pickle'))):
		# if it already exist, save time re-creating the graph
		G = LoadGraphFromFile(map_kind)
		toc = timer()
		print('Motion primitives file exist, so just loading existing (%f[sec])' %(toc-tic))
		#import pdb; pdb.set_trace()
		return G, ax

	# for now, we treat do not enter zones as obstacles so it would not have 
	# the funnels on the map. In the future, this needs to go, because we loose
	# the options to do reactive stuff with the do not enter zone
	merged_obs_list = []
	merged_obs_list.extend(obs)
	#merged_obs_list.extend(no_enter)
	
	total_count = 0
	G = nx.DiGraph(name='ConnectivityGraph')
	for orient in range(4): #corresponds to (E, N, W, S)
		# rotate the motion primitives according to initial position
		rotmat = GetRotmat(orient)

		#import pdb; pdb.set_trace()
		for i, x in enumerate(X):
			for j, y in enumerate(Y):
				for key, mp in MotionPrimitives.items():
					#import pdb; pdb.set_trace()
					connect2  = np.array([[ mp['e'][0]-mp['s'][0] ], \
								          [ mp['e'][1]-mp['s'][1] ]])
					# rotate to global coordinate system
					connect2  = rotmat.dot( connect2 ) 
					toLoc     = np.array([[x], [y]]) + connect2

					#if(int(orient) == 1 and i==12 and j==6):
					#	import pdb; pdb.set_trace()

					# check if funnel is in bounds and does not collide with any obstacle
					if(IsPathFree(mp, merged_obs_list, rotmat, orient, x, y, toLoc[0], toLoc[1], \
								  X[0], X[-1], Y[0], Y[-1], ax )):
						toRot    = (mp['e'][2]-mp['s'][2]) / (90.0*math.pi/180.0)
						toRot    = (orient+toRot)%4
						connect2 = connect2/cell
						# add some weighting based on the fact that you should prefer longer
						# primitives (motion 0 will be better than 4 times motion 2) and
						# turning around would be more expensive than going straight (motion 7-8 vs. motion 0).
						# when constructing the library, the key is ranked with ascending difficulty
						difficulty_factor = 1.0  + key/20.0 #just a factor
						if(key>6):
							difficulty_factor = difficulty_factor*5
													
						G.add_edge( 'H' + str(int(orient)) + 'X' + str(i) + 'Y' + str(j), \
								    'H' + str(int(toRot)) + 'X' + str(i+int(connect2[0][0])) + 'Y' + str(j+int(connect2[1][0])), \
								    weight=LA.norm(connect2)*difficulty_factor, motion=key, index=total_count )
						total_count += 1
		plt.pause(0.05)
		print('Done computing transition map for orientation (%d/4).' %(orient+1))
	
	print(nx.info(G))
	# save time for next time
	SaveGraphToFile(G, map_kind)
	#import pdb; pdb.set_trace()
	toc = timer()
	print('Motion primitives on map took %f[sec]' %(toc-tic))
	return G, ax

# function that decides if an obstacle-free path exist between start and end point
# taking into account the width of the funnels
def IsPathFree(mp, obstacles, rotmat, orient, xs, ys, xe, ye, xmin, xmax, ymin, ymax, ax):
	global pix2m
	# check if you're not going out of bounds (it's fast, but doesn't account for a U-turn!)
	if( (xe < xmin) or (xe > xmax) ):
		return False
	if( (ye < ymin) or (ye > ymax) ):
		return False
	
	if(orient == 3):
		orient = -1 #just so it would look nicer on the 3d plots 270 -> -90
		
	for i, S in enumerate(mp['V']):
		#import pdb; pdb.set_trace()
		# in motion primitive's relative coordinates
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
	if(False): #np.random.rand() > 0.9): #True): 
		#import pdb; pdb.set_trace()
		for i, S in enumerate(mp['V']):
			x_rel = mp['xcenter'][i]
			theta = x_rel[2]
			x_rel = rotmat.dot(x_rel[0:2]) # just x,y
			e_center = np.array([xs, ys]) + x_rel
			e_center = np.hstack((e_center, theta + orient*90.0*math.pi/180.0))
			# create the ellipsoid object
			e = gf.Ellipse(e_center, S)
			plot_ellipsoid(ax, e, orient)
			
			# plot the 2-D ellipse 
			#plot_ellipse(ax, S, e_center, orient)
			
	return True

# this will be loading coordinates of a map from a file in the future
def ReplicateMap(map_kind = 'none'):
	global W_Height
	global W_Width
	global ft2m
	global pix2m
	
	tic = timer()
		
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
		
		goals = np.array([[670., 90., 90.0*math.pi/180.0], [200., 990., 0.0*math.pi/180.0]])
		
		W_Height = 233.0 * ft2m # [m]
		W_Width  = 434.0 * ft2m # [m]
		
		no_enter = []
		'''
		elif(map_kind.lower() == 'lab'):
			workspace = box( -8*ft2m, -12.0*ft2m, 9.0*ft2m, 11.0*ft2m ) 

			W_Height = workspace.bounds[2] - workspace.bounds[0]  # [m]
			W_Width  = workspace.bounds[3] - workspace.bounds[1] # [m]

			obs = []
			obs.append( box( 6.*ft2m, -12.0*ft2m, 9.0*ft2m,  -4.0*ft2m ) ) # rack, in pixels
			obs.append( box( 0.*ft2m, 0.*ft2m, 7.5*ft2m,  7.0*ft2m ) ) # table, in pixels

			goals = np.array([[ 2.5*ft2m, -7.0*ft2m,  90.0*math.pi/180.0], \
							  [ 5.0*ft2m,  8.5*ft2m,  0.0*math.pi/180.0], \
							  [-7.0*ft2m, -10.*ft2m, -90.0*math.pi/180.0] ]) 

			no_enter = []
			no_enter.append(box( -4.0*ft2m,  0.0*ft2m, 0.0*ft2m,  7.0*ft2m ) )
			no_enter.append(box( -4.0*ft2m, -12.*ft2m, 0.0*ft2m, -5.0*ft2m ) )
		'''
	elif(map_kind.lower() == 'lab'):
		workspace = box( -8*ft2m, -12.0*ft2m, 9.0*ft2m, 11.0*ft2m ) 

		W_Height = workspace.bounds[2] - workspace.bounds[0]  # [m]
		W_Width  = workspace.bounds[3] - workspace.bounds[1] # [m]

		obs = []
		obs.append( box( 6.*ft2m, -12.0*ft2m, 9.0*ft2m,  -4.0*ft2m ) ) # rack, in pixels
		obs.append( box( 0.*ft2m, 0.*ft2m, 7.5*ft2m,  7.0*ft2m ) ) # table, in pixels
		
		goals = np.array([[ 2.5*ft2m, -7.0*ft2m,  90.0*math.pi/180.0], \
						  [ 5.0*ft2m,  8.5*ft2m,  0.0*math.pi/180.0], \
						  [-7.0*ft2m, -10.*ft2m, -90.0*math.pi/180.0] ]) 
		
		no_enter = []
		no_enter.append(box( -2.0*ft2m,  0.0*ft2m, 0.0*ft2m,  7.0*ft2m ) )
		no_enter.append(box( -2.0*ft2m, -12.*ft2m, 0.0*ft2m, -5.0*ft2m ) )
			
	elif(map_kind.lower() == 'map1'):
		workspace = box( 0.0, 0.0, 100.0, 200.0 )
		obs = []
		obs.append( box( 15., 50.0, 25.0,  150.0 ) ) # rack, in pixels
		obs.append( box( 45., 50.0, 55.0,  150.0 ) ) # rack, in pixels
		obs.append( box( 75., 50.0, 85.0,  130.0 ) ) # rack, in pixels
		
		goals = np.array([[63., 197., -90.0*math.pi/180.0], [877., 200., 90.0*math.pi/180.0]])
		
		W_Height = 30.0 * ft2m # [m]
		W_Width  = 60.0 * ft2m # [m]
		
		no_enter = []
		
	elif(map_kind.lower() == 'none'):
		workspace = box( 0.0, 0.0, 100.0, 200.0 )
		obs = []
		obs.append( box( 30.0, 60.0, 50.0,  140.0 ) ) # rack, in pixels
		
		goals = np.array([[70., 70., 90.0*math.pi/180.0], \
						  [10., 120., -90.0*math.pi/180.0]])
		
		W_Height = 20.0 # [m]
		W_Width  = 40.0 # [m]
		
		no_enter = []
	
	pix2m  = W_Width/(workspace.bounds[3]-workspace.bounds[1])
	#import pdb; pdb.set_trace()
	MapToFile(map_kind.lower(), workspace, obs, goals, no_enter, pix2m)
	
	toc = timer()
	print('Loading map took %f[sec]' %(toc-tic))
	
	return workspace, obs, goals, no_enter

def MapToFile(map_kind, workspace, obstacles, goals, no_enter, scale):
	data = {}
	data['workspace'] = []
	data['obstacles'] = []
	data['goals'] = []
	data['no_enter'] = []
	
	bounds = list(workspace.bounds)
	data['workspace'].append({
		'x': bounds[0]*pix2m,
		'y': bounds[1]*pix2m,
		'X': bounds[2]*pix2m,
		'Y': bounds[3]*pix2m
	})
		
	for i,obs in enumerate(obstacles):
		data['obstacles'].append({
			'x': obs.bounds[0]*pix2m,
			'y': obs.bounds[1]*pix2m,
			'X': obs.bounds[2]*pix2m,
			'Y': obs.bounds[3]*pix2m
		})
		
	for g in goals:
		data['goals'].append({
			'x': g[0]*pix2m,
			'y': g[1]*pix2m,
			'teta': g[2]
		})
	
	for ne in no_enter:
		bounds = list(ne.bounds)
		data['no_enter'].append({
			'x': bounds[0]*pix2m,
			'y': bounds[1]*pix2m,
			'X': bounds[2]*pix2m,
			'Y': bounds[3]*pix2m
		})
	data['cell'] = CELL_SIZE
	
	with open(map_kind + '.map', 'w') as outfile:
		json.dump(data, outfile)

def MotionPrimitivesToFile(map_kind, motion_primitives):
	data = {}
	data['mp'] = {}

	for key, mp in motion_primitives.items():
		data['mp']['x' + str(key)] = []
		for i, x in enumerate(mp['xcenter']):
			data['mp']['x' + str(key)].append({
				'x': mp['xcenter'][i].tolist(),
				'V': mp['V'][i].flatten().tolist() })
		
	with open(map_kind + '.motion', 'w') as outfile:
		json.dump(data, outfile)
	

# finds a path between each goal using graph search just because it's so much
# faster than slugs so we can immediatley alert the user that the path is infeasible
def FindPathBetweenGoals(G, goals):
	global pix2m
	global cell
	
	tic = timer()
		
	N, __ = goals.shape
	paths = []

	for i in range(N):
		start = goals[i, :]
		if(i == N-1):
			finish = goals[0, :] # complete the cycle
		else:
			finish = goals[i+1, :]
		start_label  = GetNodeLabel(start)
		finish_label = GetNodeLabel(finish)
		
		try:
			paths.append( nx.dijkstra_path(G, source=start_label, target=finish_label, weight='weight') ) #shortest_path
			print('Found path between %s to %s' %(start_label, finish_label))
		except:
			print('Could not find any path between %s to %s' %(start_label, finish_label))
			paths.append([])

	toc = timer()
	print('Find shortest path (Dijkstra Graph search) took %f[sec]' %(toc-tic))
	return paths

# conversion from arbitrary location on map to the closest funnel (location & orientation)
def GetNodeLabel(pose):
	global W_xgrid
	global W_ygrid

	orient = int(np.round( pose[2] / (math.pi/2.0) ) % 4)  # what quadrant you're closest to
	label = 'H' + str(orient) + 'X' + str(find_nearest(W_xgrid, pix2m*pose[0])) + \
								'Y' + str(find_nearest(W_ygrid, pix2m*pose[1]))
	
	return label

# utility function to find the closest grid point
def find_nearest(arr, value):
	arr = np.asarray(arr)
	idx = (np.abs(arr - value)).argmin()
	return idx

# visualize the graph edges
def PlotGraph(G):
	pos = nx.spring_layout(G, iterations=10)
	nx.draw(G,pos,edgelist=G.edges(),node_size=50,with_labels=False)
	plt.show()

# plots the workspace and obstacles in 3D
def plot_map(workspace, obstacles):
	global pix2m
	# plot map, obstacles, ...
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	v = pix2m * np.array(workspace.exterior.coords[:])
	wksp = gf.Box(v) 
	#plot workspace
	for s, e in combinations(wksp.vertices, 2):
		if(np.sum(s==e) == 2): # works only when parallel!!
			ax.plot3D(*zip(s, e), color="k")

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
	ax.set_title(r'Workspace, X-Y-$\theta$, 3D view')
	ax.set_aspect("equal")
	plt.show(block = False)
	plt.pause(0.05)

	return ax

# plots a single ellipsoid in 3D
def plot_ellipsoid(ax, ellipse, orient, color=None): 
	A = ellipse.M
	center = ellipse.center

	# find the rotation matrix and radii of the axes
	U, s, rotation = LA.svd(A)
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

	if(color == None):
		if(orient == 0):
			clr = (0.1,0.5,0.8)
		elif(orient == 1):
			clr = (0.3,0.5,0.8)
		elif(orient == 2):
			clr = (0.5,0.5,0.8)
		elif(orient == -1 or orient == 3):
			clr = (0.7,0.5,0.8)
	else:
		clr = color
	# plot ellipse
	ax.plot_wireframe(x, y, z,  rcount=6, ccount=6, color=clr, alpha=0.2) #rstride=4, cstride=4
	#mlab.mesh(x, y, z) 

# plots the entire sequence leading between goal points
def plot_path(ax, G, paths, MotionPrimitives):
	global W_xgrid
	global W_ygrid

	tic = timer()
	
	for j, path in enumerate(paths):
		if(not paths):
			# no path exists between these two points
			continue
		for k in range(len(path)-1):
			from_str = path[k]
			to_str = path[k+1]
			node = G[from_str]  # the parent
			mp = MotionPrimitives[ node[to_str]['motion'] ] # the child
			orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', from_str)] # extract current pose
			# transform to real world coordinates
			xs = W_xgrid[xs] 
			ys = W_ygrid[ys]
			
			'''
			if(orient == 0):
				rotmat = np.array([[0.,-1.], [1.,0.]])
			elif(orient == 1):
				rotmat = np.array([[-1.,0.], [0.,-1.]])
			elif(orient == 2):
				rotmat = np.array([[0.,1.], [-1.,0.]])
			else:
				rotmat = np.array([[1.,0.], [0.,1.]])
			'''
			rotmat = GetRotmat(orient)
			# just so it would look nice in plots
			if(orient == 3):
				orient = -1
			
			for i, S in enumerate(mp['V']):
				x_rel = mp['xcenter'][i]
				theta = x_rel[2]
				x_rel = rotmat.dot(x_rel[0:2]) # just x,y
				e_center = np.array([xs, ys]) + x_rel
				e_center = np.hstack((e_center, theta + orient*90.0*math.pi/180.0))
				# create the ellipsoid object
				# do I need to rotate the ellipse as well??? CHECK THIS!
				e = gf.Ellipse(e_center, S)
				plot_ellipsoid(ax, e, orient, color=(float(j)/float(len(paths)),0.6,0.6))
	
	toc = timer()
	print('Plotting the shortest path took %f[sec]' %(toc-tic))

# generates the specification file for slugs
def CreateSlugsInputFile(G, goals, MP, no_enter, filename='map_funnel'):
	global W_xgrid
	global W_ygrid
	
	tic = timer()
	
	Nnodes = nx.number_of_nodes(G)
	Nedges = nx.number_of_edges(G)
	
	start_label = []
	finish_label = []
	
	print('Creating the structured slugs file ...')
	
	N, __ = goals.shape
	#print('Start at: %s' %(GetNodeLabel(goals[0, :])))
	
	for i in range(N):
		start = goals[i, :]
		if(i == N-1):
			finish = goals[0, :] # complete the cycle
		else:
			finish = goals[i+1, :]
		start_label.append(GetNodeLabel(start))
		finish_label.append(GetNodeLabel(finish))
		#print('Then go to: %s' %(finish_label[-1]))
	
	map_label_2_bit = {}
	node_count = 0
	for node in G:
		map_label_2_bit.update({node: node_count})
		node_count += 1
		## this was an attempt to not include nodes that don't have children (dead-ends)
		## but it needs to be handled with care. doesn't work at the moment
		#children = G.neighbors(node) #list of successor nodes of parent
		#if(children.__length_hint__()>0):
		#	map_label_2_bit.update({node: node_count})
		#	node_count = node_count + 1
		#else:
		#	#this node is a dead-end so we might as well just not include it
		#	pass
		
	with open(filename + '.structuredslugs', 'w') as f: 	
		# structured slugs
		f.write('[INPUT]\n') 
		f.write('R:0...%d\n' %( node_count ) ) #funnels/regions
		f.write('\n')
		
		f.write('[OUTPUT]\n')
		f.write('mp:0...%d\n' %( len(MP) ))
		f.write('\n')
				
		f.write('[ENV_LIVENESS]\n')
		f.write('\n')
		
		f.write('[SYS_LIVENESS]\n')
		# all the points we want to reach (goals)
		for g in start_label:
			f.write('R=%d\n' %( map_label_2_bit[ g ] ))
		f.write('\n')
		
		f.write('[SYS_INIT]\n') 
		f.write('\n')
		
		f.write('[ENV_INIT]\n')
		f.write('R=%d\n' %( map_label_2_bit[ start_label[0] ] )) # need to change this, either to have
		# no specific start point, or, start at the current location, or have some mechanism to get from current
		# location to start position
		f.write('\n')
		
		f.write('[SYS_TRANS]\n')
		f.write('\n')
		
		f.write('[ENV_TRANS]\n')
		all_mps = Set(np.arange(0, len(MP)))
		
		# first, add all theoretically available motions to the environment transitions
		for parent in G:
			children = G.neighbors(parent) #list of successor nodes of parent
			avail_links = Set([])
			for child in children:
				#grandchild = G.neighbors(child) 
				#if(grandchild.__length_hint__()>0):
					# if it has no grandchild then it was probably removed earlier
					# and it is not even available in the dictionary
				mp_num = G[parent][child]['motion']
				avail_links.update([mp_num])
				
				# all the points we wish to avoid (do not enter zones)
				orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', parent)] # extract ellipse pose
				#if(map_label_2_bit[parent] == 1290): #'H1X12Y6'
				#	import pdb; pdb.set_trace()
				out_of_no_enter = IsPathFree(MP[mp_num], no_enter, GetRotmat(orient), orient, W_xgrid[xs], W_ygrid[ys], 0, 0, \
								  -10e6, 10e6, -10e6, 10e6, 0 ) #the last 7 parameters don't really matter here
				if(out_of_no_enter):
					# good, does not intersect the no entry zones
					f.write('(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
														   mp_num, \
														   map_label_2_bit[child]))
				else:
					# intersects a no entry zone. can add reactivity right here.
					# for now, it's a global avoid this region (as if we would've treated it as an obstacle)
					# the # is just so I could recognize where it happens in the file
					f.write('(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
														   mp_num, \
														   map_label_2_bit[parent]))
			
			links_not_provided = all_mps - avail_links
			while(len(links_not_provided)>0):
				f.write('(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
												   links_not_provided.pop(), \
												   map_label_2_bit[parent]))
		f.write('\n')
						
		
	print('done.')
	print('Converting to slugsin ...')
	try:
		resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
	except ValueError:
		pass # Cannot increase limit
	sys.setrecursionlimit(10**6)
	sys.stdout = open(filename + '.slugsin', "w") # the compiler function is just printing on screen
	slugscomp.performConversion(filename + '.structuredslugs', False)
	sys.stdout = sys.__stdout__  # convert back to printing on screen
	print('done.')
	
	dbfile = open(filename + '.label2bit', 'wb')
	dill.dump(map_label_2_bit, dbfile)
	dbfile.close()
	
	toc = timer()
	print('Creating structuredslugs & converting to slugsin file took %f[sec]' %(toc-tic))
	
	return map_label_2_bit

# get a complete synthesized controller for our spec.
def SynthesizeController(filename='map_funnel'):
	tic = timer()
	
	slugsLink = '/home/cornell/Tools/slugs/src/slugs'
	specFile  = filename + '.slugsin'
	
	#--interactiveStrategy
	slugsProcess = subprocess.Popen(slugsLink + ' --explicitStrategy --jsonOutput ' + specFile, \
									shell=True, bufsize=1048000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

	#slugsProcess.stdin.write("XPRINTINPUTS\n")
	slugsProcess.stdin.flush()
	slugsProcess.stdout.readline() # Skip the prompt
	lastLine = " "
	inputAPs = '{ '
	while (lastLine!="}}"):
		lastLine = slugsProcess.stdout.readline().strip()
		if lastLine!="":
			inputAPs = inputAPs + ' ' + lastLine
	
	#import pdb; pdb.set_trace()
	controller = json.loads(inputAPs)
	with open(filename + '.json', "w") as write_file:
		json.dump(controller, write_file)
	
	toc = timer()
	print('Synthesizing control via slugs took %f[sec]' %(toc-tic))
	
	return controller

# decipher the sequence of moves to satisfy spec.
def GetSpecificControl(C, map_bit_2_label, debug=True):
	tic = timer()
	
	trans = 0
	actions_count = 0
	var = C['variables']
	states = []
	mps = []
	
	for v in reversed(var):
		if 'mp' in v:
			actions_count += 1
		else:
			break;
	state_count = len(var) - actions_count
	
	cntr = 1
	break_on_next_loop = 0
	#import pdb; pdb.set_trace()
	while True:
		in_state = C['nodes'][str(trans)]
		state = in_state['state']

		# it comes in reverse binary
		cur_state_str = state[::-1][actions_count:]
		cur_state = 0
		for b in cur_state_str:
			cur_state = 2 * cur_state + b
		
		trans = in_state['trans'][0]
		n_state = C['nodes'][str(trans)]
		next_state_str = n_state['state'][::-1][actions_count:]
		next_state = 0
		for b in next_state_str:
			next_state = 2 * next_state + b
		
		if(next_state == cur_state):
			# for some reason slugs sometimes gives wrong actions in the beginning
			# this is a way to overcome it. In the future, need to undestand why this happens
			# and solve it in the slugs level (or specification).
			continue
			
		# it comes in reverse binary
		action_str = state[::-1][0:actions_count]
		action = 0
		for b in action_str:
			action = 2 * action + b
		mps.append(action)
		
		states.append(map_bit_2_label[cur_state])
		if(debug):
			print('%d) In state: %s, take motion primitive: %d' %(cntr, states[-1], action))
		
		
		if(break_on_next_loop):
			break;
			
		if (trans == 0):
			# we've looped
			break_on_next_loop = 1
		
		cntr += 1

	if(debug):
		print('Go back to step 1)')
	
	toc = timer()
	if(debug):
		print('Extracting plan (from slugs) took %f[sec]' %(toc-tic))
	
	#import pdb; pdb.set_trace()
	jsonList = []
	for i in range(len(states)):
		jsonList.append({"state" : states[i], "action" : mps[i]})
	with open('lab.states', 'wt') as f:
		json.dump(jsonList, f)
		
	return states, mps


def SaveGraphToFile(G, filename):
	nx.write_gpickle(G, filename + '.pickle')

def LoadGraphFromFile(filename):
	return nx.read_gpickle(filename + '.pickle')

# not currently in use
def plot_ellipse(ax, A, x0, orient):
	vertices = 51
	
	# simple projection to 2D assuming we want to plot on (x1,x2)
	A = A[0:2,0:2]
	b = np.array([[0], [0]])  # check this!!!! GUY
	c = 0.0
	#Plots the 2D ellipse representing x'Ax + b'x + c <= 1, e.g.
	#the one sub-level set of a quadratic form.
	H = .5*(A+A.T)
	xmin = LA.solve(-2*H, np.reshape(b, (2, 1)))
	fmin = -xmin.T.dot(H).dot(xmin) + c  # since b = -2*H*xmin
	assert fmin <= 1, "The minimum value is > 1; there is no sub-level set " \
			  "to plot"

	# To plot the contour at f = (x-xmin)'H(x-xmin) + fmin = 1,
	# we make a circle of values y, such that: y'y = 1-fmin,
	th = np.linspace(0, 2*np.pi, vertices)
	Y = np.sqrt(1-fmin)*np.vstack([np.sin(th), np.cos(th)])
	# then choose L'*(x - xmin) = y, where H = LL'.
	L = np.linalg.cholesky(H)
	X = np.tile(xmin, vertices) + LA.inv(np.transpose(L)).dot(Y)

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
	parser = argparse.ArgumentParser()
	parser.add_argument("-f","--force", help="(y/[n]) force re-creating the funnels on top of the map")
	args = parser.parse_args()
	force = args.force
	if(force == None):
		force = False
	else:
		if('y' in force.lower()):
			force = True
		else:
			force = False
	
	#map_kind = 'none' 
	map_kind = 'lab' 
	#map_kind = 'raymond'
	MP = LoadMP(fName='MPLibrary.lib')
	MotionPrimitivesToFile(map_kind, MP)
	#import pdb; pdb.set_trace()
	workspace, obs, goals, no_enter = ReplicateMap(map_kind=map_kind) #
	
	DiGraph, ax = PopulateMapWithMP(MP, workspace, obs, no_enter, map_kind, cell_h=cell, cell_w=cell, force=force)
	path = FindPathBetweenGoals(DiGraph, goals)
	# create the amount of jackals you want
	RU.CreateJackals(map_kind, [[pix2m*goals[0][0], pix2m*goals[0][1], goals[0][2]]])
	#import pdb; pdb.set_trace()
	
	plot_path(ax, DiGraph, path, MP)
	#import pdb; pdb.set_trace()
	map_label_2_bit = CreateSlugsInputFile(DiGraph, goals, MP, no_enter, filename=map_kind) #'map_funnel')
	# the reverse dictionary is useful
	map_bit_2_label = dict((v, k) for k, v in map_label_2_bit.items())
	
	synctrl = SynthesizeController(filename=map_kind) #'map_funnel')
	GetSpecificControl(synctrl, map_bit_2_label)
	print('Done. Close figure to exit.')
	plt.show(block=True)
	

							 	