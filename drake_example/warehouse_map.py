#!/usr/bin/env python

#from __future__ import print_function

import numpy as np
import matplotlib.pyplot as plt
import math
import dill
import re
import glob
import sys
import resource
from sets import Set
import curses, sys, subprocess, os
import json
from itertools import combinations
from timeit import default_timer as timer
import argparse
from tqdm import tqdm

import networkx as nx

from numpy import linalg as LA
from shapely.geometry import Polygon, box, Point
#from mayavi import mlab

from StructuredSlugsParser import compiler as slugscomp

#MAP_KIND = 'raymond'
MAP_KIND = 'lab' 
# I commented this because it causes loading DubinsPlantCar which tries to load pydrake which is unavailable in the lab computer
#from DubinsPlantCar import CELL_SIZE
#CELL_SIZE = 0.25 #[m]
CELL_SIZE = 0.4 #[m]
#CELL_SIZE = 1.25 #[m]

import GeometryFunctions as gf_old
import GeometryFunctions_fcl as gf
import ROSUtilities as RU

ft2m     = 0.3048  # [m/ft]
W_Height = 0.0 #233.0 * ft2m # [m]
W_Width  = 0.0 #434.0 * ft2m # [m]
cell     = CELL_SIZE #1.25 # [m]
pix2m    = 0.0  # not really used anymore, was active when got pictures as maps
W_xgrid  = []
W_ygrid  = []
robots_num = 0 #global variable. actually, decided later from the spec file
	
#plant = DubinsCarPlant_[float]() # Default instantiation
#FL_WB = plant.L      # wheel base, for a 36inch length fork, dimension C in spec.
#FL_W  = plant.TotalW # width of car, for all forklifts, dimension G in spec.
#FL_L  = plant.TotalL # length of car, for a 36inch length fork, dimension B in spec.
FL_WB  = 1.882 # wheel base, for a 36inch length fork, dimension C in spec.
#FL_W   = 0.902 # width of car, for all forklifts, dimension G in spec.
FL_W   = 0.31 # width of jackal
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
def PopulateMapWithMP(MotionPrimitives, workspace, obs, no_enter, one_ways, map_kind, cell_h=1.25, cell_w=1.25, force=False, plot=True):
	global W_Width
	global W_xgrid
	global W_ygrid
	global pix2m
	
	tic = timer()

	bounds = np.array(list(workspace.bounds))
	
	#pix2m  = W_Width/(bounds[3]-bounds[1])
	bounds = bounds * pix2m
	
	#import pdb; pdb.set_trace()
	# plot obstacles & total workable space
	if(plot==True):
		ax = plot_map(workspace, obs)
	else:
		ax = 0
		
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
	# GUY, TODO: because I don't prune u-turns with bound overlapping (lab settings will
	# have no trajectory otherwise), this moves the walls a bit in the simulation
	extend_bounds = bounds.copy()
	extend_bounds[0] -= 0.0
	extend_bounds[1] -= 0.0
	extend_bounds[2] += 0.0
	extend_bounds[3] += 0.0
	# create the world map for gazebo simulation
	RU.CreateCustomMapWorld(map_kind, extend_bounds, obstacles)
	#import 	pdb; pdb.set_trace()
	
	if((force == False) and (glob.glob(map_kind + '.pickle'))):
		# if it already exist, save time re-creating the graph
		G = LoadGraphFromFile(map_kind)
		toc = timer()
		print('Motion primitives file exist, so just loading existing (%.2f[sec])' %(toc-tic))
		print(nx.info(G))
		#import pdb; pdb.set_trace()
		return G, ax

	# Two ways to treat do not enter zones: 1. treat as obstacles so it would not have 
	# the funnels on the map. 2. because we loose the options to do reactive stuff in (1)
	# add constraints to the specification to not enter that (this is what's realized currently).
	# also, add the bounds as obstacles so no trajectories will be too close to the walls.
	merged_obs_list = []
	merged_obs_list.extend(obs)
	#merged_obs_list.extend(no_enter)
	wks_obs = []
	wks_obs.append( box( bounds[0], bounds[1]-0.1+FL_W/2.0, bounds[2], bounds[1]    +FL_W/2.0 ) ) # left wall, account for width
	wks_obs.append( box( bounds[0], bounds[3]    -FL_W/2.0, bounds[2], bounds[3]+0.1-FL_W/2.0 ) ) # right wall, account for width
	wks_obs.append( box( bounds[0]-0.1+FL_W/2.0, bounds[1], bounds[0]    +FL_W/2.0, bounds[3] ) ) # top wall, account for width
	wks_obs.append( box( bounds[2]    -FL_W/2.0, bounds[1], bounds[2]+0.1-FL_W/2.0, bounds[3] ) ) # bottom wall, account for width
	#merged_obs_list.extend(wks_obs)
	
	total_count = 0
	G = nx.DiGraph(name='ConnectivityGraph')
	for orient in range(4): #corresponds to (E, N, W, S)
		print('Computing transition map for orientation (%d/4):' %(orient+1))
		with tqdm(total=nX*nY) as pbar:
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

						toRot     = (mp['e'][2]-mp['s'][2]) / (90.0*math.pi/180.0)
						toRot     = (orient+toRot)%4
						connect2  = connect2/cell
						
						#if(x==3 and Y==13 and orient==3 and connect2[0][0]):
						
						# check if the funnel is not oriented with the one way direction
						if(IsOneWayCompliant(x, y, orient, int(toRot), toLoc, one_ways)):
							# check if funnel is in bounds and does not collide with any obstacle
							if(IsPathFree(mp, merged_obs_list, rotmat, orient, x, y, toLoc[0], toLoc[1], \
										  X[0], X[-1], Y[0], Y[-1], ax )):
								# add some weighting based on the fact that you should prefer longer
								# primitives (motion 0 will be better than 4 times motion 2) and
								# turning around would be more expensive than going straight (motion 7-8 vs. motion 0).
								# when constructing the library, the key is ranked with ascending difficulty
								difficulty_factor = 1.0  + key/20.0 #just a factor
								if(key>6):
									difficulty_factor = difficulty_factor*5

								#import pdb; pdb.set_trace()
								# fully-reactive
								#adjList = GenerateAdjacentCells(mp, rotmat, orient, i, j, i+int(connect2[0][0]), j+int(connect2[1][0]), W_xgrid, W_ygrid )
									
								G.add_edge( 'H' + str(int(orient)) + 'X' + str(i) + 'Y' + str(j), \
											'H' + str(int(toRot)) + 'X' + str(i+int(connect2[0][0])) + \
											'Y' + str(j+int(connect2[1][0])), \
											weight=LA.norm(connect2)*difficulty_factor, motion=key, index=total_count) # fully-reactive adjList=adjList )
								total_count += 1
				pbar.update(nY)
		plt.pause(0.05)
		print(' ')
	
	#import pdb; pdb.set_trace()

	print(nx.info(G))
	# save time for next time
	SaveGraphToFile(G, map_kind)
	#import pdb; pdb.set_trace()
	toc = timer()
	print('Motion primitives on map took %.2f[sec]' %(toc-tic))
	return G, ax

# the idea is not to check all available cells if this mp runs through them
# but only the ones that are in the vicinity of the mp.
def GenerateAdjacentCells(mp, rotmat, orient, i1, j1, i2, j2, X, Y ):
	# find out how to span the motion
	i_min = np.min([i1, i2])
	i_max = np.max([i1, i2])
	j_min = np.min([j1, j2])
	j_max = np.max([j1, j2])
	# now, expand by 2 so it can touch those regions too, especially in the middle of a motion (u turns, etc..)
	i_min = np.max([0, i_min-2])
	j_min = np.max([0, j_min-2])
	i_max = np.min([len(X)-1, i_max+2])
	j_max = np.min([len(Y)-1, j_max+2])
	num_ellipses = len(mp['V'])
	# check all these cells if the mp runs through them
	adjacent_cells = []
	for cell_orient in range(4):
		for i in range(i_min, i_max+1):
			for j in range(j_min, j_max+1):
				if(i==i1 and j==j1):
					# not interesting, we know first region intersects with the motion (even if orientation is different)
					continue
				if(i==i2 and j==j2):
					# not interesting, we know last region intersects with the motion
					continue
					
				region = []
				region.append( box( X[i]-cell/2.0, Y[j]-cell/2.0, X[i]+cell/2.0, Y[j]+cell/2.0) ) # x,y is the middle of the cell
				# orient is the motion primitives direction for the ellipses centers & direction. the cells in this
				# situation is independant so it has a different center in the theta direction, +-45 around the cell orientation
				if(cell_orient == 3):
					cell_o = -1 
				else:
					cell_o = cell_orient
				# first ellipse (0) will include also the adjacent regions of region 1 (min. ellipse that covers the cell will) 
				# necessarily be bigger than the region so it will "intersect" with the adjacent region) but that is not interesting
				# because we know that we start in the first region.
				pathfree = IsPathFree(mp, region, rotmat, orient, X[i1], Y[i1], 0, 0, -1e6, 1e6, -1e6, 1e6, -1, \
									  box_center=cell_o*90.0*math.pi/180.0, initial_ell=1, end_ell=num_ellipses-1)
				if(pathfree == False):
					adjacent_cells.append('H' + str(int(cell_orient)) + 'X' + str(i) + 'Y' + str(j))
	return adjacent_cells
	
	
# function that decides if an obstacle-free path exist between start and end point
# taking into account the width of the funnels
def IsPathFree(mp, obstacles, rotmat, orient, xs, ys, xe, ye, xmin, xmax, ymin, ymax, ax, box_center=None, \
			   initial_ell=-1, end_ell=-1, ext_pix2m=None):
	if(ext_pix2m == None):
		global pix2m
	else:
		pix2m = ext_pix2m
	
	# check if you're not going out of bounds (it's fast, but doesn't account for a U-turn!)
	if( (xe < xmin) or (xe > xmax) ):
		return False
	if( (ye < ymin) or (ye > ymax) ):
		return False
	
	if(orient == 3):
		orient = -1 #just so it would look nicer on the 3d plots 270 -> -90
		
	for i, S in enumerate(mp['V']):
		if(i < initial_ell):
			# ignore all ellipses till the first wanted one
			continue
		if((end_ell > 0) and (i >= end_ell)):
			# ignore all ellipses from the last wanted one (so we won't get regions after our terminal point)
			break
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
		e = gf.Ellipsoid(e_center, S)
		if(box_center == None):
			bc = e_center[2]
		else:
			bc = box_center
		#e1 = gf_old.Ellipsoid(e_center, S)
		# iterate through all obstacles to see if any one of them
		# touches any of the ellipsoids (funnel). This does not take
		# into account the funnel in between any two ellipsoids
		for obs in obstacles:
			v = pix2m * np.array(obs.exterior.coords[:])
			b = gf.Box(v, theta_center=bc, theta_delta=np.pi*45./180.) #because that's how we defined the cell
																				#the purpose is the "ignore" the badly scaled
																				#ellipses in the theta direction by making box smaller
			#b1 = gf_old.Box(v)
			overlaps = gf.TestIntersectionBoxEllipsoid(b, e)
			#overlaps1 = gf_old.TestIntersectionBoxEllipsoid(b1, e1)
			#if(overlaps != overlaps1):
			#	import pdb; pdb.set_trace()
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
			e = gf.Ellipsoid(e_center, S)
			plot_ellipsoid(ax, e, orient)
			
			# plot the 2-D ellipse 
			#plot_ellipse(ax, S, e_center, orient)
			
	return True

def IsOneWayCompliant(x, y, orient, toRot, toLoc, one_ways):
	#import pdb; pdb.set_trace()
	pnt = Point(x, y)
	# check the end position. is it in the restricted zone
	pnt2 = Point(toLoc[0][0], toLoc[1][0])
	for key, regions in enumerate(one_ways):
		for idx, region in enumerate(regions):
			start_in = region.contains(pnt)
			end_in = region.contains(pnt2)
			if(start_in == True and end_in == True):
				if(orient != key or toRot != key):
					return False
			if(start_in == True and end_in == False):
				if(orient != key):
					return False
			#if(start_in == False and end_in == True):
			#	if(toRot != key):
			#		return False
			#if((region.contains(pnt) and orient != key) or (region.contains(pnt2) and toRot != key)):
					#print('x=%f\ty=%f in one-way %d' %(x,y,idx))
					# or (toRot != key)
					# not starting with right orientation or not ending with one.
					# currently I'm not using the not ending with one because we
					# might loose funnels the are exiting the one-way isle. on the
					# other hand, inside the isle, those funnels will lead to nowhere else
					# so we're ok.
					#return False
	return True

# this will be loading coordinates of a map from a file in the future
def ReplicateMap(map_kind = 'none'):
	global W_Height
	global W_Width
	global ft2m
	global pix2m
	
	tic = timer()
	
	try:
		with open(map_kind + '.specification', 'r') as spec_file:
			spec = json.load(spec_file)
	except:
		print('Specification %s file has syntax error.' %(map_kind))
		raise
		
	workspace = box(*spec['workspace'])
	obstacles = spec["obstacles"]
	no_entrance = spec["no_entrance"]
	one_way = spec["one_way"]
	obs = []
	no_enter = []
	one_ways = [[], [], [], [] ]
	
	for val in obstacles.values():
		if(len(val)>0):
			obs.append(box(*val))
	for val in no_entrance.values():
		if(len(val)>0):
			no_enter.append(box(*val))
	for key, val in one_way.items():
		if(len(val)>0):
			if(key[0] == 'N'):
				numeric_key = 2
			elif(key[0] == 'E'):
				numeric_key = 1
			elif(key[0] == 'W'):
				numeric_key = 3
			else: 
				#key[0] == 'S'
				numeric_key = 0
				
			one_ways[numeric_key].append(box(*val))
	
	W_Height = workspace.bounds[2] - workspace.bounds[0]  # [m]
	W_Width  = workspace.bounds[3] - workspace.bounds[1] # [m]
		
	pix2m  = W_Width/(workspace.bounds[3]-workspace.bounds[1])
	#import pdb; pdb.set_trace()
	
	toc = timer()
	print('Loading map took %.3f[sec]' %(toc-tic))
	
	return workspace, obs, no_enter, one_ways

def GetGoals(map_kind = 'none'):
	global pix2m
	
	with open(map_kind + '.specification', 'r') as spec_file:
		spec = json.load(spec_file)
	
	all_goals = []
	num_robots = spec['active_robots']
	for i in range(num_robots):
		robot     = spec['robot%d' %i]
		num_goals = robot['goals']
		goals = []
		for j in range(num_goals):
			goal = robot['goal%d' %(j+1)]
			goals.append(goal)
		all_goals.append(goals)
	
	return all_goals, num_robots
	
	'''
	if(map_kind.lower() == 'raymond'):
		goals = np.array([[670., 90., 90.0*math.pi/180.0], [200., 990., 0.0*math.pi/180.0]])
	elif(map_kind.lower() == 'lab'):
		goals = np.array([[ 4.0*ft2m, -7.0*ft2m,  90.0*math.pi/180.0], \
						  [ 5.0*ft2m,  8.5*ft2m,  0.0*math.pi/180.0], \
						  [-7.0*ft2m, -9.*ft2m, -90.0*math.pi/180.0] ]) 
	elif(map_kind.lower() == 'map1'):
		goals = np.array([[63., 197., -90.0*math.pi/180.0], [877., 200., 90.0*math.pi/180.0]])
	elif(map_kind.lower() == 'none'):
		goals = np.array([[70., 70., 90.0*math.pi/180.0], \
						  [10., 120., -90.0*math.pi/180.0]])
	
	return goals
	'''

def MapToFile(map_kind, workspace, obstacles, goals, no_enter, one_ways, scale):
	data = {}
	data['workspace'] = []
	data['obstacles'] = []
	#data['goals'] = []
	data['no_enter'] = []
	data['one_ways'] = []
	
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
	
	data['robots'] = len(goals)
	for i, robot_i in enumerate(goals):
		data['r%d' %i] = []
		for g in robot_i:
			data['r%d' %i].append({
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
	
	for key, regions in enumerate(one_ways):
		for idx, region in enumerate(regions):
			bounds = list(region.bounds)
			data['one_ways'].append({
				'x': bounds[0]*pix2m,
				'y': bounds[1]*pix2m,
				'X': bounds[2]*pix2m,
				'Y': bounds[3]*pix2m,
				'D': key
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
	print('Find shortest path (Dijkstra Graph search) took %.3f[sec]' %(toc-tic))
	return paths

# conversion from arbitrary location on map to the closest funnel (location & orientation)
def GetNodeLabel(pose, ext_xgrid=[None], ext_ygrid=[None], ext_pix2m=None):
	if(None in ext_xgrid):
		global W_xgrid
	else:
		W_xgrid = ext_xgrid
	if(None in ext_ygrid):
		global W_ygrid
	else:
		W_ygrid = ext_ygrid
	if(ext_pix2m == None):
		global pix2m
	else:
		pix2m = ext_pix2m
	#global W_xgrid
	#global W_ygrid

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
		boxx = gf.Box(v) 
		#plot box
		for s, e in combinations(boxx.vertices, 2):
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
def plot_ellipsoid(ax, ellipsoid, orient, color=None): 
	A = ellipsoid.M
	center = ellipsoid.center

	# find the rotation matrix and radii of the axes
	U, s, rotation = LA.svd(A)
	radii = 1.0/np.sqrt(s)

	# sometimes the optimization gives a really small number
	# in the S[2,2] coefficient
	if(np.abs(radii[2]) > math.pi):
		radii[2] = 45.0*math.pi/180.0
		
	# create ellipsoid in spherical coordinates
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
	# plot ellipsoid
	ax.plot_wireframe(x, y, z,  rcount=6, ccount=6, color=clr, alpha=0.2) #rstride=4, cstride=4
	#mlab.mesh(x, y, z) 

# plots the entire sequence leading between goal points
def plot_path(ax, G, paths, MotionPrimitives, robot_i):
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
				# do I need to rotate the ellipsoid as well??? CHECK THIS!
				e = gf.Ellipsoid(e_center, S)
				plot_ellipsoid(ax, e, orient, color=(float(j)/float(len(paths)),robot_i,robot_i))
	
	toc = timer()
	print('Plotting the shortest path took %.2f[sec]' %(toc-tic))

# generates the specification file for slugs (decentralized) 
def UpdateGoalsSlugsInputFile(goals, robot_num, filename='map_funnel'):
	global W_xgrid
	global W_ygrid
	
	tic = timer()
	
	# if this is called then it means we only update the goals so we assume the mapping is correct
	# so we just load it from file
	dbfile = open(filename + '.label2bit', 'rb')
	map_label_2_bit = dill.load(dbfile)
	dbfile.close()
	
	start_label = {}
	finish_label = {}
	
	print('Updating the structured slugs file ...')
		
	goals_r = np.array(goals[robot_num])
	N, __ = goals_r.shape
	#print('Start at: %s' %(GetNodeLabel(goals[0, :])))
	start_label['r%d' %robot_num] = []
	finish_label['r%d' %robot_num] = []
	for i in range(N):
		start = goals_r[i, :]
		if(i == N-1):
			finish = goals_r[0, :] # complete the cycle
		else:
			finish = goals_r[i+1, :]
		start_label['r%d' %robot_num].append(GetNodeLabel(start))
		finish_label['r%d' %robot_num].append(GetNodeLabel(finish))
		#print('Then go to: %s' %(finish_label[-1]))
	

	f = open(filename + '_r' + str(robot_num) + '.structuredslugs', 'r')
	full_file = f.read()
	# because nothing changes in the spec besides the init and liveness,
	# we just edit those specific parts and not compute again intersections with no-enter zones etc.
	i1 = full_file.find('[SYS_LIVENESS]') + 15
	i2 = full_file.find('[SYS_INIT]') - 1
	i3 = full_file.find('[ENV_INIT]') + 11
	i4 = i3 + full_file[i3:].find('\n') + 1

	new_text = full_file[0:i1] 
	cyc_goals = start_label['r%d' %robot_num]
	for g in cyc_goals[1:]:
		new_text = new_text + 'R=%d\n' %( map_label_2_bit[g] )
	new_text = new_text + 'R=%d\n' %( map_label_2_bit[cyc_goals[0]] )
	new_text = new_text + full_file[i2:i3]
	new_text = new_text + 'R=%d\n' %( map_label_2_bit[cyc_goals[0]] )
	new_text = new_text + full_file[i4:]
	f.close()

	new_file = open(filename + '_r' + str(robot_num) + '.structuredslugs', 'w')
	new_file.write(new_text)
	new_file.close()

	print('done.')
	'''
	print('Converting to slugsin ...')
	try:
		resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
	except ValueError:
		pass # Cannot increase limit
	sys.setrecursionlimit(10**6)
	sys.stdout = open(filename + '_r' + str(robot_num) + '.slugsin', "w") # the compiler function is just printing on screen
	slugscomp.performConversion(filename + '_r' + str(robot_num) + '.structuredslugs', False)
	sys.stdout = sys.__stdout__  # convert back to printing on screen
	'''
	Convert2Slugsin(filename, [robot_num])
	print('done.')
	
	toc = timer()
	print('Creating structuredslugs & converting to slugsin file took %.2f[sec]' %(toc-tic))
	
	return map_label_2_bit

# generates the specification file for slugs (decentralized) 
def UpdateRestrictionSlugsInputFile(initial_pose, blocked_funnels, goals, robot_num, filename='map_funnel'):
	# if this is called then it means we only update the goals so we assume the mapping is correct
	# so we just load it from file
	#dbfile = open(filename + '.label2bit', 'rb')
	#map_label_2_bit = dill.load(dbfile)
	#dbfile.close()
	tic = timer()
	start_label = {}
	finish_label = {}
	
	print('Updating the structured slugs file ...')
	# in this function, the goals are already the cell numbers	
	N = len(goals)
	#print('Start at: %s' %(GetNodeLabel(goals[0, :])))
	start_label['r%d' %robot_num] = []
	finish_label['r%d' %robot_num] = []
	for i in range(N):
		start = goals[i]
		if(i == N-1):
			finish = goals[0] # complete the cycle
		else:
			finish = goals[i+1]
		start_label['r%d' %robot_num].append(goals[i])
		finish_label['r%d' %robot_num].append(finish)

	f = open(filename + '_r' + str(robot_num) + '.structuredslugs', 'r')
	full_file = f.read()
	# because nothing changes in the spec besides the init and liveness,
	# we just edit those specific parts and not compute again intersections with no-enter zones etc.
	i1 = full_file.find('[SYS_LIVENESS]') + 15
	i2 = full_file.find('[SYS_INIT]') - 1
	i3 = full_file.find('[ENV_INIT]') + 11
	i4 = full_file.find('[SYS_TRANS]') + 12
	i5 = i3 + full_file[i3:].find('\n') + 1
	i6 = full_file.find('##') + 3
	
	#import pdb; pdb.set_trace()
	new_restrictions = ''
	if(i6 == 2):
		# because if it fails it will find -1, but we add +3 to it
		i6 = len(full_file)
		new_restrictions = '##\n'

	new_text = full_file[0:i1] 
	cyc_goals = start_label['r%d' %robot_num]
	for g in cyc_goals:
		new_text = new_text + 'R=%d\n' %( g )
	
	new_text = new_text + full_file[i2:i3]
	new_text = new_text + 'R=%d\n' %( initial_pose ) #the actual current position
	
	new_text = new_text + full_file[i5:i6]
	
	for i, batch in enumerate(blocked_funnels):
		for bf in batch:
			new_restrictions = new_restrictions + ('!(R=%d)\n' %(bf))
		
	new_text = new_text + new_restrictions
	
	f.close()

	new_file = open(filename + '_r' + str(robot_num) + '.structuredslugs', 'w')
	new_file.write(new_text)
	new_file.close()
	#import pdb; pdb.set_trace()
	print('done.')
	'''
	print('Converting to slugsin ...')
	try:
		resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
	except ValueError:
		pass # Cannot increase limit
	sys.setrecursionlimit(10**6)
	sys.stdout = open(filename + '_r' + str(robot_num) + '.slugsin', "w") # the compiler function is just printing on screen
	slugscomp.performConversion(filename + '_r' + str(robot_num) + '.structuredslugs', False)
	sys.stdout = sys.__stdout__  # convert back to printing on screen
	print('done.')
	'''
	Convert2Slugsin(filename, [robot_num])
	toc = timer()
	print('Creating structuredslugs & converting to slugsin file took %.2f[sec]' %(toc-tic))
	
	#return map_label_2_bit

# generates the specification file for slugs (decentralized) 
def CreateSlugsInputFile(G, goals, MP, no_enter, robots_num, robot_idx=None, filename='map_funnel', \
						 ext_xgrid=[None], ext_ygrid=[None], ext_pix2m=None, ext_ic=None, map_label_2_bit={}):
	if(None in ext_xgrid):
		global W_xgrid
	else:
		W_xgrid = ext_xgrid
	if(None in ext_ygrid):
		global W_ygrid
	else:
		W_ygrid = ext_ygrid
	if(ext_pix2m == None):
		global pix2m
	else:
		pix2m = ext_pix2m
		
	tic = timer()
	
	Nnodes = nx.number_of_nodes(G)
	Nedges = nx.number_of_edges(G)
	
	start_label = {}
	finish_label = {}
	
	print('Creating the structured slugs file ...')
	
	#robots_num = len(goals)
	
	for r in range(robots_num):
		goals_r = np.array(goals[r])
		N, __ = goals_r.shape
		#print('Start at: %s' %(GetNodeLabel(goals[0, :])))
		start_label['r%d' %r] = []
		finish_label['r%d' %r] = []
		for i in range(N):
			start = goals_r[i, :]
			if(i == N-1):
				finish = goals_r[0, :] # complete the cycle
			else:
				finish = goals_r[i+1, :]
			start_label['r%d' %r].append(GetNodeLabel(start,ext_xgrid=W_xgrid, ext_ygrid=W_ygrid,ext_pix2m=pix2m))
			finish_label['r%d' %r].append(GetNodeLabel(finish,ext_xgrid=W_xgrid, ext_ygrid=W_ygrid,ext_pix2m=pix2m))
			#print('Then go to: %s' %(finish_label[-1]))
	
	if(len(map_label_2_bit)==0):
		#map_label_2_bit = {}
		node_count = 0
		ext_map_l2b = False
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
	else:
		# use the same structure you currently have
		node_count = len(map_label_2_bit)
		ext_map_l2b = True
		
	full_file = ''
	if (robot_idx == None):
		robot_vec = range(robots_num)
		reuse_spec = True
	else:
		robot_vec = [robot_idx]
		reuse_spec = False
		
	for self_r in robot_vec:
		with open(filename + '_r' + str(self_r) + '.structuredslugs', 'w+') as f: 	
			if((self_r>0) and (reuse_spec == True) and (full_file != '')):
				# because nothing changes in the spec besides the init and liveness,
				# we just edit those specific parts and not compute again intersections with no-enter zones etc.
				i1 = full_file.find('[SYS_LIVENESS]') + 15
				i2 = full_file.find('[SYS_INIT]') - 1
				i3 = full_file.find('[ENV_INIT]') + 11
				i4 = i3 + full_file[i3:].find('\n') + 1

				new_text = full_file[0:i1] 
				cyc_goals = start_label['r%d' %self_r]
				for g in cyc_goals[1:]:
					new_text = new_text + 'R=%d\n' %( map_label_2_bit[g] )
				new_text = new_text + 'R=%d\n' %( map_label_2_bit[cyc_goals[0]] )
				new_text = new_text + full_file[i2:i3]
				new_text = new_text + 'R=%d\n' %( map_label_2_bit[cyc_goals[0]] )
				new_text = new_text + full_file[i4:]
				
				new_file = open(filename + '_r' + str(self_r) + '.structuredslugs', 'w')
				new_file.write(new_text)
				new_file.close()
				continue
			# structured slugs
			f.write('[INPUT]\n') 
			#import pdb; pdb.set_trace()
			other_robots = np.setdiff1d(np.arange(0,robots_num), np.array([self_r]) )
			total_actions    = len(MP) 
			
			f.write('R:0...%d\n' %( node_count ) ) # existence of self robot in funnel/region
			#for r in other_robots:
			if(len(other_robots)>0):
				for act in range(total_actions):
					f.write('B_%d\n' %act) # %( r, act ) ) # existence of any robot anywhere nearby when taking funnel j
			f.write('\n')

			f.write('[OUTPUT]\n')
			# every mp is an action, +1 (remember it's zero based so no actual addition) for the action 'stay in place'
			mp_stay_in_place = total_actions
			f.write('mp:0...%d\n' %( total_actions )) # action mp of robot i 
			f.write('\n')

			#import pdb; pdb.set_trace()
			f.write('[SYS_LIVENESS]\n')
			# all the points we want to reach (goals)
			cyc_goals = start_label['r%d' %self_r]
			for g in cyc_goals[1:]:
				f.write('R=%d\n' %( map_label_2_bit[g] ))
			f.write('R=%d\n' %( map_label_2_bit[cyc_goals[0]] ))
			f.write('\n')

			f.write('[SYS_INIT]\n') 
			f.write('!(mp=%d)\n' %( total_actions ))
			f.write('\n')

			f.write('[ENV_TRANS]\n')
			all_mps = Set(np.arange(0, len(MP)))
			list_of_formulas = []
			list_of_mp_constraints = []
			list_of_liveness_assmp = []
			dict_connections = {}
			# first, add all theoretically available motions to the environment transitions
			for parent in G:
				# add functionality to stay in place (added first with the hope of choosing this rather than some
				# other mp which leads to nowhere when in conflict with another robot
				f.write('(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
													   mp_stay_in_place, \
													   map_label_2_bit[parent] ))
				children = G.neighbors(parent) #list of successor nodes of parent
				avail_links = Set([])
				list_of_avail_funnels = []
				for child in children:
					mp_num = G[parent][child]['motion']
					avail_links.update([mp_num])

					# all the points we wish to avoid (do not enter zones)
					orient, xs, ys = [int(s) for s in re.findall(r'-?\d+\.?\d*', parent)] # extract Ellipsoid pose
					#if(map_label_2_bit[parent] == 1290): #'H1X12Y6'
					#	import pdb; pdb.set_trace()
					out_of_no_enter = IsPathFree(MP[mp_num], no_enter, GetRotmat(orient), orient, W_xgrid[xs], W_ygrid[ys], 0, 0, \
									  -10e6, 10e6, -10e6, 10e6, 0, ext_pix2m=pix2m ) #the last 7 parameters don't really matter here
					if(out_of_no_enter):
						# good, does not intersect the no-entry zones. need to check if other robots do not interfere
						#f.write('(R=%d & (R%d=%d) & mp=%d)->(R\'=%d)\n' %(\
						#					map_label_2_bit[parent], \
						#					r, map_label_2_bit[child], \
						#					mp_num, \
						#					map_label_2_bit[parent] ))
						list_of_avail_funnels.append(map_label_2_bit[child])
						'''
						try:
							dict_connections[map_label_2_bit[child]].append(map_label_2_bit[parent])
						except KeyError:
							dict_connections[map_label_2_bit[child]] = []
							dict_connections[map_label_2_bit[child]].append(map_label_2_bit[parent])
						'''
						# connectivity map (action->result)
						'''# the new way:
						possible_places_to_stop = ''
						adjList = G[parent][child]['adjList']
						if 0:
							for poss_region in adjList:
								try:
									possible_places_to_stop = possible_places_to_stop + (' | R\'=%d' %(map_label_2_bit[poss_region]))
								except:
									# GUY, TODO: check why this happens. i think it's not supposed to happen
									# this happens when there's no bit label for this label
									pass
						possible_places_to_stop = possible_places_to_stop + ')'
						'''
						
						'''# the new way: to the new region, or got stuck in place	
						f.write('(R=%d & mp=%d)->((R=%d)W(R=%d)%s\n' %(\
											map_label_2_bit[parent], \
											mp_num, \
											map_label_2_bit[parent], map_label_2_bit[child], possible_places_to_stop ))
						'''
						#f.write('(R=%d & mp=%d & B_%d)->(R\'=%d | R\'=%d%s\n' %(\
						#					map_label_2_bit[parent], \
						#					mp_num, mp_num, \
						#					map_label_2_bit[child], map_label_2_bit[parent], possible_places_to_stop )) 
						#f.write('(R=%d & mp=%d & !B_%d)->(R\'=%d)\n' %(\
						#					map_label_2_bit[parent], \
						#					mp_num, mp_num, \
						#					map_label_2_bit[child])) 
						#f.write('(R=%d & mp=%d & !B_%d)->(F(R=%d))\n' %(\
						#					map_label_2_bit[parent], \
						#					mp_num, mp_num, \
						#					map_label_2_bit[child])) 
						#list_of_liveness_assmp.append('(R=%d & mp=%d)->(R\'=%d)\n' %(\
						#					map_label_2_bit[parent], \
						#					mp_num, \
						#					map_label_2_bit[child]) )
						#list_of_liveness_assmp.append('(R=%d & mp=%d)->(<>R=%d)\n' %(\
						#					map_label_2_bit[parent], \
						#					mp_num, \
						#					map_label_2_bit[child]) )
						
						# the old deterministic way
						f.write('(R=%d & mp=%d)->(R\'=%d)\n' %(\
											map_label_2_bit[parent], \
											mp_num, \
											map_label_2_bit[child] )) 
						
						# not needed
						#for r in other_robots:
						#	list_of_mp_constraints.append('(R=%d & (R%d=%d))->!(mp\'=%d)\n' %(map_label_2_bit[parent], \
						#																	   r, map_label_2_bit[child], mp_num))
						
					else:
						# intersects a no entry zone. can add reactivity right here.
						# for now, it's a global avoid this region (as if we would've treated it as an obstacle)
						# the # is just because if it's enabled it would add a state for every single motion 
						# primitive. whereas, mp=stay is enough to capture that. it will reduce amount of states,
						# although even if it was there, the shortest path would not have selected that
						#f.write('#(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
						#									   mp_num, \
						#									   map_label_2_bit[parent] ))
						list_of_mp_constraints.append('(R=%d)->!(mp=%d)\n' %(map_label_2_bit[parent], mp_num))

				# store the available formula for this funnel for the other robots
				# option 1
				'''
				formula_str = '(X=%d)->(' %map_label_2_bit[parent]
				for next_funnel in list_of_avail_funnels:
					formula_str = formula_str + ('(X\'=%d <-> !(R=%d)) | ' %(next_funnel, next_funnel))
				formula_str = formula_str + ('X\'=%d)\n' %(map_label_2_bit[parent]))
				list_of_formulas.append(formula_str)
				'''		
				# connectivity map
				'''
				formula_str = '(X=%d)->(' %map_label_2_bit[parent]
				for next_funnel in list_of_avail_funnels:
					formula_str = formula_str + ('X\'=%d | ' %(next_funnel))
				formula_str = formula_str + ('X\'=%d)\n' %(map_label_2_bit[parent]))
				list_of_formulas.append(formula_str)
				'''
				# explicitly state that all actions that are unavailable (were removed because of an obstacle)
				# are pointing the the same funnel
				links_not_provided = all_mps - avail_links
				while(len(links_not_provided)>0):
					bad_mp = links_not_provided.pop()
					#f.write('#(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
					#								   bad_mp, \
					#								   map_label_2_bit[parent] ))
					list_of_mp_constraints.append('(R=%d)->!(mp=%d)\n' %(map_label_2_bit[parent], \
													   bad_mp))
			# option 2
			'''
			for decendant, parents in dict_connections.items():
				formula_str = '((' 
				for next_funnel in parents:
					formula_str = formula_str + ('X=%d | ' %(next_funnel))
				formula_str = formula_str[0:-3] + (') & (R=%d | R\'=%d))->!(X\'=%d)\n' %(decendant, \
																				   decendant, decendant))
				list_of_formulas.append(formula_str)
			'''
			#import pdb; pdb.set_trace()
			# this is for the other robots, how can they move but we don't worry ourselves with their exact
			# policies, only that they cannot hit static obstacles nor the main robot
			f.write('\n')
													  
			f.write('[ENV_LIVENESS]\n')
			# all the points the other robots want to reach (goals)
			live_str = '('
			for act in range(total_actions):
				live_str = live_str + 'X_%d & ' %( act )
			live_str = live_str[0:-3] + ')\n'
			for r in other_robots:
				adv_far_away = live_str.replace('X', '!B') #'!R%d'%r)
				adv_near = live_str.replace('X', 'B') #'R%d'%r)
				adv_near = adv_near.replace('&', '|')
				f.write(adv_far_away)
				f.write('#' + adv_near)
				#for g in start_label['r%d' %r]:
				#	f.write('R%d=%d\n' %( r, map_label_2_bit[g] ))
			f.writelines(list_of_liveness_assmp)
			f.write('\n')
			
			f.write('[ENV_INIT]\n')
			if(ext_ic == None):
				f.write('R=%d\n' %( map_label_2_bit[start_label['r%d' %self_r][0]] )) 
			else:
				f.write('R=%d\n' %( ext_ic )) 
			# need to change this, either to have no specific start point, or, start at the current location,
			# or have some mechanism to get from current location to start position
			#for r in other_robots:
			if(len(other_robots)>0):
				adv_far_away = live_str.replace('X', '!B') #'!R%d'%r)
				f.write(adv_far_away)
				#for act in range(total_actions):
				#	f.write('!R%d_%d\n' %(r, act )) # existence of any robot in funnel close to self robot
			f.write('\n')

			'''
			for r in other_robots:
				res = [sub.replace('X', 'R%d'%r) for sub in list_of_formulas]
				f.writelines(res)
				f.write('!(R%d\'=R)\n' %r)
				f.write('!(R%d\'=R\')\n' %r)
			f.write('\n')
			'''
						
			f.write('[SYS_TRANS]\n')
			#for r in other_robots:
			#	f.write('!(R\'=R%d)\n' %r)
			#for r in other_robots:
			if(len(other_robots)>0):
				for act in range(total_actions):
					#f.write('R%d_%d\'->!(mp\'=%d)\n' %(r, act, act )) # existence of any robot in funnel close to self robot
					f.write('B_%d\'->!(mp\'=%d)\n' %(act, act )) # existence of any robot in funnel close to self robot
					#f.write('B_%d->!(mp\'=%d)\n' %(act, act )) # existence of any robot in funnel close to self robot
			f.write('\n')
			
			# this is the robot which the policy is about
			f.writelines(list_of_mp_constraints)
													  			
			if(self_r == 0):
				#store for later use with other robots
				f.seek(0)
				full_file = f.read()
		
	print('done.')
	'''
	print('Converting to slugsin ...')
	try:
		resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
	except ValueError:
		pass # Cannot increase limit
	sys.setrecursionlimit(10**6)
	for self_r in range(robots_num):
		sys.stdout = open(filename + '_r' + str(self_r) + '.slugsin', "w") # the compiler function is just printing on screen
		slugscomp.performConversion(filename + '_r' + str(self_r) + '.structuredslugs', False)
		sys.stdout = sys.__stdout__  # convert back to printing on screen
	print('done.')
	'''
	Convert2Slugsin(filename, robot_vec) #range(robots_num))
	# store for easy access when running realtime
	if(ext_map_l2b == False):
		dbfile = open(filename + '.label2bit', 'wb')
		dill.dump(map_label_2_bit, dbfile)
		dbfile.close()
	
	toc = timer()
	print('Creating structuredslugs & converting to slugsin file took %.2f[sec]' %(toc-tic))

	return map_label_2_bit


# convert to slugsin.
def Convert2Slugsin(filename, robots_num):
	print('Converting to slugsin ...')
	try:
		resource.setrlimit(resource.RLIMIT_STACK, (2**29,-1))
	except ValueError:
		pass # Cannot increase limit
	sys.setrecursionlimit(10**6)
	for self_r in robots_num:
		sys.stdout = open(filename + '_r' + str(self_r) + '.slugsin', "w") # the compiler function is just printing on screen
		slugscomp.performConversion(filename + '_r' + str(self_r) + '.structuredslugs', False)
		sys.stdout = sys.__stdout__  # convert back to printing on screen
	print('done.')


# get a complete synthesized controller for our spec.
def CheckRealizeability(robots_num, filename='map_funnel', robot_num=-1):
	tic = timer()
	controllers = []
	slugsLink = '/home/cornell/Tools/slugs_ltl_stack/src/slugs'
	realizable = True
	robot_fail_number = -1
	#import pdb; pdb.set_trace()
	if(robot_num == -1):
		# check all
		check_vector = range(robots_num)
	else:
		# check only the updated one
		check_vector = range(robot_num, robot_num+1)
	print('Starting to check realizeability. This might take some time ...')
	#import pdb; pdb.set_trace()
	for r in check_vector:
		if(not realizable):
			break;
			
		baseName  = '%s_r%d' %(filename, r)
		specFile  = baseName + '.slugsin' 
		
		slugsProcess = subprocess.Popen(slugsLink + ' ' + specFile, bufsize=1048000, stderr=subprocess.PIPE, \
                                    shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
		# wait for synthesis to be done
		stderr, stdout = "",""
		
		while True: 
			# retrieve outputs
			stdout += slugsProcess.stdout.readline().strip()
			stderr += slugsProcess.stderr.readline().strip()
			# exit if synthesis is done and ready for execution
			if "error" in stderr.lower():
				realizable = False
				robot_fail_number = r
				break
			elif "unrealizable" in stderr:
				realizable = False
				robot_fail_number = r
				break
			elif "realizable" in stderr:
				break
		print('Robot %d done ...' %(r))
						
	toc = timer()
	print('Checking realizeability for the %d robots, via slugs took %.2f[sec]' %(robots_num, toc-tic))
	
	if(realizable):
		print('All specifications are realizeable.\nHURRAY!')
	else:
		print('Robot%d specification is not realizeable or error occured with its spec.' %robot_fail_number)
	return realizable

# get a complete synthesized controller for our spec.
def SynthesizeController(robots_num, filename='map_funnel'):
	tic = timer()
	controllers = []
	slugsLink = '/home/cornell/Tools/slugs_ltl_stack/src/slugs'
	
	robots_num = 1 # GUY, TO REMOVE!!!
	for r in range(robots_num):
		baseName  = '%s_r%d' %(filename, r)
		specFile  = baseName + '.slugsin' 
		
		#--interactiveStrategy
		slugsProcess = subprocess.Popen(slugsLink + ' --explicitStrategy --cooperativeGR1Strategy --jsonOutput ' + specFile, \
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
		controllers.append(json.loads(inputAPs))
		with open(baseName + '.json', "w") as write_file:
			json.dump(controllers[-1], write_file)
	
	toc = timer()
	print('Synthesizing control via slugs took %.2f[sec]' %(toc-tic))
	
	return controllers

# decipher the sequence of moves to satisfy spec.
def GetSpecificControl(Controllers, map_bit_2_label, filename='map_funnel', debug=True):
	tic = timer()
	
	trans = 0
	actions_count = 0
	#quick hack until I deal with several controllers
	try:
		C = Controllers[0] 
	except:
		C = Controllers
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
		print('Extracting plan (from slugs) took %.2f[sec]' %(toc-tic))
	
	#import pdb; pdb.set_trace()
	jsonList = []
	for i in range(len(states)):
		jsonList.append({"state" : states[i], "action" : mps[i]})
	with open(filename + '.states', 'wt') as f:
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
	
	map_kind = MAP_KIND 
	
	MP = LoadMP(fName='MPLibrary.lib')
	MotionPrimitivesToFile(map_kind, MP)
	#import pdb; pdb.set_trace()
	workspace, obs, no_enter, one_ways = ReplicateMap(map_kind=map_kind) #
	goals, robots_num = GetGoals(map_kind=map_kind)
	# save it to file
	MapToFile(map_kind.lower(), workspace, obs, goals, no_enter, one_ways, pix2m)
	
	DiGraph, ax = PopulateMapWithMP(MP, workspace, obs, no_enter, one_ways, map_kind, cell_h=cell, cell_w=cell, force=force)
	
	print('Searching for naive paths (disregarding no-entry zones) using graph search ...')
	for i in range(robots_num):
		# plot each individual robot's path
		path = FindPathBetweenGoals(DiGraph, np.array(goals[i]) )
		plot_path(ax, DiGraph, path, MP, float(i)/float(robots_num))
	plt.pause(0.05)
	#import pdb; pdb.set_trace()
	# create the amount of jackals you want
	robots_ic = []
	for i in range(robots_num):
		robots_ic.append([pix2m*goals[i][0][0], pix2m*goals[i][0][1], goals[i][0][2]])
	RU.CreateJackals(map_kind, robots_ic)
	#import pdb; pdb.set_trace()
	map_label_2_bit = CreateSlugsInputFile(DiGraph, goals, MP, no_enter, robots_num, filename=map_kind) #'map_funnel')
	# the reverse dictionary is useful
	map_bit_2_label = dict((v, k) for k, v in map_label_2_bit.items())
	
	CheckRealizeability(robots_num, filename=map_kind)
	#synctrl = SynthesizeController(robots_num, filename=map_kind) #'map_funnel')
	#GetSpecificControl(synctrl, map_bit_2_label, filename=map_kind)
	print('Done. Close figure to exit.')
	plt.show(block=True)
	

							 	