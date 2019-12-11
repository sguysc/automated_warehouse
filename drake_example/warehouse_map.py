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

import networkx as nx

from numpy import linalg as LA
from shapely.geometry import Polygon, box


from StructuredSlugsParser import compiler as slugscomp

#from DubinsPlantCar import *
#from pydrake.symbolic import Expression
#from pydrake.all import PiecewisePolynomial

import GeometryFunctions as gf
import ROSUtilities as RU

ft2m     = 0.3048
W_Height = 0.0 #233.0 * ft2m # [m]
W_Width  = 0.0 #434.0 * ft2m # [m]
cell     = 1.25 # [m]
pix2m    = 0.0
W_xgrid  = []
W_ygrid  = []
	
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
	
def PopulateMapWithMP(MotionPrimitives, workspace, obs, map_kind, cell_h=1.25, cell_w=1.25):
	global W_Width
	global W_xgrid
	global W_ygrid
	global pix2m
	
	tic = timer()

	bounds = np.array(list(workspace.bounds))
	pix2m  = W_Width/(bounds[3]-bounds[1])
	bounds = bounds * pix2m
	
	# plot obstacles & total workable space
	ax = plot_map(workspace, obs)
		
	X = np.arange(bounds[0]+cell, bounds[2]-cell, cell)
	W_xgrid = X.copy()
	Y = np.arange(bounds[1]+cell, bounds[3]-cell, cell)
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
	import 	pdb; pdb.set_trace()
	
	if(glob.glob(map_kind + '.*')):
		# if it already exist, save time re-creating the graph
		G = LoadGraphFromFile(map_kind)
		toc = timer()
		print('Motion primitives file exist, so just loading (%f[sec])' %(toc-tic))
		return G, ax

	#possible_orientations = {0: 0.*math.pi/180.0, 1: 90.*math.pi/180.0, 2: 180.*math.pi/180.0, 3: 270.*math.pi/180.0}
	
	total_count = 0
	G = nx.DiGraph(name='ConnectivityGraph')
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
								  X[0], X[-1], Y[0], Y[-1], ax )):
						toRot    = (mp['e'][2]-mp['s'][2]) / (90.0*math.pi/180.0)
						toRot    = (orient+toRot)%4
						connect2 = connect2/cell
						# add some weighting based on the fact that you should prefer longer
						# primitives (motion 0 will be better than 4 times motion 2) and
						# turning around would be more expensive than going straight (motion 7-8 vs. motion 0).
						# when constructing the library, the key is ranked with ascending difficulty
						difficulty_factor = 1.0  + key/20.0 #just a factor
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
def IsPathFree(workspace, mp, obstacles, rotmat, orient, xs, ys, xe, ye, xmin, xmax, ymin, ymax, ax):
	global pix2m
	# check if you're not going out of bounds (doesn't account for a U-turn!)
	if( (xe < xmin) or (xe > xmax) ):
		return False
	if( (ye < ymin) or (ye > ymax) ):
		return False
	
	if(orient == 3):
		orient = -1 #just so it would look nicer on the 3d plots 270 -> -90
		
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

def ReplicateMap(map_kind = 'none'):
	global W_Height
	global W_Width
	global ft2m
	
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
		
		goals = np.array([[670., 90., 0.0], [200., 990., -90.0*math.pi/180.0]])
		
		W_Height = 233.0 * ft2m # [m]
		W_Width  = 434.0 * ft2m # [m]

	elif(map_kind.lower() == 'map1'):
		workspace = box( 0.0, 0.0, 100.0, 200.0 )
		obs = []
		obs.append( box( 15., 50.0, 25.0,  150.0 ) ) # rack, in pixels
		obs.append( box( 45., 50.0, 55.0,  150.0 ) ) # rack, in pixels
		obs.append( box( 75., 50.0, 85.0,  130.0 ) ) # rack, in pixels
		
		goals = np.array([[63., 197., -90.0*math.pi/180.0], [877., 200., 90.0*math.pi/180.0]])
		
		W_Height = 30.0 # [m]
		W_Width  = 60.0 # [m]
	
	elif(map_kind.lower() == 'none'):
		workspace = box( 0.0, 0.0, 100.0, 200.0 )
		obs = []
		obs.append( box( 30.0, 60.0, 50.0,  140.0 ) ) # rack, in pixels
		
		goals = np.array([[70., 70., 0.0], \
						  [10., 120., 180.0*math.pi/180.0]])
		
		W_Height = 20.0 # [m]
		W_Width  = 40.0 # [m]
		
	#import pdb; pdb.set_trace()
	toc = timer()
	print('Loading map took %f[sec]' %(toc-tic))
	
	return workspace, obs, goals

def FindPathBetweenGoals(G, goals):
	global pix2m
	global cell
	
	tic = timer()
		
	N, __ = goals.shape
	paths = []
	#import pdb; pdb.set_trace()
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
		except:
			print('Could not find any path between %s to %s' %(start_label, finish_label))
			#import pdb; pdb.set_trace()
			paths.append([])

	toc = timer()
	print('Find shortest path (Dijkstra Graph search) took %f[sec]' %(toc-tic))
	
	return paths

def GetNodeLabel(pose):
	global W_xgrid
	global W_ygrid

	#import pdb; pdb.set_trace()
	orient = int(np.round( pose[2] / (math.pi/2.0) ) % 4)  # what quadrant you're closest to
	label = 'H' + str(orient) + 'X' + str(find_nearest(W_xgrid, pix2m*pose[0])) + \
								'Y' + str(find_nearest(W_ygrid, pix2m*pose[1]))
	
	return label

def find_nearest(arr, value):
	arr = np.asarray(arr)
	idx = (np.abs(arr - value)).argmin()
	return idx
	
def PlotGraph(G):
	pos=nx.spring_layout(G, iterations=10)
	nx.draw(G,pos,edgelist=G.edges(),node_size=50,with_labels=False)
	plt.show()

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

	ax.set_aspect("equal")
	plt.show(block = False)
	plt.pause(0.05)

	return ax

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
	ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color=clr, alpha=0.2)

	#plt.show() #block = False)
	#plt.pause(0.05)	

def plot_path(ax, G, paths, MotionPrimitives):
	global W_xgrid
	global W_ygrid

	tic = timer()
	
	#import pdb; pdb.set_trace()
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
			
			if(orient == 0):
				rotmat = np.array([[0.,-1.], [1.,0.]])
			elif(orient == 1):
				rotmat = np.array([[-1.,0.], [0.,-1.]])
			elif(orient == 2):
				rotmat = np.array([[0.,1.], [-1.,0.]])
			else:
				rotmat = np.array([[1.,0.], [0.,1.]])

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

def CreateSlugsInputFile(G, goals, MP, filename='map_funnel'):
	
	tic = timer()
	
	Nnodes = nx.number_of_nodes(G)
	Nedges = nx.number_of_edges(G)
	
	start_label = []
	finish_label = []
	
	print('Creating to structured slugs file ...')
	
	N, __ = goals.shape
	print('Start at: %s' %(GetNodeLabel(goals[0, :])))
	
	for i in range(N):
		start = goals[i, :]
		if(i == N-1):
			finish = goals[0, :] # complete the cycle
		else:
			finish = goals[i+1, :]
		start_label.append(GetNodeLabel(start))
		finish_label.append(GetNodeLabel(finish))
		print('Then go to: %s' %(finish_label[-1]))
	
	map_label_2_bit = {}
	node_count = 0
	#import pdb; pdb.set_trace()
	for node in G:
		map_label_2_bit.update({node: node_count})
		node_count = node_count + 1
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
		for parent in G:
			children = G.neighbors(parent) #list of successor nodes of parent
			avail_links = Set([])
			for child in children:
				grandchild = G.neighbors(child) 
				#if(grandchild.__length_hint__()>0):
					# if it has no grandchild then it was probably removed earlier
					# and it is not even available in the dictionary
				avail_links.update([G[parent][child]['motion']])
				f.write('(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
													   G[parent][child]['motion'], \
													   map_label_2_bit[child]))
			links_not_provided = all_mps - avail_links
			while(len(links_not_provided)>0):
				f.write('(R=%d & mp=%d)->(R\'=%d)\n' %(map_label_2_bit[parent], \
												   links_not_provided.pop(), \
												   map_label_2_bit[parent]))
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
	while True:
		in_state = C['nodes'][str(trans)]
		state = in_state['state']
		# it comes in reverse binary
		action_str = state[::-1][0:actions_count]
		action = 0
		for b in action_str:
			action = 2 * action + b
		mps.append(action)
		
		cur_state_str = state[::-1][actions_count:]
		cur_state = 0
		for b in cur_state_str:
			cur_state = 2 * cur_state + b
		
		states.append(map_bit_2_label[cur_state])
		if(debug):
			print('%d) In state: %s, take motion primitive: %d' %(cntr, states[-1], action))
		
		trans = in_state['trans'][0]
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
	
	return states, mps


def SaveGraphToFile(G, filename):
	nx.write_gpickle(G, filename + '.pickle')

def LoadGraphFromFile(filename):
	return nx.read_gpickle(filename + '.pickle')
	
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
	map_kind = 'none' 
	#map_kind = 'raymond'
	MP = LoadMP(fName='MPLibrary.lib')
	workspace, obs, goals = ReplicateMap(map_kind=map_kind) #
	
	DiGraph, ax = PopulateMapWithMP(MP, workspace, obs, map_kind, cell_h=cell, cell_w=cell)
	path = FindPathBetweenGoals(DiGraph, goals)
	# create the amount of jackals you want
	RU.CreateJackals([pix2m*goals[0]])
	#import pdb; pdb.set_trace()
	
	plot_path(ax, DiGraph, path, MP)
	#import pdb; pdb.set_trace()
	map_label_2_bit = CreateSlugsInputFile(DiGraph, goals, MP, filename='map_funnel')
	# the reverse dictionary is useful
	map_bit_2_label = dict((v, k) for k, v in map_label_2_bit.items())
	
	synctrl = SynthesizeController(filename='map_funnel')
	GetSpecificControl(synctrl, map_bit_2_label)
	
	plt.show(block=True)
	

							 