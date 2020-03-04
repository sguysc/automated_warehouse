
import numpy as np
import fcl

from timeit import default_timer as timer

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # This import registers the 3D projection, but is otherwise unused

	
class Ellipsoid:
	def __init__(self, center, M):
		self.center = center.copy()
		# wrap the angle between (-pi,+pi)
		#self.center[2] = (( -self.center[2] + np.pi) % (2.0 * np.pi ) - np.pi) * -1.0
		#no need to fix the theta (for badly scaled matrices), it will be handled in the Box class (see how)
		self.M      = M.copy()
		
		# find the rotation matrix and radii of the axes
		rotation, ss, rotationT = np.linalg.svd(M)
		self.radii = 1.0/np.sqrt(ss)
				
		# with this line I ensure that the vectors comply with the right hand rule for axes
		rotation[:,2] = np.cross(rotation[:,0], rotation[:,1])
		
		self.e   = fcl.Ellipsoid(self.radii[0], self.radii[1], self.radii[2])
		self.tf  = fcl.Transform(rotation, self.center)   # Matrix rotation and translation
		# create the collision object
		self.obj = fcl.CollisionObject(self.e, self.tf)

class Ellipse(Ellipsoid):
	def __init__(self, center, M):
		# pretty straightforward here, artificially inflate to a 3-d ellipsoid
		# inflate
		M = np.insert(np.insert(M, 2, 0, axis=1), 2, 0, axis=0)
		M[2,2] = 1e6 # this gives the theta axis a very small radii
		center = np.hstack((center.copy(), 0.0))
		# now it's an inflated ellipsoid so continue on
		Ellipsoid.__init__(self, center, M)
		
class Box:
	def __init__(self, vertices, theta_center=0.0, theta_delta=6.2832):
		# we do assume here it's a box and not some other thing like a parallelogram 
		# we fix the sometimes bad scale of the theta axis in the ellipsoid by only creating
		# a box with +- 45deg length on each side thus radii with huge numbers won't eventually touch the box.
		# we do not fix it in the ellipsoid class because the svd/eig change order of eigenvalues
		# and we can't fix on the matrix itself because it's a matter of scales. just look at mp[0]V[0] in motion library.
		
		#last point given is basically returning to the first, so remove it
		vertices = vertices[0:-1,:]
		
		#vertices come in 2d, here we "inflate" them by 3rd axis = theta = [-pi, pi]
		vneg     = np.hstack((vertices, (theta_center-theta_delta)*np.ones((4,1))))
		vpos     = np.hstack((vertices, (theta_center+theta_delta)*np.ones((4,1))))
		vertices = np.vstack((vpos, vneg))
		self.vertices = vertices.copy()
		
		center   = np.mean(vertices, axis=0)
		
		# half the length of the recatangle edge
		length = np.zeros(3)
		length[2] = 2.0*theta_delta  #theta axis
		length[1] = np.linalg.norm(vertices[0,:]-vertices[1,:]) #"y" axis
		length[0] = np.linalg.norm(vertices[0,:]-vertices[3,:]) #"x" axis
		
		theta_rotated = np.arctan2(vertices[1,1]-vertices[0,1], vertices[1,0]-vertices[0,0])
		theta_rotated -= np.pi/2.0 # to keep the axes, if we get 90deg it means we do not need to rotate the box
		
		#import pdb; pdb.set_trace()
		# since the z direction is always theta and fixed upwards and it is a box,
		# the rotation is only with respect to the z direction
		Rot = np.array([[np.cos(theta_rotated), -np.sin(theta_rotated), 0.], \
					    [np.sin(theta_rotated),  np.cos(theta_rotated), 0.], \
					    [0.,                     0.,                    1.]]) 
		self.b   = fcl.Box(length[0], length[1], length[2])
		self.tf  = fcl.Transform(Rot, center)   # Matrix rotation and translation
		# create the collision object
		self.obj = fcl.CollisionObject(self.b, self.tf)			

class Rectangle(Box):
	def __init__(self, center, L, W):
		# pretty straightforward here, artificially inflate to a 3-d box
		# GUY TODO: think if there's need for rotated boxes as well.
		center = np.hstack((center, 0.0))
		vertices = np.array([[center[0]+L/2., center[1]-W/2.], \
							 [center[0]+L/2., center[1]+W/2.], \
							 [center[0]-L/2., center[1]+W/2.], \
							 [center[0]-L/2., center[1]-W/2.], \
							 [center[0]+L/2., center[1]-W/2.] ])
		Box.__init__(self, vertices, theta_center=0.0, theta_delta=0.01)
		#R = np.eye(3) # change this to true rotation to be able to accept rotated boxes
		#self.b   = fcl.Box(L, W, 0.0)
		#self.tf  = fcl.Transform(R, center)   # Matrix rotation and translation
		#self.obj = fcl.CollisionObject(self.b, self.tf)			
		
def TestIntersectionRectangleEllipse(rect, ellipse):
	return TestIntersectionBoxEllipsoid(rect, ellipse)

		
def TestIntersectionBoxEllipsoid(box, ellipsoid):
	request = fcl.CollisionRequest()
	result = fcl.CollisionResult()
	
	ret = fcl.collide(box.obj, ellipsoid.obj, request, result)
	
	return (ret>0)
	
		
def test():
	# only load when testing

	'''
	n=1000
	e0 = np.random.randn(n,3)
	S = np.zeros((n,9))
	for i in range(n):
		uu = special_ortho_group.rvs(3)
		aa = special_ortho_group.rvs(3)
		teta = np.arctan2(aa[0,1], aa[0,0])
		D = np.zeros(3)
		
		D[0] = 1./(2.0*(np.random.rand(1)+0.5))**2
		D[1] = 1./(2.0*(np.random.rand(1)+0.5))**2
		D[2] = 1./teta**2
		
		S[i,:] = np.dot(np.dot(uu,np.diag(D)),uu.transpose()).flatten()

	vertices = np.array([[10.,  1.], [10.,  11.], [5.,  11.], [5.,  1.], [10.,  1.]])
	'''
	'''
	#e0 = np.array([ 6. ,3. ,-np.pi-1.1 ])
	#S  = np.array([ [1, 0., 0.], [0., 0.2, 0.], [0., 0., 1.] ])
	e0 = np.array([3.13210913, 0.05988775, 0.15797532])
	S  = np.array([[ 0.1579781 , -0.01026418, -0.01524015], \
				   [-0.01026418,  0.13445994,  0.01890003], \
				   [-0.01524015,  0.01890003,  0.19282797]])

	vertices = np.array([[10.,  1.], [10.,  11.], [5.,  11.], [5.,  1.], [10.,  1.]])
	e  = Ellipsoid(e0, S)
		
	b = Box(vertices)
	overlaps = TestIntersectionBoxEllipsoid(b, e)
	'''	
	'''
	e0 = np.array([ 0. , 1.6 ])
	S  = np.array([ [1.0, 0.01], [0.01, 1.0] ])
	center = np.array([0.,0.])
	e  = Ellipse(e0, S)
	b = Rectangle(center, 1., 1.)
	overlaps = TestIntersectionRectangleEllipse(b, e)
	'''
	'''
	results = []
	results_old = []
	tic = timer()
	for i in range(n):
		e  = Ellipsoid(e0[i,:], S[i,:].reshape([3,3]))
		b  = Box(vertices)
		overlaps = TestIntersectionBoxEllipsoid(b, e)
		results.append(overlaps)
	toc = timer()
	print('New Took %.3f[sec] to run' %(toc-tic))
	
	tic = timer()
	for i in range(n):
		e_old = GeometryFunctions.Ellipsoid(e0[i,:], S[i,:].reshape([3,3]))
		b_old = GeometryFunctions.Box(vertices)
		overlaps = GeometryFunctions.TestIntersectionBoxEllipsoid(b_old, e_old)
		results_old.append(overlaps)

	toc = timer()
	print('Old Took %.3f[sec] to run' %(toc-tic))
		
	res = [idx for idx, val in enumerate(results) if val != results_old[idx]] 
	# printing result 
	print("Indices of Non-Zero elements : " + str(res)) 
	if(len(res)>0):
		import pdb; pdb.set_trace()
	'''
	e0 = np.array([-0.99-1./np.sqrt(2.), 1.+1./np.sqrt(2.), 0.0])
	#e0 = np.array([-0.99-1./np.sqrt(2.), 1.+1./np.sqrt(2.)])
	rr = R.from_euler('z', 45, degrees=True)
	#import pdb; pdb.set_trace()
	uu = rr.as_dcm()
	#uu = uu[0:2,0:2]
	D = np.array([1./2.0**2, 1./1.0**2, 1./0.8**2])
	#D = np.array([1./2.0**2, 1./1.0**2])#, 1./0.8**2])
	S = np.matmul(uu, np.matmul(np.diag(D),uu.transpose()))
	#print(uu)
	vertices = np.array([[1.,  -1.], [1.,  1.], [-1.,  1.], [-1.,  -1.], [1.,  -1.]])
	e  = Ellipsoid(e0, S)
	#e  = Ellipse(e0, S)
	b  = Box(vertices)
	#b = Rectangle(np.array([0.0,0.0]), 2, 2)
	overlaps = TestIntersectionBoxEllipsoid(b, e)
	#overlaps = TestIntersectionRectangleEllipse(b, e)
	#import pdb; pdb.set_trace()
	ret = True
	if(overlaps == True):
		print('overlap')
		ret = False
	else:
		print('not overlapping')
		ret = True
		
	#plot_be(b, e, ret)
	#xbar = e0 - np.array([0.0, 0.5])
	#xbar = e0 - np.array([0.0, 0.5])
	#print('my test found %d' %(xbar.dot(S.dot(xbar)) <=1 ))
	
	return ret
	
		
if __name__ == "__main__":
	import GeometryFunctions
	from scipy.stats import special_ortho_group
	from scipy.spatial.transform import Rotation as R

	tic = timer()
	for i in range(1):
		test()
	toc = timer()
	print('Took %.3f[sec] to run' %(toc-tic))

	
	
	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		