
import numpy as np
import math
from itertools import combinations
from timeit import default_timer as timer

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # This import registers the 3D projection, but is otherwise unused

class Ellipse:
	def __init__(self, center, M):
		# pretty straightforward here
		self.center = center.copy()
		
		self.M      = M
		self.invM   = np.linalg.inv(M)
		
		w, v = np.linalg.eig(M)
		self.length = 1./np.sqrt(w)
		self.axis   = v.T
		
		
class Ellipsoid:
	def __init__(self, center, M):
		self.center = center.copy()
		# wrap the angle between (-pi,+pi)
		#self.center[2] = (( -self.center[2] + math.pi) % (2.0 * math.pi ) - math.pi) * -1.0
		
		
		# find the rotation matrix and radii of the axes
		u, s, vh = np.linalg.svd(M)
		radii = 1.0/np.sqrt(s[2])

		# sometimes the optimization gives a really small number in the M[2,2] coefficient
		# quick fix for badly scaled matrices. need to fix it in future
		if( radii > math.pi ):
			radii = 45.0*math.pi/180.0
			s[2] = 1.0/(radii**2)
		
		smat = np.diag(s)
		M    = np.dot(u, np.dot(smat, vh))
		
		self.M      = M
		self.invM   = np.linalg.inv(M)
		
		w, v = np.linalg.eig(M)
		self.length = 1./np.sqrt(w)
		self.axis   = v.T
		
class Rectangle:
	def __init__(self, center, L, W):
		self.center, self.axis, self.length = self.__GetParams__(center, L, W)

	#vertices come in 2d
	def __GetParams__(self, center, L, W):
		# define the box given the center and length and width of it
		self.vertices = np.array([[center[0]+L/2.0, center[1]-W/2.0], \
								  [center[0]+L/2.0, center[1]+W/2.0], \
								  [center[0]-L/2.0, center[1]+W/2.0], \
								  [center[0]-L/2.0, center[1]-W/2.0] ])
	
		axis = np.zeros((2, 2))
		axis[:, 1] = self.__GetNormal__(self.vertices[1,:], self.vertices[2,:])
		axis[:, 0] = self.__GetNormal__(self.vertices[0,:], self.vertices[1,:])
		
		# half the length of the recatangle edge
		length = np.zeros(2)
		length[1] = W/2
		length[0] = L/2
		
		return center, axis, length
	
	# ax+by+cz+d=0
	def __GetNormal__(self, p1, p2):
	
		# find slope m, the perpendicular is -1/m and return normalized vector
		if( abs(p1[0]-p2[0]) <= 1e-4 ):
			# avoid division by zero
			return np.array([1.0, 0.0])
		elif( abs(p1[1]-p2[1]) <= 1e-4):
			# avoid division by zero
			return np.array([0.0, 1.0])
		else:
			m = (p1[1]-p2[1]) / (p1[0]-p2[0])
			m_perp = - 1.0 / m
			n = np.array([1.0, m_perp])
			return n/np.linalg.norm(n)

class Box:
	def __init__(self, vertices):
		self.center, self.axis, self.length = self.__GetParams__(vertices)

	#vertices come in 2d, here we "inflate" them by 3rd axis = theta = [-pi, pi]
	def __GetParams__(self, vertices):
		#convert to 3d
		vertices = vertices[0:-1,:]
		#import pdb; pdb.set_trace()
		vneg     = np.hstack((vertices, -2.0*math.pi*np.ones((4,1))))
		vpos     = np.hstack((vertices, +2.0*math.pi*np.ones((4,1))))
		vertices = np.vstack((vpos, vneg))
		self.vertices = vertices
		
		center   = np.mean(vertices, axis=0)
		
		axis = np.zeros((3, 3))
		axis[:, 2] = self.__GetNormal__(vertices[3,:], vertices[2,:], vertices[1,:])
		axis[:, 1] = self.__GetNormal__(vertices[1,:], vertices[2,:], vertices[6,:])
		axis[:, 0] = self.__GetNormal__(vertices[0,:], vertices[1,:], vertices[5,:])
		
		# half the length of the recatangle edge
		length = np.zeros(3)
		length[2] = 2.0*math.pi
		length[1] = np.linalg.norm(vertices[0,:]-vertices[1,:])/2.0
		length[0] = np.linalg.norm(vertices[0,:]-vertices[3,:])/2.0
		
		return center, axis, length
	
	# ax+by+cz+d=0
	def __GetNormal__(self, p1, p2, p3):
		# These two vectors are in the plane
		v1 = p3 - p1
		v2 = p2 - p1

		# the cross product is a vector normal to the plane
		cp = np.cross(v1, v2)
		a, b, c = cp
		
		# This evaluates a * x3 + b * y3 + c * z3 which equals d
		#d = -np.dot(cp, p3)
		
		n = np.array([a,b,c])
		
		return n/np.linalg.norm(n)
				

def TestIntersectionRectangleEllipse(rect, ellipse):
	L = np.zeros(2)
	
	# computes the increase in extents for R'
	for i in range(2):
		L[i] = np.sqrt( np.dot( rect.axis[:,i], np.dot( ellipse.invM, rect.axis[:,i].reshape((-1,1)) ).reshape((2,)) ) )
		
	# Transform ellipse center to rect coord.
	KmC = ellipse.center - rect.center
	xi  = np.array([ np.dot(rect.axis[:,0], KmC.reshape((-1,1))), \
					 np.dot(rect.axis[:,1], KmC.reshape((-1,1))) ])
	
	if ( (np.abs(xi[0]) <= rect.length[0] + L[0]) and \
		 (np.abs(xi[1]) <= rect.length[1] + L[1]) ):
		
		s = np.array([ 1.0 if xi[0]>=0.0 else -1.0, 1.0 if xi[1]>=0.0 else -1.0 ])
		PmC = s[0]*rect.length[0]*rect.axis[:,0] + s[1]*rect.length[1]*rect.axis[:,1]
		Delta = KmC-PmC
		MDelta = np.dot(ellipse.M, Delta.reshape((-1,1)))
		for i in range(2):
			if(s[i]*np.dot(rect.axis[:,0], MDelta)[0] <= 0):
				#print('less than zero')
				return True
		#print('got to questainable line')
		return np.dot(Delta, np.dot(ellipse.M, Delta))
	
	#print('reached end')
	return False

		
def TestIntersectionBoxEllipsoid(box, ellipsoid):
	L = np.zeros(3)
	
	# computes the increase in extents for B'
	for i in range(3):
		L[i] = np.sqrt( np.dot( box.axis[:,i], np.dot( ellipsoid.invM, box.axis[:,i].reshape((-1,1)) ).reshape((3,)) ) )
		
	# Transform ellipsoid center to box coord.
	KmC = ellipsoid.center - box.center
	xi  = np.array([ np.dot(box.axis[:,0], KmC.reshape((-1,1))), \
					 np.dot(box.axis[:,1], KmC.reshape((-1,1))), \
					 np.dot(box.axis[:,2], KmC.reshape((-1,1))) ])
	
	if ( (np.abs(xi[0])> box.length[0] + L[0]) or \
		 (np.abs(xi[1])> box.length[1] + L[1]) or \
		 (np.abs(xi[2])> box.length[2] + L[2]) ):
		# K is outside of box B'
		return False
	
	s = np.array([ 1.0 if xi[0]>=0.0 else -1.0, 1.0 if xi[1]>=0.0 else -1.0, 1.0 if xi[2]>=0.0 else -1.0 ])
	PmC = s[0]*box.length[0]*box.axis[:,0] + s[1]*box.length[1]*box.axis[:,1] + s[2]*box.length[2]*box.axis[:,2]
	Delta = KmC-PmC
	MDelta = np.dot(ellipsoid.M, Delta.reshape((-1,1)))
	
	rsqr = np.zeros(3)
	#import pdb; pdb.set_trace()
	for i in range(3):
		r = np.dot( ellipsoid.axis[:,i], Delta.reshape((-1,1)) )[0] / ellipsoid.length[i]
		rsqr[i] = r*r
	
	UMD = np.array([ np.dot(box.axis[:,0], MDelta)[0], \
					 np.dot(box.axis[:,1], MDelta)[0], \
					 np.dot(box.axis[:,2], MDelta)[0] ])
	UMU = np.zeros((3,3))
	for row in range(3):
		prod = np.dot(ellipsoid.M, box.axis[:,row].reshape((-1,1)))
		for col in range(3):
			UMU[row, col] = np.dot(box.axis[:,col], prod)[0]
	
	if(s[0]*(UMD[0]*UMU[2,2]-UMU[0,2]*UMD[2]) > 0 and \
	   s[1]*(UMD[1]*UMU[2,2]-UMU[1,2]*UMD[2]) > 0 and \
	   rsqr[0] + rsqr[1] > 1):
		#K is outside the elliptical cylinder (P,U2)
		return False
	
	if(s[0]*(UMD[0]*UMU[1,1]-UMU[0,1]*UMD[1]) > 0 and \
	   s[2]*(UMD[2]*UMU[1,1]-UMU[1,2]*UMD[1]) > 0 and \
	   rsqr[0] + rsqr[2] > 1):
		#K is outside the elliptical cylinder (P,U1)
		return False
	
	if(s[1]*(UMD[1]*UMU[0,0]-UMU[0,1]*UMD[0]) > 0 and \
	   s[2]*(UMD[2]*UMU[0,0]-UMU[0,2]*UMD[0]) > 0 and \
	   rsqr[1] + rsqr[2] > 1):
		#K is outside the elliptical cylinder (P,U0)
		return False
	
	if(s[0]*UMD[0] > 0 and s[1]*UMD[1] > 0 and s[2]*UMD[2] > 0 and np.sum(rsqr) > 1):
		#K is outside the ellipsoid at P
		return False
	
	return True
		

def plot_be(box, ellipsoid, overlap): 
	A = ellipsoid.M
	center = ellipsoid.center

	# find the rotation matrix and radii of the axes
	U, s, rotation = np.linalg.svd(A)
	radii = 1.0/np.sqrt(s)

	# now carry on with EOL's answer
	u = np.linspace(0.0, 2.0 * np.pi, 100)
	v = np.linspace(0.0, np.pi, 100)
	x = radii[0] * np.outer(np.cos(u), np.sin(v))
	y = radii[1] * np.outer(np.sin(u), np.sin(v))
	z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
	for i in range(len(x)):
		for j in range(len(x)):
			[x[i,j],y[i,j],z[i,j]] = np.dot([x[i,j],y[i,j],z[i,j]], rotation) + center

	# plot ellipsoid
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color='b', alpha=0.2)
	
	#plot box
	for s, e in combinations(box.vertices, 2):
		if(np.sum(s==e) == 2):
			ax.plot3D(*zip(s, e), color="b")
	
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('$\Theta$')
	
	if(overlap):
		plt.title('Not intersecting')		
	else:
		plt.title('Intersecting')
	ax.set_aspect("equal")
	plt.show() #block = False)
		
		
def test():
	
	#e0 = np.array([ 6. ,3. ,-math.pi-1.1 ])
	e0 = np.array([3.13210913, 0.05988775, 0.15797532])
	#S  = np.array([ [1, 0., 0.], [0., 0.2, 0.], [0., 0., 1.] ])
	S  = np.array([[ 0.1579781 , -0.01026418, -0.01524015], \
				   [-0.01026418,  0.13445994,  0.01890003], \
				   [-0.01524015,  0.01890003,  0.19282797]])
	vertices = np.array([[10.,  1.], [10.,  11.], [5.,  11.], [5.,  1.], [10.,  0.]])
	e  = Ellipsoid(e0, S)
	
	b = Box(vertices)
	overlaps = TestIntersectionBoxEllipsoid(b, e)
	'''
	e0 = np.array([ 0. , 1.6 ])
	S  = np.array([ [1.0, 0.01], [0.01, 1.0] ])
	center = np.array([0.,0.])
	e  = Ellipse(e0, S)
	b = Rectangle(center, 1., 1.)
	overlaps = TestIntersectionRectangleEllipse(b, e)
	'''
	
	ret = True
	if(overlaps == True):
		print('overlap')
		ret = False
	else:
		print('not overlapping')
		ret = True
		
	plot_be(b, e, ret)
	#xbar = e0 - np.array([0.0, 0.5])
	#print('my test found %d' %(xbar.dot(S.dot(xbar)) <=1 ))
	
	return ret
	
		
if __name__ == "__main__":
	tic = timer()
	#for i in range(1000):
	test()
	toc = timer()
	print('Took %.3f[sec] to run' %(toc-tic))

	
	
	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		