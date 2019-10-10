import numpy as np
import matplotlib.pyplot as plt
import math
import pickle

from shapely.geometry import Polygon, box

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
	
	