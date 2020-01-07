from matplotlib.pyplot import *
import numpy as np
import csv

data = []
with open('calibration.log', 'rb') as f:
	data = np.loadtxt(f,delimiter=',')
    #reader = csv.reader(f)
    #for row in reader:
    #    data.append(row)

#import pdb; pdb.set_trace()
figure()
plot(data[:,2], -data[:,1])
figure()
plot(data[:,0], -data[:,1], label='x')
plot(data[:,0], data[:,2], label='y')
plot(data[:,0], data[:,3], label='$\Theta$')
legend()
show()

