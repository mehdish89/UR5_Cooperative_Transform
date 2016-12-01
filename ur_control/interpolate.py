import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


def interp(p, t=[], kind='quadratic'):
	
	rows = len(p)
	cols = len(p[0])
	
	if len(t)==0:
		t = range(rows)
	
	#print(p)
	#print('####')
	
	q = np.array(p)
	
	
	
	f = []
	
	for i in range(cols):
		Xi = q[:,i]
		
		#print(Xi)
		#print('####')
		
		fi = interpolate.interp1d(t, Xi, kind=kind)
		f.append(fi)
	
	def func(time):
		vec = []
		for fx in f:
			vec.append(fx(time))
		return np.array(vec)
	
	def func_list(ts):
		if(isinstance(ts, list) or isinstance(ts, np.ndarray)):
			pts = []
			for t in ts:
				pts.append(func(t))
			return np.array(pts)
		else:
			return func(ts)
	return func_list
	
def plan(p, q, v, t):
	p = np.array(p).reshape(3)
	q = np.array(q).reshape(3)
	v = np.array(v).reshape(3)
	dt = 0.01
	dx = v*dt
			
	points = [p, p+dx, q]
	times = [0, dt, t]
	
	return interp(points, times)
	


#f = plan([0,0],[10,0], [5,1], 10)

#fn = f(np.arange(0,10,0.1))

#x = fn[:,0]
#y = fn[:,1]

#plt.plot(x,y)
#plt.show()
