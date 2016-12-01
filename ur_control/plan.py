import rospy
from std_msgs.msg import String
from sensor_msgs.msg  import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped

import interpolate

from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
from ur_kin_py.kin import Kinematics
kin = Kinematics('ur5') # or ur10


s = kin.forward([1.57,-0.15,-1.57,0,0,0])
f = kin.forward([0,-0.15,-1.57,0,0,0])

xs = s[0:3,3]
xf = f[0:3,3]

m = kin.forward([0.]*6)

def plan(xs, xf, v=[0,0,0], t=10):
	
	n = 100
	
	global m
	
	global kin
			
	fn = interpolate.plan(xs, xf, v,t)
	
	points = fn(np.arange(0,t,t*1./n))
	
	
	pub=rospy.Publisher('/arm_controller/command',JointTrajectory,queue_size=3)
	
	msg = JointTrajectory()
	msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		
	#print(points)	
		
	for i in range(len(points)):
		p = points[i]
		
		m[0:3,3] = p.reshape(3,1)
		point = JointTrajectoryPoint()
		point.positions = kin.inverse(m)
		
		if(not hasattr(point.positions, "__len__")):
			continue
		point.time_from_start = rospy.Duration(0.5+t*(i+1.)/n)
		msg.points.append(point)
	#print(msg)
	rate = rospy.Rate(10)
	rospy.sleep(0.2)
	pub.publish(msg)
	rate.sleep()
	#rospy.spin()
	return True

def plan_js(js, jf, v=[0,0,0], t=10):
	s = kin.forward(np.array(js).reshape(6))
	f = kin.forward(np.array(jf).reshape(6))
	
	xs = s[0:3,3]
	xf = f[0:3,3]
	global m
	m = kin.forward(np.array(js).reshape(6))
	return plan(xs,xf, v, t)

#rospy.init_node('listener', anonymous=True)

	
#plan(xs,xf)

"""
x = np.linspace(0, 3, num=4, endpoint=True)
y = np.array([5,1,2,5])
f = interp1d(x, y)
f2 = interp1d(x, y, kind='cubic')

xnew = np.linspace(0, 3, num=41, endpoint=True)

plt.plot(x, y, 'o', xnew, f(xnew), '-', xnew, f2(xnew), '--')
plt.legend(['data', 'linear', 'cubic'], loc='best')
plt.show()
"""

