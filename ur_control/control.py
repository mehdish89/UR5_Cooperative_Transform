#!/usr/bin/env python

from jacobian import *
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg  import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
import tf
import os
import plan



import numpy as np


count = 0


def state(data):
    #rospy.loginfo(data.effort)
    
    global position

    position = np.array(data.position[:]).reshape(6,1)   
    effort = list(data.effort[:])
    
    tmp = position[0]
    position[0] = position[2]
    position[2] = tmp
    
    tmp = effort[0]
    effort[0] = effort[2]
    effort[2] = tmp
    
    T = np.array(effort).reshape(6,1)
    
    global J
    
    J = jacobian(position)
      
  
done = False    
mF = np.zeros(3).reshape(3,1)
hF = np.zeros(3).reshape(3,1)

def callback(data):
    
    global count
    global mF 
    global hF         
    global listener
    global done
    
    (t,r) = listener.lookupTransform('/base', '/wrist_3_link', rospy.Time(0))
    M = listener.fromTranslationRotation(t,r)
    
    M = M[0:3,0:3]
    
    p = data.wrench.force
    x = np.array([p.x, p.y, p.z]).reshape(3,1)
    
    F = np.dot(M, x)
    F[0] = -F[0]
    F[1] = -F[1]
    F[2] += 1.84203854
    
    
    kF = mF - hF
    
    ws = WrenchStamped()
    ws.header.frame_id = '/wrist_3_link'
    ws.wrench.force.x = kF[0]/100.
    ws.wrench.force.y = kF[1]/100.
    ws.wrench.force.z = kF[2]/100.
    
    ws.wrench.torque.x = 0
    ws.wrench.torque.y = 0
    ws.wrench.torque.z = 0
    
    global fpub
    
    fpub.publish(ws)
    
    #global J
    #global dq
    #global position
    
    #Jinv = np.linalg.pinv(J)
    
         
    alpha = 0.98
    omF = mF
    mF = (mF*alpha + (1-alpha)*(F*100/10))
    nF = np.linalg.norm(mF)
    
    beta = 0.91
    hF = beta*hF+(mF-omF)*beta
    nkF = np.linalg.norm(kF)
    
    v = 0.1*kF/10#nF
      
    dF = np.linalg.norm(mF-omF)
   	
                
    if(count%10==0):
    	os.system('clear') 
    	if nkF>10 and not done:
			a=1
			#mF = mF*0
			if(plan.plan_js(position, [0.5, 0.15, -1.5, 0, 0, 0], v, 10)):
				mF = mF*0
				hF = hF*0
				print('planned successfully')
				print(nkF)
				#done = True
			#	rospy.sleep(2)
			else:
				print('couldnt find the plan')
    
    
    	
    	print(kF)
        print('++++++')
        print(np.floor(nkF))
        print('++++++')
        print(np.floor(dF))
    count+=1
    
def listener():
   
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/ft_sensor_topic", WrenchStamped, callback)
    
    rospy.Subscriber("/joint_states", JointState, state)
    
    global listener
    listener = tf.TransformListener()
    
    while len(listener.getFrameStrings())==0:
	    print('not ready')
	
	#pub = rospy.Publisher('/arm_controller/command', JointTrajectory)
    rate = rospy.Rate(10) # 10hz
    global fpub
    fpub=rospy.Publisher('/endeffector_force',WrenchStamped)
    pub=rospy.Publisher('/arm_controller/command',JointTrajectory)
    
    global position
    global dq
    
    #rate.sleep(1)
    #plan.plan_js(position, [0,0,0,0,0,0])
	
    #while not rospy.is_shutdown():
		
		
		#if(a>0):
		#	pub.publish(msg)		
	#	rate.sleep()
		
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
