from sympy import *

q1, q2, q3, q4, q5, q6 = symbols('q1 q2 q3 q4 q5 q6')

d1, a2, a3, d4, d5, d6 = symbols('d1 a2 a3 d4 d5 d6')

s1 = sin(q1) 
c1 = cos(q1)

q234 = q2+q3+q4 
s2 = sin(q2)
c2 = cos(q2)
s3 = sin(q3)
c3 = cos(q3)
s5 = sin(q5)
c5 = cos(q5)
s6 = sin(q6)
c6 = cos(q6) 
s234 = sin(q234)
c234 = cos(q234)

nx = ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0

ox = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - 
          (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0)
          
ax = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - 
          s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0))
          
px = ((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - 
          d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - 
          a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3)        
          
ny = c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0

oy = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + 
          s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0))
          
ay = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - 
          s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0))          
          
py = ((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + 
          (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - 
          a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3)
          
nz = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0)

oz = ((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6)

az = (s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0)

pz = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - 
         (d6*(c234*c5+s234*s5))/2.0 - d5*c234)
         
##########
         
tx1 = diff(px, q1)
tx2 = diff(px, q2)
tx3 = diff(px, q3)
tx4 = diff(px, q4)
tx5 = diff(px, q5)
tx6 = diff(px, q6)

ty1 = diff(py, q1)
ty2 = diff(py, q2)
ty3 = diff(py, q3)
ty4 = diff(py, q4)
ty5 = diff(py, q5)
ty6 = diff(py, q6)

tz1 = diff(pz, q1)
tz2 = diff(pz, q2)
tz3 = diff(pz, q3)
tz4 = diff(pz, q4)
tz5 = diff(pz, q5)
tz6 = diff(pz, q6)

#########

wx = -asin(nz)
wy = atan2(oz/cos(px),az/cos(px))
wz = atan2(ny/cos(px),nx/cos(px))

#########

rx1 = diff(wx, q1)
rx2 = diff(wx, q2)
rx3 = diff(wx, q3)
rx4 = diff(wx, q4)
rx5 = diff(wx, q5)
rx6 = diff(wx, q6)

ry1 = diff(wy, q1)
ry2 = diff(wy, q2)
ry3 = diff(wy, q3)
ry4 = diff(wy, q4)
ry5 = diff(wy, q5)
ry6 = diff(wy, q6)

rz1 = diff(wz, q1)
rz2 = diff(wz, q2)
rz3 = diff(wz, q3)
rz4 = diff(wz, q4)
rz5 = diff(wz, q5)
rz6 = diff(wz, q6)

