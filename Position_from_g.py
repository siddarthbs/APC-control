import math as m
import numpy as np
from numpy import *

def rotate(p,q):
    # initial position vector
    v = np.matrix([[0], 
               [0], 
               [0.3]])
    # Euler angles from the g values
    # CCW +ve, CW -ve
    a = m.asin(q/9.8)
    b = m.asin(p/9.8)
    # Rotation matrix
    R = np.matrix([[           (m.cos(b)),         0,               m.sin(b)],
              [ (-m.sin(a))*(m.sin(b)),  m.cos(a), (m.sin(a))*(m.cos(b))],
              [(-m.sin(b))*(m.cos(a)),  -m.sin(a), (m.cos(a))*(m.cos(b))]])
    new_v = R*v
    #print v
    #print R
    new_v = np.transpose(new_v)
    return new_v

new_position = rotate(4.4,2.4)
print 'New co-ordinates are'
print new_position
