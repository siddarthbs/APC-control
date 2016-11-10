#arm 1 (arm a) has its base at (  0    ,0 ,0)
#	arm 2 (arm b) has its base at (- 0.51 ,0 ,0)

#The Kinect is located at (0.04, -0.28, 0.66) and points along its own x axis
#To go from Kinect to World frame, rotate -90 ~ 135 degrees in the x followed by a -45 degree rotation about the z axis 
#p_0 = point in World frame
d_10 = np.matrix([[0.04],[-0.28],[0.53]])

#Therefore
R_10 = np.matrix([[np.cos(psi) , np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(psi)],\
		  [-np.sin(psi), np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(psi)],\
		  [0           ,-np.sin(phi)            , np.cos(phi)            ]])
#R_01 = R_10 transposed so
R_01 = np.matrix([[np.cos(psi)	          ,-np.sin(psi)	           , 0	        ],\
	          [np.cos(phi)*np.sin(psi), np.cos(phi)*np.cos(psi),-np.sin(phi)],\
	          [np.sin(phi)*np.sin(psi), np.sin(phi)*np.cos(psi), np.cos(phi)]])
#and
psi = np.pi/4
phi = np.pi/2 + np.pi/12
H_10 = np.matrix([[ np.cos(psi), np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(psi), 0.04],\
	          [-np.sin(psi), np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(psi),-0.28],\
	          [ 0	       ,-np.sin(phi)	        , np.cos(phi)		 , 0.53],\
		  [ 0	       , 0                      , 0                      , 1   ]])

p_0 = H_10 * p_1
#which means
d_01 = -R_01 * d_10

#and
H_01 = np.matrix([[np.cos(psi)	          ,-np.sin(psi)	           , 0          , d_01[0]],\
	          [np.cos(phi)*np.sin(psi), np.cos(phi)*np.cos(psi),-np.sin(phi), d_01[1]],\
	          [np.sin(phi)*np.sin(psi), np.sin(phi)*np.cos(psi), np.cos(phi), d_01[2]],\
	          [0                      , 0                      , 0          , 1     ]])
