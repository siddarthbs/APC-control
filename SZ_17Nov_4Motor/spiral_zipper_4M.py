import numpy as np
#import algopy as ap
# from algopy import UTPM
from numpy import pi
import serial
import os
from time import time as now, sleep
import math as m
import matplotlib.pyplot as plt
import datetime
import slack_removal as slack

class SpiralZipper:

	def __init__(sz, start_position, r_winch, c, looptime, armflag):
			
		#Lengths set the initial configuration of the system.
		# Lengths: array 1x4 [L0, L1, L2, L3]
		#create spiral zipper object
		#physical notes
		#sz ckbot direction: CCW subtracts tether and CW adds/releases tether.
		sz.startposition = start_position
		sz.start_detected = False
		sz.entered = False
		sz.tether_subtract_CCW = True
		sz.looptime = looptime  #control speed to be enforced
		sz.timeold = 0
		sz.timeold2 = 0

		sz.goal_prev = start_position
		sz.goal_start = now()
		sz.goal_stop =now()
		sz.target_achieved = [0, 0]
		sz.target_reached = False
		sz.P = 10*np.eye(2) #System covariance. Trust in initial Process Model Conditions
		sz.Q = 0.01*np.eye(2) #System noise Covariance. What we think the initial process noise is
		sz.R = 1*np.eye(2) #Sensor Noise Covariance. What we think of the sensor noise is. 

		got_port = [1, 1, 1]

		ser = [0, 0, 0]

		try:
			ser[0] = serial.Serial('/dev/ttyACM0', 57600)
		except Exception, e:
			print "No Arduino on ACM0"
			print str(e)
			got_port[0] = 0
		try:
			ser[1] = serial.Serial('/dev/ttyACM1', 57600)
		except Exception, e:
			print "No Arduino on ACM1"
			print str(e)
			got_port[1] = 0
		try:
			ser[2] = serial.Serial('/dev/ttyACM2', 57600)
		except Exception, e:
			print "No Arduino on ACM2"
			print str(e)
			got_port[2] = 0

		for port in range(3):
			if got_port[port]:
				figuredout_LIDAR1 = False
				figuredout_LIDAR2 = False
				figuredout_GYRO = False

				while (not figuredout_LIDAR1 and not figuredout_LIDAR2 and not figuredout_GYRO):
					bytesToRead = ser[port].inWaiting()
					readings = ser[port].read(bytesToRead)
					for t in reversed(readings.split()):  # read from most recent serial data
						if (t is ']' or t is '[' or t is '}' or t is '{'):
							sz.ser_gyro = ser[port]
							figuredout_GYRO = True
							print "Port %d is GYRO" % port
							break
						elif (t is '<' or t is '>'):
							sz.ser_lidar1 = ser[port]
							figuredout_LIDAR1 = True
							print "Port %d is LIDAR1" % port
							break
						elif(t is '#' or t is '$'):
							sz.ser_lidar2 = ser[port]
							figuredout_LIDAR2 = True
							print "Port %d is LIDAR2" % port
							break

						else:
							print "Undecided..."
			else:
				print "Nothing on port %d" % port


		'''except serial.SerialException:
		sz.ser = serial.Serial('/dev/ttyACM1', 9600)'''
		sleep(1)
		# set params for ckbot control
		sz.max_t = 65535
		sz.min_t = 0
		sz.limit_t = sz.max_t/2
		sz.radian_const = 0.06*np.pi/180

		#init position of tether sources
		sz.rb = 0.225           # circle inscribing the spiral zipper motors #.22 for the APC arm, .11 for the RCTA arm
		sz.rt = 0.0695  	       # diameter of the spiral zipper column
		sz.r_winch = r_winch   # the radius of the winch used to wind tether.

		#phi = pi/6  #rad
		phi = 28.81 * pi/180

		p1 = (sz.rb)*np.array([0,1, 0]) + np.array([0,0,.1119])  #positions of the motors in xyz
		p2 = sz.rb*np.array([np.cos(phi),-np.sin(phi), 0]) + np.array([0,0,.1119])
		p3 = sz.rb*np.array([-np.cos(phi),-np.sin(phi), 0]) + np.array([0,0,.1119])
		sz.p = [np.array(start_position),p1,p2,p3]

		sz.L = [0,0,0,0]
		sz.L_vel_desired = [0,0,0,0]
		sz.L_vel_actual = [0,0,0,0]
		sz.errorsum = [0,0,0,0]	

		#get ckbot's current rotation position for each Ckbot
		if armflag == 0:
			theta1 = c.at.T1.get_pos()
			theta2 = c.at.T2.get_pos()
			theta3 = c.at.T3.get_pos()
			theta4 = c.at.T7.get_pos()
			sz.theta0 = [theta1,theta2,theta3,theta4]
			sz.L[0] = sz.sensed_lidar(armflag)
			#sz.L[0] = sz.get_lidar1_readings() #takes one lidar reading directly
			sz.lold = [0,0,0]
			sensed_grav = sz.get_sensor_readings(armflag) #takes the average of a set of lidar readings
		elif armflag == 1:
			print "getting arm 2 readings"
			theta1 = c.at.T4.get_pos()
			theta2 = c.at.T5.get_pos()
			theta3 = c.at.T6.get_pos()
			theta4 = c.at.T8.get_pos()
			sz.theta0 = [theta1,theta2,theta3,theta4]
			print "got encoder values"
			sz.L[0] = sz.sensed_lidar(armflag)
			print "got Lidar vals"			
			#sz.L[0] = sz.get_lidar2_readings() #takes one lidar reading directly
			sz.lold = [0,0,0]			
			sensed_grav = sz.get_sensor_readings(armflag) #takes the average of a set of IMU readings
			print "got IMU vals"
		sz.theta_prev = sz.theta0 # used to update delta position
		sz.theta_curr = sz.theta0 # will hold the current theta of the system

		#First LIDAR Readings
		#sz.L[0] = sz.get_lidar1_readings()
		#zipper2 = sz.get_lidar2_readings()
		sz.L0_desired = sz.L[0]

		ef1 = sz.rt*np.array([0,1, 0]) #tether attachment points on the end effector assuming arm is straight up
		ef2 = sz.rt*np.array([np.cos(pi/6),-np.sin(pi/6), 0])
		ef3 = sz.rt*np.array([-np.cos(pi/6),-np.sin(pi/6), 0])
		sz.ef = [np.array(start_position),ef1,ef2,ef3]
		print "end effector cable positions are: "
		print sz.ef[1]
		#First predicted position is based on initial sensor reading
		
		#sz.sensed_pos = sz.rotate(sensed_grav[0], sensed_grav[1],sz.L[0])
		sz.sensed_pos = sz.rotate(sensed_grav[1], sensed_grav[2],sz.L[0])

		sz.L = sz.cart2tether_actual(sz.sensed_pos)
		sz.L_old = sz.L
		print "sensed_accelerations "
		print sensed_grav
		print "Sensed_Position "
		print sz.sensed_pos
		print "Alternate sensed pos 1 "
		print sz.get_xyz_pos()
		print "Tether_Length "
		print sz.L

		sz.goal = sz.sensed_pos # init goal as initial length
		#fixed column goal limit
		sz.goal_max_fixed_col = np.cos(pi/5)*sz.L[0] #limit to 60 degrees away

		#sz.data_store_predict = open("SZ_DATA_predict.txt", "w")
		#sz.data_store_measurement = open("SZ_DATA_measurement.txt", "w")
		#sz.data_store_estimate = open("SZ_DATA_estimate.txt", "w")
		#if armflag == 0:
		slack.slack_remove(sz,c, 0)
		#if armflag == 1:
			#slack.slack_remove(sz,c, 1)


	def vacuum_cleaner(sz):  #sends a command to the arduino that controls the vacuum.  Toggles it on and off
		sz.ser_lidar2.write("r")

	def slack_removal(sz,c):
		slack.slack_remove(sz,c, 0)

	def set_Kp_Gains(sz, Kp, remote_or_auto):
		if remote_or_auto == 0:
			sz.Kp_remote = Kp
		if remote_or_auto == 1:
			sz.Kp = Kp

	def set_Ki_Gains(sz, Ki, remote_or_auto):
		if remote_or_auto == 0:		
			sz.Ki_remote = Ki
		if remote_or_auto == 1:
			sz.Ki = Ki

	def get_Kp_Gains(sz, remote_or_auto):
		if remote_or_auto == 0:
			return sz.Kp_remote
		if remote_or_auto == 1:
			return sz.Kp

	def get_Ki_Gains(sz, remote_or_auto):
		if remote_or_auto == 0:
			return sz.Ki_remote
		if remote_or_auto == 1:
			return sz.Ki

	def get_goal(sz): 	#get goal information
		return sz.goal

	def update_state (sz, c,armflag):
		#This function updates the position of the system based on encoder data and the IMU.
		if armflag ==0:		
			theta1 = c.at.T1.get_pos()
			theta2 = c.at.T2.get_pos()
			theta3 = c.at.T3.get_pos()
			theta4 = c.at.T7.get_pos()
			#theta_reading = [theta1,theta2,theta3]
			theta_reading = [theta1,theta2,theta3,theta4]
		if armflag ==1:		
			theta1 = c.at.T4.get_pos()
			theta2 = c.at.T5.get_pos()
			theta3 = c.at.T6.get_pos()
			theta4 = c.at.T8.get_pos()
			#theta_reading = [theta1,theta2,theta3]
			theta_reading = [theta1,theta2,theta3,theta4]
		#print "motor theta readings:"
		#print theta_reading

		#update current theta
		sz.theta_curr = theta_reading
		#calculate the change in position since last call
		dtheta = sz.get_CR_delta() # see method for sign convention
		#update previous theta
		sz.theta_prev = sz.theta_curr

		#calculate the new state of each tether
		# since CCW delta is assumed positive we check how the tether
		# grows in the CCW directiatan(.1674/.3139) *180/pi. if it subtracts we take the negative
		# of the delta to change signs
		rotation=[]
		rotation = sz.get_sensor_readings(armflag)
		if armflag ==0:
			print "IMU data is : "
			print rotation		
		sz.L[0] = sz.L[0] - dtheta[0] * 0.00955 #0.06/(2*3.14159)  #change in column height is 16 cm/rev of the column. 2.666 motor revs/column
		sz.sensed_pos = sz.rotate(rotation[0],rotation[1],sz.L[0])
		#sz.sensed_pos = sz.rotate(rotation[1],rotation[2],sz.L[0])

		sz.L = sz.cart2tether_actual(sz.sensed_pos)  #tether length update based on IMU only

		#sz.L[0] = sz.L[0] - dtheta[0] * 0.00955 #0.06/(2*3.14159)  #change in column height is 16 cm/rev of the column. 2.666 motor revs/column rev.  .06 cm/motor revs constants convert units to [cm/rad]
		#sz.L[1] = sz.L[1] - dtheta[3]*sz.r_winch
		#sz.L[2] = sz.L[2] - dtheta[1]*sz.r_winch  # tether length update based on encoder data only
		#sz.L[3] = sz.L[3] - dtheta[2]*sz.r_winch  # local update

		sz.L_old = sz.L
		sz.L_vel_actual[0] = (-dtheta[0] * 0.00955)    / sz.looptime #0.06/(2*3.14159))/sz.looptime #gets the current speed of the system
		sz.L_vel_actual[1] = (-dtheta[3] * (sz.r_winch)) / sz.looptime
		sz.L_vel_actual[2] = (-dtheta[1] * sz.r_winch) / sz.looptime
		sz.L_vel_actual[3] = (-dtheta[2] * sz.r_winch) / sz.looptime

	def get_CR_delta(sz):
		#compare previous value and current value for modules
		# returns the delta in radians assums CCW is positive and CCW is defined facing the servo front
		#get current readings
		#Assume the current value of the sensor is stored and the previous value is correct.
		diff = [0.0,0.0,0.0,0.0]
		#diff = [0.0, 0.0]
		for i in range(4):
			diff[i] = sz.theta_curr[i]-sz.theta_prev[i]
			if abs(diff[i])>sz.limit_t: #calculate valid delta and switch signs to be correct direction
				if sz.theta_curr[i] >= sz.theta_prev[i]:# diff is +ve, so the solution should be -ve
					diff[i] = ((sz.max_t-sz.theta_curr[i])+(sz.theta_prev[i]-sz.min_t)+1)*(-1)
				else: # diff is negative, therfore the solution should be positive
					diff[i] = ((sz.max_t-sz.theta_prev[i])+(sz.theta_curr[i]-sz.min_t)+1)
			else:
				diff[i] = diff[i] #valid calculation

		#print "motor position change "
		#print np.array(diff)*sz.radian_const
		return np.array(diff)*sz.radian_const # convert ticks to radians


	def update_goal(sz,delta,mode):
		#add a delta value to the current position to create a goal position
		#  delta: array 1x3[dx,dy,dz]
		if mode == 0: 
			sz.L0_desired = sz.L[0] + delta[2]
			sz.goal = sz.sensed_pos + np.array([delta[0],delta[1],0]) 		
			sz.goal = (np.array(sz.goal)/np.linalg.norm(sz.goal))*sz.L0_desired	
		else:
			sz.goal = np.array([delta[0],delta[1],delta[2]])
			print "goal is "
			print sz.goal

	def set_tether_speeds(sz):
		L_goal = sz.cart2tether_actual(sz.goal)

		position = sz.sensed_pos
		for i in range(4):
			sz.L_vel_desired[i] = (L_goal[i] - sz.L[i]) / sz.looptime


	def end_effector_position(sz,A,B): #forward kinematics. Good for logging but not necessary for control
		yaw = np.arcsin((A[1]-B[1])/np.linalg.norm(A-B))
		pitch = np.arcsin((A[2]-B[2])/np.linalg.norm(A-B))
			
		print A
		print B
		print "yaw is %f" %yaw
		print "pitch is %f" %pitch
		R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],\
				   		   [np.sin(yaw),  np.cos(yaw), 0],\
				   		   [0	       ,  0	     	 , 1]])

		R_pitch = np.array([[ np.cos(pitch), 0, np.sin(pitch)],\
				     		 [0		    	, 1, 0	      	  ],\
				     		 [-np.sin(pitch), 0, np.cos(pitch)]])

		C = np.dot(np.dot(R_yaw,R_pitch),np.array([.25,0,0])) + np.array([A[0],A[1],A[2]])
		return C

	def end_effector_goal(sz,C_des, yaw_des, pitch_des): #inverse kinematics

		R_yaw = np.array([[np.cos(yaw_des), -np.sin(yaw_des), 0],\
				   		   [np.sin(yaw_des),  np.cos(yaw_des), 0],\
				   		   [0		   	   ,  0		     	 , 1]])

		R_pitch = np.array([[ np.cos(pitch_des), 0, np.sin(pitch_des)],\
				    		 [ 0				, 1, 0		     	  ],\
				    		 [-np.sin(pitch_des), 0, np.cos(pitch_des)]])

		A_des = C_des - R_pitch * R_yaw * np.matrix([[.25] ,[0] ,[0]]) # walks backward to A from endeffector based on angles
		
		BA = .51  #distance between A and B. Simplifies the calculations, but not a necessary condition
		
		By = BA * np.sin(yaw_des) + A_des[1]  #gets B from desired angles assuming a fixed distance between A and B
		Bz = BA * np.sin(pitch_des) + A_des[2]
		Bx = A_des[0] - np.sqrt(.51**2 - (A_des[1] - By)**2 - (A_des[2] - Bz)**2)
		B_des = np.matrix([[Bx.item(0)], [By.item(0)], [Bz.item(0)]]) 

		a = A_des - np.matrix([[0], [0], [.09]]) #shifts the desired position from the position of the crossarm to the end of the actual arm
		b = B_des - np.matrix([[-.51], [0], [.09]])	
		ab = [a.item(0), a.item(1), a.item(2), b.item(0), b.item(1), b.item(2)]
		print ab
		return ab

	def Camera_2_World(sz, p_1):
		psi = np.pi/4
		phi = np.pi/2 + np.pi/12
		H_10 = np.array([[ np.cos(psi), np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(psi), 0.04  ],\
						  [-np.sin(psi), np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(psi),-0.28  ],\
						  [ 0	       ,-np.sin(phi)	        , np.cos(phi)		 	 , 0.5725],\
						  [ 0	       , 0                      , 0                      , 1     ]])

		p_0 = np.dot(H_10, p_1)
		print "p_0 is "
		print p_0

		return np.array([p_0.item(0),p_0.item(1),p_0.item(2)])


	def cart2tether_actual(sz,xyz):
		#convert a cartesian goal to tether length goals. assumes the tethers go to points on the outside edge of the end effector.
		#returns a more precise estimate of tether length, but one that is inadmissible to the get_xyz_pos function
		#goal : 1x3 array [x,y,z]
			# L = 1x4 array [L0,L1,L2,L3] spiral zipper and each tether length
		#finds the axis-angle rotation matrix from the column's vertical pose.
		k = np.cross([0,0,sz.L[0]],xyz)
		if np.linalg.norm(k) != 0:
			k = k/np.linalg.norm(k)

		theta = sz.angle_between([0,0,sz.L[0]], xyz)
		Xk = k[0]
		Yk = k[1]
		Zk = k[2]
		#print " k and theta are : "
		#print k
		#print theta
		v = 1 - m.cos(theta)
		R = np.array( [[m.cos(theta) + (Xk**2*v)   , (Xk*Yk*v) - (Zk*m.sin(theta)), (Xk*Zk*v) + Yk*m.sin(theta)],\
					   [2*((Yk*Xk*v) + Zk*m.sin(theta)), m.cos(theta) + (Yk**2*v)     , (Yk*Zk*v) - Xk*m.sin(theta)],\
					   [(Zk*Xk*v) - Yk*m.sin(theta), (Zk*Yk*v) + Xk*m.sin(theta)  , m.cos(theta) + (Zk**2*v)   ]])
		print "cart2tether R is :"
		print R
		L0 = m.sqrt((xyz[0]**2+xyz[1]**2+xyz[2]**2)) # should just be sz.L[0] if not there is a math mistake

		#calculates position vector of the column tether attachment points in the world frame
		OB1 = xyz + np.dot(R,sz.ef[1])  
		OB2 = xyz + np.dot(R,sz.ef[2])		
		OB3 = xyz + np.dot(R,sz.ef[3])

		L1 = np.linalg.norm(OB1 - sz.p[1])
		L2 = np.linalg.norm(OB2 - sz.p[2])
		L3 = np.linalg.norm(OB3 - sz.p[3])

		L = [L0,L1,L2,L3]
		return L

	def rotate(sz,p,q,r): # determines current orientation based on gravity data
		# initial position vector
		v = np.array([0,0,r])
		# Euler angles in degrees from the sensor
		# CCW +ve, CW -ve
		#need to check how things fall here
		#a =  (q) * pi/180  #-1.875
		#b =  (p) * pi/180  #+.5
		# Rotation matrix
		#Ra = np.array([[1, 0 	  ,  0		 ],\
		#			   [0, m.cos(a),-m.sin(a)],\
		#			   [0, m.sin(a), m.cos(a)]])
		#Rb = np.array([[ m.cos(b), 0 , m.sin(b)],\
		#			  [  0		 , 1 , 0	   ],\
		#			  [ -m.sin(b), 0 , m.cos(b)]])
		#R = np.dot(Ra,Rb)
		a = m.asin((q )/9.8)#  + .0875#gets angle and accounts for bias in IMU reading
		b = m.asin((p )/9.8)#  + .01438

		R = np.array([[ (m.cos(b))			  ,         0,  m.sin(b)			],\
					   [(-m.sin(a))*(m.sin(b)),  m.cos(a), (m.sin(a))*(m.cos(b))],\
					   [(-m.sin(b))*(m.cos(a)), -m.sin(a), (m.cos(a))*(m.cos(b))]])
		print "rotate R : "
		print R
		new_v = np.dot(R,v)

		return new_v.T
	
	def angle_between(sz, v1, v2): #gets angle between two input vectors
		v1_u = v1/np.linalg.norm(v1) # unit vectors
		v2_u = v2/np.linalg.norm(v2)

		return np.arccos(np.dot(v1_u, v2_u))  

	def calc_L1(sz, l_state):  #this is probably an unnecessary calculation. get_xyz_pos trilaterates using this tether length.  It shouldn't.
		#Trilateration should done using tether lengths 2 and 3, and the column length
		p1 = (sz.rb+.003)*np.array([0,1,0]) #Coordinates for motor 1
		a = sz.rb*np.cos(pi/6)
		b = sz.rb*np.sin(pi/6)

		x = ((l_state[1]**2)-(l_state[0]**2))/(4*a)
		y = -((sz.L[0]**2)-(l_state[0]**2)+(a**2)+(b**2))/(2*b)+((l_state[1]**2)-(l_state[0]**2))/(4*b)
		z = abs(((sz.L[0]**2)-(x**2)-(y**2)))**0.5

		xyz = [x,y,z]
		return np.linalg.norm(xyz-p1) #Tether length L1 is calculated

	def get_sensor_readings(sz,armflag):
		l = []
		l2 = []
		startread = False
		startRead2 = False
		finished1 = False
		finished2 = False
		count = 0
		#readings = sz.ser_gyro.readline()
		while (len(l) != 3) or (len(l2) != 3):
			# print "trying to read"
			#bytesToRead = sz.ser_gyro.inWaiting()
			#readings = sz.ser_gyro.read(bytesToRead)			
			readings = sz.ser_gyro.readline()
			l = []
			l2 = []
			count = count + 1
			t = 0
			for t in reversed(readings.split()):  # read from most recent serial data

				if (t is '}'):
					startRead2 = True
					continue
				if (startRead2):
					if (t is '{'):
						startRead2 = False
						finished2 = True
					else:
						try:
							l2.append(float(t))
						except ValueError:
							pass

				if (t is ']'):
					startread = True
					continue
				if (startread):
					if (t is '['):
						startread = False
						finished1 = True

					else:
						try:
							l.append(float(t))
						except ValueError:
							pass
				if (finished1 and finished2):
					continue
		#print t
		if t != '[':
			l = sz.lold
		else:
			l = l[::-1]  # Reverse readings to get in [x y z] order
		l2 = l2[::-1]  # Reverse readings to get in [x y z] order
		sz.lold = l			
			#if (len(l2) != 3 or len(l) != 3):
			#	sleep(0.01)

		'''for i in range(3):
			if l[i] >= 9.82:
				l[i] = l[i]/100
			if l2[i] >= 9.82:
				l2[i] = l2[i]/100'''

		#print "Sensor Readings:"
		#print l, l2

		#print "sensor loop repeats %f" %count
		sz.ser_gyro.flushInput()
		if armflag ==0:
			return l2 #check which sensor needs to be checked.  Change between l and l2 as necessary
		elif armflag ==1:
			return l

	def get_lidar1_readings(sz):  #reads lidar sensor data back
		l = []
		while (len(l) != 1):
			#bytesToRead = sz.ser_lidar1.inWaiting()
			#readings = sz.ser_lidar1.read(bytesToRead)
			#sz.ser_lidar1.flushInput()
			sleep(.1)
			readings = sz.ser_lidar1.readline()
			l = []
			startread = False
			for t in reversed(readings.split()):  # read from most recent serial data
				if (t is '>'):
					startread = True
					continue
				if (startread):
					if (t is '<'):
						break
					else:
						try:
							l.append(float(t))
						except ValueError:
							pass
			l = l[::-1]  # Reverse readings to get in [x y z] order
			if (len(l) != 1):
				sleep(0.05)
			#print "LIDAR Readings:"
			#print l
		p = (l[0] * .001 + .02)
		return np.array(p)

	def get_lidar2_readings(sz):  #reads lidar sensor data back
		l = []
		while (len(l) != 1):			
			readings = sz.ser_lidar2.readline()
			l = []
			startread = False
			for t in reversed(readings.split()):  # read from most recent serial data
				if (t is '#'):
					startread = True
					continue
				if (startread):
					if (t is '$'):
						break
					else:
						try:
							l.append(float(t))
						except ValueError:
							pass
			l = l[::-1]  # Reverse readings to get in [x y z] order
			if (len(l) != 1):
				sleep(0.05)
			print "LIDAR Readings:"
			print l
		p = (l[0] * .001 + .02)
		
		sz.ser_lidar2.write("s") #stops arduino from printing data
		return np.array(p)

	def sensed_lidar(sz,flag):  #averages a set of 5 lidar readings
		avg = 0		
		if flag == 0:		
			size = 10
			for i in range(10):
				read = sz.get_lidar1_readings()        
				if read < .12:  #throws away bad data
					read = 0
					size = size - 1
				avg = avg + read
			avg = avg/size
		if flag == 1:
			size = 10
			for i in range(10):
				read = sz.get_lidar2_readings()        
				if read < .12:
					read = 0
					size = size - 1
				avg = avg + read
			if size == 0:
				print "ERROR. lidar not sensing column height properly"
				size = 1
			avg = avg / size
		return avg

	def get_xyz_pos(sz): #Performs Trilateration to get XYZ position
		#assumes tethers begin in z=0 plane and end at the center of the end effector. Also is currently using the three tether lengths. It ought to use tethers 2 & 3 and the column length
		#return end effector position.
		#triangulate the end effector position wikipedia this not x and y are swapped
		d = sz.rb*(3)**0.5 #from state init
		i = d/2
		j = (d**2-i**2)**0.5

		x = (sz.L[3]**2-sz.L[2]**2+d**2)/(2*d)#+i**2+j**2)/(2*j)-i/j*x
		y = (sz.L[3]**2-sz.L[1]**2-x**2+(x-i)**2+j**2)/(2*j)
		#import pdb; pdb.set_trace()
		z = (sz.L[3]**2-y**2-x**2)**0.5
		position_xyz = np.array([x,y,z])+np.array(sz.p[3]) # shift offset
		return position_xyz

	def PI_control(sz,remote_or_auto): #velocity PI control
		desired_speed = sz.L_vel_desired #creates a local variable from the object variable
		actual_speed = sz.L_vel_actual
	
		Kp = sz.get_Kp_Gains(remote_or_auto) #gets PI gains from  the Main file
		Ki = sz.get_Ki_Gains(remote_or_auto) 		
		
		sz.errorsum = sz.errorsum + (np.array(desired_speed) - np.array(actual_speed))  #I control variable

		errorsummax = 1  #sets a limit on how large the I parameter can grow.  Needs to be tuned
		for i in range(4):  #antiwindup loop. Ensures I term never grows larger than the errorsum max
			if sz.errorsum[i] > errorsummax:
				sz.errorsum[i] = errorsummax
			elif sz.errorsum[i] < -1*errorsummax:
				sz.errorsum[i] = -1*errorsummax
		
		t1 = Kp[0] * (desired_speed[0]-actual_speed[0]) + Ki[0]*sz.errorsum[0] #the PI control equation
		if desired_speed[0] ==0:  #if no command input. system shouldn't move from jitters in the controller
			sz.errorsum[0] = 0
			t1 = 0
		
		t2 = Kp[1] * (desired_speed[2]-actual_speed[2]) + Ki[1]*sz.errorsum[2]
		if desired_speed[2] ==0:
			sz.errorsum[2] = 0
			t2 = 0

		t3 = Kp[2] * (desired_speed[3]-actual_speed[3]) + Ki[2]*sz.errorsum[3]
		if desired_speed[3] ==0:
			sz.errorsum[3] = 0
			t3 = 0

		t4 = Kp[2] * (desired_speed[1]-actual_speed[1]) + Ki[2]*sz.errorsum[1]
		if desired_speed[1] ==0:
			sz.errorsum[1] = 0
			t4 = 0
		
		#command_torques = [t1,t2,t3]
		command_torques = [t1,t2,t3,t4]
		#print "command_torques are: "
		#print command_torques
		for i in range(4):
		#for i in range(3):  #check that scales speed commands that are too large down to something the system can manage. Good for position control/step inputs

			if command_torques[i] > 0.75:
				factor = command_torques[i] / 0.75
				for j in range(4):
					command_torques[j] =  command_torques[j] / factor
			if command_torques[i] < -0.75:
				factor = command_torques[i] / -0.75
				for j in range(4):
					command_torques[j] = command_torques[j] / factor

		return command_torques

	def actuate_Motors(sz, c, armflag, remote_or_auto):  #converts desired user input commands into useable servo command inputs
		command_torques = sz.PI_control(remote_or_auto)
		
		for i in range(4):
			if(m.isnan(command_torques[i])): #guarantees that the motors won't vibrate due to I control instabilities
				command_torques[i]=0

		sz.torque_threshold = 0.02 #stops jitters when arm is supposed to be stationary

		if command_torques[0] < .1 and command_torques[0] > 0:  #increases length changing velocity commands if they are too small to help jump the system into motion and avoid stiction and tooth tolerance delays from ruining control
			command_torques[0] = .1
		if command_torques[0] > -.1 and command_torques[0] < 0:
			command_torques[0] = -.1

 		#modification of the constants allows user to specify an extra torque input for the tethers when retracting.  this helps ensure the tethers never go slack.  Probably an obsolete command that can be removed.  Command also flips the sign to fit the system convention.
		if armflag == 0:
			c.at.T1.set_torque(-command_torques[0])
			c.at.T2.set_torque(-command_torques[1])
			c.at.T3.set_torque(-command_torques[2])
			c.at.T7.set_torque(-command_torques[3])
		elif armflag == 1:
			c.at.T4.set_torque(-command_torques[0])
			c.at.T5.set_torque(-command_torques[1])
			c.at.T6.set_torque(-command_torques[2])
			c.at.T8.set_torque(-command_torques[3])

