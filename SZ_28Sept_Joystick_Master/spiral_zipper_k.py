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
		#sz.radian_const = .00549*np.pi/180		
		sz.radian_const = 0.06*np.pi/180

		#init position of tether sources
		sz.rb = 0.22           # circle containing spiral zipper  #.22 for the APC arm, .11 for the RCTA arm
		sz.rt = 0.070  	       # circle containing the spiral zipper column
		sz.r_winch = r_winch   # the radius of the winch used to wind tether.

		phi = pi/6  #rad

		p1 = sz.rb*np.array([0,1, 0]) #+ np.array([0,0,.105])  #positions of the motors in xyz
		p2 = sz.rb*np.array([np.cos(phi),-np.sin(phi), 0]) + np.array([0,0,.125])
		p3 = sz.rb*np.array([-np.cos(phi),-np.sin(phi), 0]) + np.array([0,0,.125])
		sz.p = [np.array(start_position),p1,p2,p3]

		ef1 = sz.rt*np.array([0,1, 0]) #tether attachment points on the end effector assumming arm is straight up
		ef2 = sz.rt*np.array([np.cos(pi/6),-np.sin(pi/6), 0])
		ef3 = sz.rt*np.array([-np.cos(pi/6),-np.sin(pi/6), 0])
		sz.ef = [np.array(start_position),ef1,ef2,ef3]
		print "end effector cable positions are: "
		print sz.ef[1]

		sz.L = sz.cart2tether(start_position, True) #tether length initialization routine
		sz.L_vel_desired = [0,0,0,0]
		sz.L_vel_actual = [0,0,0,0]
		sz.errorsum = [0,0,0,0]	

		#get ckbot's current rotation position for each Ckbot
		if armflag == 0:
			theta1 = c.at.T1.get_pos()
			theta2 = c.at.T2.get_pos()
			theta3 = c.at.T3.get_pos()
			sz.theta0 = [theta1,theta2,theta3]
			sz.L[0] = sz.sensed_lidar(armflag)
			#sz.L[0] = sz.get_lidar1_readings() #takes one lidar reading directly
			sensed_grav = sz.get_sensor_readings(armflag) #takes the average of a set of lidar readings
		elif armflag == 1:
			theta1 = c.at.T4.get_pos()
			theta2 = c.at.T5.get_pos()
			theta3 = c.at.T6.get_pos()
			sz.theta0 = [theta1,theta2,theta3]
			sz.L[0] = sz.sensed_lidar(armflag)			
			#sz.L[0] = sz.get_lidar2_readings() #takes one lidar reading directly
			sensed_grav = sz.get_sensor_readings(armflag) #takes the average of a set of lidar readings
		sz.theta_prev = sz.theta0 # used to update delta position
		sz.theta_curr = sz.theta0 # will hold the current theta of the system

		#First LIDAR Readings
		#sz.L[0] = sz.get_lidar1_readings()
		#zipper2 = sz.get_lidar2_readings()
		sz.L0_desired = sz.L[0]

		#First predicted position is based on initial sensor reading

		sz.sensed_pos = sz.rotate(sensed_grav[0], sensed_grav[1],sz.L[0])
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

		sz.data_store_predict = open("SZ_DATA_predict.txt", "w")
		sz.data_store_measurement = open("SZ_DATA_measurement.txt", "w")
		sz.data_store_estimate = open("SZ_DATA_estimate.txt", "w")
		

	def vacuum_cleaner(sz):  #sends a command to the arduino that controls the vacuum.  Toggles it on and off
		sz.ser_lidar2.write("r")


	def set_Kp_Gains(sz, Kp):
		sz.Kp = Kp


	def set_Ki_Gains(sz, Ki):
		sz.Ki = Ki


	def get_Kp_Gains(sz):
		return sz.Kp


	def get_Ki_Gains(sz):
		return sz.Ki


	def get_goal(sz): #sets the 
		#get goal information
		return sz.goal

		
	def update_state_sensors(sz, c): #currently unused
		#This function updates the position of the system based purely on the IMU and Lidar sensor readings
		rotation=[]
		rotation = sz.get_sensor_readings()
		sz.L[0] = sz.get_lidar1_readings()
		sz.sensed_pos = sz.rotate(rotation[0],rotation[1],sz.L[0])
		sz.L = sz.cart2tether(sz.sensed_pos,False)
		#print "sensed tether lengths are : "
		#print sz.L


	def update_state (sz, c,armflag):
		#This function updates the position of the system based on encoder data and the IMU.
		if armflag ==0:		
			theta1 = c.at.T1.get_pos()
			theta2 = c.at.T2.get_pos()
			theta3 = c.at.T3.get_pos()
			theta_reading = [theta1,theta2,theta3]
		if armflag ==1:		
			theta1 = c.at.T4.get_pos()
			theta2 = c.at.T5.get_pos()
			theta3 = c.at.T6.get_pos()
			theta_reading = [theta1,theta2,theta3]
			
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
		# grows in the CCW direction. if it subtracts we take the negative
		# of the delta to change signs
		rotation=[]
		rotation = sz.get_sensor_readings(armflag)
		sz.L[0] = sz.L[0] - dtheta[0] * 0.00955 #0.06/(2*3.14159)  #change in column height is 16 cm/rev of the column. 2.666 motor revs/column
		sz.sensed_pos = sz.rotate(rotation[0],rotation[1],sz.L[0])
		sz.L = sz.cart2tether_actual(sz.sensed_pos)  #tether length update based on IMU only
		sz.L_old = sz.L
		looptime = now() - sz.timeold
		if sz.tether_subtract_CCW:
			#sz.L[0] = sz.L[0] - dtheta[0] * 0.00955 #0.06/(2*3.14159)  #change in column height is 16 cm/rev of the column. 2.666 motor revs/column rev.  .06 cm/motor revs constants convert units to [cm/rad]
			#sz.L[2] = sz.L[2] - dtheta[1]*sz.r_winch  # tether length update based on encoder data only
			#sz.L[3] = sz.L[3] - dtheta[2]*sz.r_winch# local update
			#sz.L[1] = sz.calc_L1([sz.L[2], sz.L[3]])
		
			sz.L_vel_actual[0] = (- dtheta[0] * 0.00955) / sz.looptime #0.06/(2*3.14159))/sz.looptime #gets the current speed of the system
			#sz.L_vel_actual[1] = (sz.L[1] - (sz.calc_L1([sz.L[2], sz.L[3]]))/sz.looptime
			sz.L_vel_actual[2] =  (-dtheta[1] * sz.r_winch) / sz.looptime
			sz.L_vel_actual[3] =  (-dtheta[2] * sz.r_winch) / sz.looptime

			
		#do not use: sz.L[i] = -(theta_reading[i-1]-sz.theta0[i-1])*sz.r_winch # more global update
        	
		#for i in range(4):
		#	sz.L_vel_actual[i] = (sz.L[i] - sz.L_old[i])/ sz.looptime

		sz.timeold = now()

	def get_CR_delta(sz):
		#compare previous value and current value for modules
    		# returns the delta in radians assums CCW is positive and CCW is defined facing the servo front
      		#get current readings
      		#Assume the current value of the sensor is stored and the previous value is correct.
		diff = [0.0,0.0,0.0]
		#diff = [0.0, 0.0]
		#print "Theta_1 is %f" %sz.theta_curr[1]
		for i in range(3):
			diff[i] = sz.theta_curr[i]-sz.theta_prev[i]
			if abs(diff[i])>sz.limit_t: #calculate valid delta and switch signs to be correct direction
				if sz.theta_curr[i] >= sz.theta_prev[i]:# diff is +ve, so the solution should be -ve
					diff[i] = ((sz.max_t-sz.theta_curr[i])+(sz.theta_prev[i]-sz.min_t)+1)*(-1)
				else: # diff is negative, therfore the solution should be positive
					diff[i] = ((sz.max_t-sz.theta_prev[i])+(sz.theta_curr[i]-sz.min_t)+1)
			else:
				diff[i] = diff[i] #valid calculation

		print "motor position change "
		print np.array(diff)*sz.radian_const
		return np.array(diff)*sz.radian_const # convert ticks to radians


	def update_goal(sz,delta):
		#add a delta value to the current position to create a goal position
		#  delta: array 1x3[dx,dy,dz]
		position = sz.sensed_pos
		sz.L0_desired = sz.L[0] + delta[2]
		sz.goal = position + np.array([delta[0],delta[1],0]) 		
		sz.goal = (np.array(sz.goal)/np.linalg.norm(sz.goal))*sz.L0_desired	


	def set_tether_speeds(sz):
		L_goal = sz.cart2tether_actual(sz.goal)
		#print "desired tether lengths are :"
		#print L_goal
		position = sz.sensed_pos
		looptime = now() - sz.timeold2
		for i in range(4):
			#normally would just use sz.looptime. Attempts to compensate somewhat for variances in loopspeed caused by motor sensing problems
			sz.L_vel_desired[i] = (L_goal[i] - sz.L[i]) / sz.looptime

		sz.timeold2 = now()


	def cart2tether_goal(sz,xyz):  #currently unused. Probably obsolete
		#convert a cartesian goal to tether length goals. assumes a very simple geometry
		#goal : 1x3 array [x,y,z]
       	#col_fixed: logical 0 is fixed length sz 1 is free length sz (length is a free varible)
       	# OUTPUT
       	# L = 1x4 array [L0,L1,L2,L3] spiral zipper and each tether length
		x = xyz[0]
		y = xyz[1]
		z = xyz[2]
		#if half plane, ignore z
		k = x**2+y**2+z**2 #radius of sphere
		p0 = [x,y,z]
		L0 = k**0.5 # should just be sz.L[0] if not there is a math mistake
		L1 = np.linalg.norm(sz.p[1]-p0)
		L2 = np.linalg.norm(sz.p[2]-p0)
		L3 = np.linalg.norm(sz.p[3]-p0)
			
		L = [L0,L1,L2,L3]
		return L

	def cart2tether_actual(sz,xyz):
		#convert a cartesian goal to tether length goals. assumes the tethers go to points on the outside edge of the end effector.
		#returns a more precise estimate of tether length, but one that is inadmissible to the get_xyz_pos function
		#goal : 1x3 array [x,y,z]
       		# L = 1x4 array [L0,L1,L2,L3] spiral zipper and each tether length
       		U = np.cross([0,0,sz.L[0]],xyz)
		theta = sz.angle_between([0,0,sz.L[0]], xyz)
		Xu = np.take(U, 0)
        	Yu = np.take (U, 1)
        	Zu = np.take(U,2)

      		R = np.matrix( [ [m.cos(theta)+Xu**2*(1-m.cos(theta)), Xu*Yu*(1-m.cos(theta))-Zu*m.sin(theta), Xu*Zu*(1-m.cos(theta)+ Yu*m.sin(theta))], \
                               [Yu*Xu*(1-m.cos(theta)+Zu*m.sin(theta)), m.cos(theta)+Yu**2*(1-m.cos(theta)), Yu*Zu*(1-m.cos(theta)-Xu*m.sin(theta))], \
                               [ Zu*Xu*(1-m.cos(theta))+Zu*m.sin(theta), Zu*Yu*(1-m.cos(theta))+Xu*m.sin(theta), m.cos(theta)+Zu**2*(1-m.cos(theta))] ])
		
		k = xyz[0]**2+xyz[1]**2+xyz[2]**2 #radius of sphere
		L0 = k**0.5 # should just be sz.L[0] if not there is a math mistake
		#extracts the position vectors of the base and column tether attachment points.  Probably the wrong way to do this
		ef1 = sz.ef[1]  
		ef2 = sz.ef[2]
		ef3 = sz.ef[3]

		p1 = sz.p[1]
		p2 = sz.p[2]
		p3 = sz.p[3]
		#calculates position vector of the column tether attachment points in the world frame
		OB1 = np.array([[xyz[0]],[xyz[1]],[xyz[2]]]) + R * np.array([[ef1[0]],[ef1[1]],[ef1[2]]])  
		OB2 =np.array([[xyz[0]],[xyz[1]],[xyz[2]]]) + R * np.array([[ef2[0]],[ef2[1]],[ef2[2]]])		
		OB3 = np.array([[xyz[0]],[xyz[1]],[xyz[2]]]) + R * np.array([[ef3[0]],[ef3[1]],[ef3[2]]])
		#finds the norm of that 
		L1 = np.linalg.norm(OB1 - np.array([[p1[0]],[p1[1]],[p1[2]]]))	
		L2 = np.linalg.norm(OB2 - np.array([[p2[0]],[p2[1]],[p2[2]]]))
		L3 = np.linalg.norm(OB3 - np.array([[p3[0]],[p3[1]],[p3[2]]]))
			
		L = [L0,L1,L2,L3]
		return L

	def cart2tether(sz,xyz,col_fixed):
		#convert a cartesian goal to tether length goals. assumes a very simple geometry
		#Assumes tethers begin in the z=0 plane and end at the center of the end effector. Inaccuracies grow as arm length extends
		#goal : 1x3 array [x,y,z]
       		#col_fixed: logical 0 is fixed length sz 1 is free length sz (length is a free varible)
       		# OUTPUT
       		# L = 1x4 array [L0,L1,L2,L3] spiral zipper and each tether length
		x = xyz[0]
		y = xyz[1]
		z = xyz[2]
		#if half plane, ignore z
		if col_fixed is True: #this is only used for initialization purposes
			#back out tether lengths and
			k = x**2+y**2+z**2 #radius of sphere
			p0 = xyz #desiredposition (goal)
			L0 = np.linalg.norm(xyz) # length of centre column
        	
			L1 = np.linalg.norm(sz.p[1]-p0)
			L2 = np.linalg.norm(sz.p[2]-p0)
			L3 = np.linalg.norm(sz.p[3]-p0)
		elif col_fixed is False:
			#use x and y to solve for what z should be then solve for L
			z = (sz.L[0]**2-x**2-y**2)**0.5 #assume positive square root			
			k = x**2+y**2+z**2 #radius of sphere
			p0 = [x,y,z]
			L0 = k**0.5 # should just be sz.L[0] if not there is a math mistake
			L1 = np.linalg.norm(sz.p[1]-p0)
			L2 = np.linalg.norm(sz.p[2]-p0)
			L3 = np.linalg.norm(sz.p[3]-p0)
			
		else:
			print 'error in cart2tether input for col_fixed'

		L = [L0,L1,L2,L3]

		return L

	def calc_L1(sz, l_state):  #this is probably an unnecessary calculation. get_xyz_pos trilaterates using this tether length.  It shouldn't.
		#Trilateration should done using tether lengths 2 and 3, and the column length
		p1 = sz.rb*np.array([0,1,0]) #Coordinates for motor 1
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
			for t in reversed(readings.split()):  # read from most recent serial data
				# print t

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

			l = l[::-1]  # Reverse readings to get in [x y z] order
			l2 = l2[::-1]  # Reverse readings to get in [x y z] order
			#if (len(l2) != 3 or len(l) != 3):
			#	sleep(0.05)

		for i in range(3):
			if l[i] >= 9.82:
				l[i] = l[i]/100
			if l2[i] >= 9.82:
				l2[i] = l2[i]/100

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
		p = (l[0] * .001 + .03)
#		p = [float(x) for x in p]
		return np.array(p)


	def get_lidar2_readings(sz):  #reads lidar sensor data back
		l = []
		while (len(l) != 1):
			#bytesToRead = sz.ser_lidar2.inWaiting()
			#readings = sz.ser_lidar2.read(bytesToRead)
			#sz.ser_lidar2.flushInput()
			#sleep(.1)			
			readings = sz.ser_lidar2.readline()
			l = []
			startread = False
			for t in reversed(readings.split()):  # read from most recent serial data
				if (t is '$'):
					startread = True
					continue
				if (startread):
					if (t is '#'):
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
		p = (l[0] * .001 + .03)
#		p = [float(x) for x in p]
		
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

	def rotate(sz,p,q,r): # determines current orientation based on gravity data
		# initial position vector
		v = np.matrix([[0],[0],[r]])
		# Euler angles in degrees from the sensor
		# CCW +ve, CW -ve
		#need to check how things fail here
		a = m.asin(q/9.81)
		b = m.asin(p/9.81)
		# Rotation matrix
		R = np.matrix([[           (m.cos(b)),         0,               m.sin(b)],
		               [ (-m.sin(a))*(m.sin(b)),  m.cos(a), (m.sin(a))*(m.cos(b))],
		               [(-m.sin(b))*(m.cos(a)),  -m.sin(a), (m.cos(a))*(m.cos(b))]])
		new_v = R*v

		A = np.array(new_v.T)[0]
		return A

	def angle_between(sz, v1, v2): #gets angle between two input vectors
        	v1_u = v1/np.linalg.norm(v1) # unit vectors
        	v2_u = v2/np.linalg.norm(v2)
        	return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))  

	def get_xyz_pos(sz): #Performs Trilateration to get XYZ position
	#assumes tethers begin in z=0 plane and end at the center of the end effector. Also is currently using the three tether lengths. It ought to use tethers 2 & 3 and the column length
		#TODO
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

	def PI_control(sz): #velocity PI control
		desired_speed = sz.L_vel_desired #creates a local variable from the object variable
		actual_speed = sz.L_vel_actual

		Kp = sz.get_Kp_Gains() #gets PI gains from  the Main file
		Ki = sz.get_Ki_Gains() 		
		
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

		command_torques = [t1,t2,t3]

		#print "command_torques are: "
		#print command_torques

		for i in range(3):
			if command_torques[i] > 0.75:
				factor = command_torques[i] / 0.75
				for j in range(3):
					command_torques[j] =  command_torques[j] / factor
			if command_torques[i] < -0.75:
				factor = command_torques[i] / -0.75
				for j in range(3):
					command_torques[j] = command_torques[j] / factor

		return command_torques

	def actuate_Motors(sz, c, armflag):  #converts desired user input commands into useable servo command inputs
		command_torques = sz.PI_control()
		
		if(m.isnan(command_torques[0])): #guarantees that the motors won't vibrate due to I control instabilities
			command_torques[0]=0
		if(m.isnan(command_torques[1])):
			command_torques[1]=0
		if(m.isnan(command_torques[2])):
			command_torques[2]=0

		sz.torque_threshold = 0.02

		if command_torques[0] < .1 and command_torques[0] > 0:
			command_torques[0] = .1
		if command_torques[0] > -.1 and command_torques[0] < 0:
			command_torques[0] = -.1
		
		if sz.tether_subtract_CCW:
 		#modification of the constants allows user to specify an extra torque input for the tethers when retracting.  this helps ensure the tethers never go slack.  Probably an obsolete command that can be removed.  Command also flips the sign to fit the system convention.
			if armflag == 0:
				c.at.T1.set_torque(-command_torques[0])
				c.at.T2.set_torque(-command_torques[1])
				c.at.T3.set_torque(-command_torques[2])
			elif armflag == 1:
				c.at.T4.set_torque(-command_torques[0])
				c.at.T5.set_torque(-command_torques[1])
				c.at.T6.set_torque(-command_torques[2])

		else:
			if armflag == 0:
				c.at.T1.set_torque(command_torques[0])
				c.at.T2.set_torque(command_torques[1])
				c.at.T3.set_torque(command_torques[2])
			if armflag == 1:
				c.at.T4.set_torque(command_torques[0])
				c.at.T5.set_torque(command_torques[1])
				c.at.T6.set_torque(command_torques[2])

