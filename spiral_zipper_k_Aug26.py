import numpy as np
import algopy as ap
from algopy import UTPM

from numpy import pi
import serial
import os
from time import time as now, sleep
import math as m

class SpiralZipper:
	


	def __init__(sz, start_position, r_winch, c):
			
		#Lengths set the initial configureation of the system.
        # Lengths: array 1x4 [L0 ,L1,L2,L3]
        #create spiral zipper object
        #physical notes
        #sz ckbot direction: CCW subtracts tether and CW adds/releases tether.
		sz.startposition = start_position
		sz.start_detected = False
		sz.entered = False
		sz.tether_subtract_CCW = True


		sz.goal_prev = start_position
		sz.errorsum = [0, 0]
		sz.goal_start = now()
		sz.goal_stop =now()
		sz.target_achieved = [0, 0]
		sz.target_reached = False
		sz.P = 10*np.eye(2) #System covariance
		sz.Q = np.eye(2) #System noise Covariance
		sz.R = 0.01*np.eye(2) #Sensor Noise Covariance


		try:
			sz.ser = serial.Serial('/dev/ttyACM0',9600)
		except serial.SerialException:
			sz.ser = serial.Serial('/dev/ttyACM1',9600)
		#sz.wakeup_configz(c)
            # Initalize Spiral Zipper Settings #TODO: set these
            #L0 = Lengths[0]#0.3 #length of spiral zipper column
            #L1 = Lengths[1]#0.5 #length of the i-th tether cable
            #L2 = Lengths[2]#0.5#^
            #L3 = Lengths[3]#0.5 #
            #TODO: limts_xyz = []# set limits on the end effector position
            #sz.L = [L0,L1,L2,L3]

		sleep(1)
		# set params for ckbot control
		sz.max_t = 65535
		sz.min_t = 0
		sz.limit_t = sz.max_t/2
		sz.radian_const = 0.06*np.pi/180
		#sz.slack_remove(c)
		#sz.wakeup_configz(c)							!!!!!!!!!!!
	
         
		#get ckbot's current rotation position for each Ckbot
		#theta1 = c.at.T1.get_pos()
		theta2 = c.at.T2.get_pos()
		theta3 = c.at.T3.get_pos()
		#sz.theta0 = [theta1,theta2,theta3]
		sz.theta0 = [theta2, theta3]
		sz.theta_prev = sz.theta0 # used to update delta position
		sz.theta_curr = sz.theta0 #will hold the current theta of the system

		#theta1_k = c.at.T1.get_pos()
		theta2_k = c.at.T2.get_pos()
		theta3_k = c.at.T3.get_pos()
		#sz.theta0_k = [theta1_k,theta2_k,theta3_k]
		sz.theta0_k = [theta2_k, theta3_k]
		sz.theta_prev_k = sz.theta0_k # used to update delta position
		sz.theta_curr_k = sz.theta0_k #will hold the current theta of the system


		#init position of tether sources
		sz.rb = 0.13 #circle containing spiral zipper
		p1 = sz.rb*np.array([0,1,0]) #Coordinates for motor 1
		p2 = sz.rb*np.array([np.cos(pi/6),-np.sin(pi/6),0]) #Coordinates for motor 2
		p3 = sz.rb*np.array([-np.cos(pi/6),-np.sin(pi/6),0]) #Coordinates for motor 3
		sz.p = [np.array(start_position),p1,p2,p3]
		
		sz.r_winch = r_winch# the radius of the winch used to wind tether.


		#First predicted position is based on initial sensor reading
		sensed_grav = sz.get_sensor_readings()
		sensed_pos = sz.rotate(sensed_grav[0], sensed_grav[1])
		state_err = np.matrix(sz.cart2tether(sensed_pos, True)[2:])  # [sz.L[1],sz.L[2],sz.L[3]]) #goal minus current position
         
		sz.L = sz.cart2tether(sensed_pos, True)

		print "L_Actual "
		print sz.L

		sz.l_predict = state_err.transpose()

		print "L_Predict initial : "
		print sz.l_predict


		sz.goal = start_position # init goal as initail length
		#fixed column goal limit
		sz.goal_max_fixed_col = np.cos(pi/5)*sz.L[0] #limit to 60 degrees away
	
         
	def get_state(sz):
		# used to return state
		return (self.p, self.L)

	def get_error_state(sz):
		# return the error values for each actuator
		# if tether is too short the error is positive and the tether should grow.
      
     	# here i use the tether length to calculate error.
      	#import pdb; pdb.set_trace()
		tether_goal = sz.get_tether_goal()
		#sensed_grav = sz.get_sensor_readings()
		#sensed_pos = sz.rotate(sensed_grav[0],sensed_grav[1])
		l1 = sz.calc_L1(np.array([sz.l_predict[0,0], sz.l_predict[1,0]]))
		lengths = [l1, sz.l_predict[0,0], sz.l_predict[1,0]]
		state_err = np.array(tether_goal[1:])- lengths#[sz.L[1],sz.L[2],sz.L[3]]) #goal minus current position
		print "Estimated Tether Lengths "
		#print sz.cart2tether(sensed_pos,True)
		print lengths
		return state_err

	def get_speed_error(sz):
		pass


	def get_error_xyz(sz):
		return np.array(sz.get_xyz_pos())-np.array(sz.get_goal())



	def update_state (sz, c):
		# this takes in the current readings of the system and updates the
		# position of the spiral zipper and the tether lengths
        
		#get current theta of each ckbot and use the change to figure out the changes

		#read sensor data:
		"""rotation=[]
		rotation=sz.get_sensor_readings()
		print "Sensor XYZ readings:"
		print rotation
		sensed_position=[]
		sensed_position = sz.rotate(rotation[0],rotation[1])
		print sensed_position"""
		#print "Length of Array:"
    	#print len(rotation)
    	#Read current theta
		#theta1 = c.at.T1.get_pos()
		theta2 = c.at.T2.get_pos()
		theta3 = c.at.T3.get_pos()
		#theta_reading = [theta1,theta2,theta3]
		theta_reading = [theta2, theta3]
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
		for i in range(2,4):
			if sz.tether_subtract_CCW:
				sz.L[i] = sz.L[i] - dtheta[i-2]*sz.r_winch # local update
				#do not use: sz.L[i] = -(theta_reading[i-1]-sz.theta0[i-1])*sz.r_winch # more global update
        	'''else:
				sz.L[i] = sz.L[i] + dtheta[i-1]*sz.r_winch # local update'''

		sz.L[1] = sz.calc_L1([sz.L[2], sz.L[3]])
	# end function


	def get_CR_delta(sz):
		#compare previous value and current value for modules
    	# returns the delta in radians assums CCW is positive and CCW is defined facing the servo front
      	#get current readings
      	#Assume the current value of the sensor is stored and the previous value is correct.
		#diff = [0.0,0.0,0.0]
		diff = [0.0, 0.0]
		#print "Theta_1 is %f" %sz.theta_curr[1]
		for i in range(2):
			diff[i] = sz.theta_curr[i]-sz.theta_prev[i]
			if abs(diff[i])>sz.limit_t: #calculate valid delta and switch signs to be correct direction
				if sz.theta_curr[i] >= sz.theta_prev[i]:# diff is +ve, so the solution should be -ve
					diff[i] = ((sz.max_t-sz.theta_curr[i])+(sz.theta_prev[i]-sz.min_t)+1)*(-1)
				else: # diff is negative, therfore the solution should be positive
					diff[i] = ((sz.max_t-sz.theta_prev[i])+(sz.theta_curr[i]-sz.min_t)+1)
			else:
				diff[i] = diff[i] #valid calculation
		# now set the old thetas as the new
		sz.theta_prev = sz.theta_curr

		#print "CR_Delta "
		#print np.array(diff)*sz.radian_const
		return np.array(diff)*sz.radian_const # convert ticks to radians

	def get_CR_delta_k(sz):
		# compare previous value and current value for modules
		# returns the delta in radians assums CCW is positive and CCW is defined facing the servo front
		# get current readings
		# Assume the current value of the sensor is stored and the previous value is correct.
		#diff_k = [0.0, 0.0, 0.0]
		diff_k = [0.0, 0.0]
		# print "Theta_1 is %f" %sz.theta_curr[1]
		for i in range(2):
			diff_k[i] = sz.theta_curr_k[i] - sz.theta_prev_k[i]
			if abs(diff_k[i]) > sz.limit_t:  # calculate valid delta and switch signs to be correct direction
				if sz.theta_curr_k[i] >= sz.theta_prev_k[i]:  # diff is +ve, so the solution should be -ve
					diff_k[i] = ((sz.max_t - sz.theta_curr_k[i]) + (sz.theta_prev_k[i] - sz.min_t) + 1) * (-1)
				else:  # diff is negative, therfore the solution should be positive
					diff_k[i] = ((sz.max_t - sz.theta_prev_k[i]) + (sz.theta_curr_k[i] - sz.min_t) + 1)
			else:
				diff_k[i] = diff_k[i]  # valid calculation
			# now set the old thetas as the new
		#sz.theta_prev_k = sz.theta_curr_k
		return np.array(diff_k) * sz.radian_const  # convert ticks to radians




      

	def set_goal_xyz(sz,goal):
		#set goal in xyz goal in 1x3[x,y,z]
		#sz.goal_prev = sz.goal[:]
		sz.goal = goal
     

	def get_goal(sz):
		#get goal information
		return sz.goal

	def set_Motor_Gains(sz,K_gain):
		#set motor gains
		sz.K_M_set = K_gain


	def set_Integral_Data(sz, Ki, Ki_Thresholds):
		#set Ki values and threshold values for levels of integral control
		sz.Ki = Ki
		sz.Ki_t = Ki_Thresholds
		if(len(Ki)!=(len(Ki_Thresholds)-1)):
			raise ValueError('Mismatch in integral thresholds and gains')

	def get_Motor_Gains(sz):
		#get motor gain information
		print "Motor gains: "
		print sz.K_M_set
		ret_gains=sz.K_M_set[:]
		return ret_gains

	def get_Integral_Gains(sz):
		#get integral gain information
		return sz.Ki

	def get_Integral_Thresholds(sz):
		#Get integral thresholds information
		return sz.Ki_t

	def get_tether_goal(sz):
		#convert cartesian goal to tether length goal
		L_goal = sz.cart2tether(sz.goal,True)
		return L_goal


	def update_goal(sz,delta):
		#add a delta value to the current goal
		#  delta: array 1x3[dx,dy,dz]
		sz.goal = np.array(sz.goal)+delta
		#solve for z goal
		sz.goal[2] = (sz.L[0]**2-sz.goal[0]**2-sz.goal[1]**2)**0.5
		#limit goal to this range. assume onlt in x-y for now
		for i in range(2):
			if sz.goal[i]>sz.goal_max_fixed_col:
				sz.goal[i] = sz.goal_max_fixed_col
				print 'LIMIT REACHED!'
			elif sz.goal[i]<-sz.goal_max_fixed_col:
				sz.goal[i] = -sz.goal_max_fixed_col
				print 'negative LIMIT REACHED!'

     
	def cart2tether(sz,xyz,col_fixed):
		#convert a cartesian goal to tether length goals. assumes a very simple geometry
		#goal : 1x3 array [x,y,z]
       	#col_fixed: logical 0 is fixed length sz 1 is free length sz (length is a free varible)
       	# OUTPUT
       	# L = 1x4 ararry [L0,L1,L2,L3] sprial zipper and each tether length
		x = xyz[0]
		y = xyz[1]
		z = xyz[2]
		#if half plane, ignore z
		if col_fixed is True:
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
     
	'''def calc_L1(sz):
		#Trilateration done using tether lengths 2 and 3, and the column length
		p1 = sz.rb*np.array([0,1,0]) #Coordinates for motor 1
		a = sz.rb*np.cos(pi/6)
		b = sz.rb*np.sin(pi/6)

		x = ((sz.L[3]**2)-(sz.L[2]**2))/(4*a)
		y = -((sz.L[0]**2)-(sz.L[2]**2)+(a**2)+(b**2))/(2*b)+((sz.L[3]**2)-(sz.L[2]**2))/(4*b)
		z = abs(((sz.L[0]**2)-(x**2)-(y**2))**0.5)

		xyz = [x,y,z]
		sz.L[1] = np.linalg.norm(xyz-p1) #Tether length L1 is calculated'''

	def calc_L1(sz, l_state):
		#Trilateration done using tether lengths 2 and 3, and the column length
		p1 = sz.rb*np.array([0,1,0]) #Coordinates for motor 1
		a = sz.rb*np.cos(pi/6)
		b = sz.rb*np.sin(pi/6)

		x = ((l_state[1]**2)-(l_state[0]**2))/(4*a)
		y = -((sz.L[0]**2)-(l_state[0]**2)+(a**2)+(b**2))/(2*b)+((l_state[1]**2)-(l_state[0]**2))/(4*b)
		z = abs(((sz.L[0]**2)-(x**2)-(y**2))**0.5)

		xyz = [x,y,z]
		return np.linalg.norm(xyz-p1) #Tether length L1 is calculated


	def get_sensor_readings(sz):
		l=[]
		while(len(l)!=3):
			bytesToRead = sz.ser.inWaiting()
			readings = sz.ser.read(bytesToRead)
		
	
			l = []
			startread=False
			for t in reversed(readings.split()): # read from most recent serial data
				if(t is ']'):
					startread=True
					continue
				if(startread):
					if(t is '['):
						break
					else:
						try:
							l.append(float(t))
						except ValueError:
							pass
			l=l[::-1] # Reverse readings to get in [x y z] order
			if(len(l)!=3):
				sleep(0.25)
			print "Sensor Readings:"
			print l

			
		return l
  	        
	def rotate(sz,p,q): # determines current orientation based on gravity data
		# initial position vector
		v = np.matrix([[0],
		              [0],
					 [0.3175]])
		# Euler angles in degrees from the sensor
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
	    #new_v = np.transpose(new_v)
		A = np.array(new_v.T)[0]
		return A

	'''def slack_remove(sz, c):
		print "Removing slacks..."
		T1_slack=True
		T2_slack=True
		T3_slack=True
		unwind_time=0.3
		deflection_small=0.05
		deflection_large=0.10

		grav_comps = sz.get_sensor_readings() # read GRAVITY components 
		desired_comps = [0.0, 0.0, 9.8]
		#STEP_1 remove slacks
		if (grav_comps[1]<0): # leaning away from T1
			#Unwind T2 and T3 a bit
			c.at.T2.set_torque(-0.1) #T2
			sleep(unwind_time)
			c.at.T2.set_torque(0)
			c.at.T3.set_torque(-0.1) #T3
			sleep(unwind_time)
			c.at.T3.set_torque(0)
			#Wind T1 till deflection sensed => 				T1 slack removed.
			grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components 
			pos_old = c.at.T1.get_pos()
			while(T1_slack):
				grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
				
				if(grav_sensed_curr[1]-grav_sensed_init[1] > deflection_large):
					T1_slack=False
					print "T1 Slack removed."
					break				
				c.at.T1.set_torque_mx(0.05) 
				pos_new = c.at.T1.get_pos()
				change = sz.measure_dtheta(pos_new, pos_old)
				print "Delta Theta is : %f"%change
				pos_old=pos_new
			c.at.T1.set_torque_mx(0) #stop
			
			if(grav_comps[0]<0): #leaning away from T2
				#unwind T3 a bit
				c.at.T3.set_torque(-0.1) #T3
				sleep(unwind_time)
				c.at.T3.set_torque(0)
				#wind T2 till deflection sensed =>				T2 slack removed
				pos_old = c.at.T2.get_pos()
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components 
				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
				#wind T3 till *small*deflection sensed =>		T3 slack removed
				pos_old = c.at.T3.get_pos()
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components 
				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):
						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				
			elif(grav_comps[0]>0): #leaning away from T3
				#unwind T2 a bit
				c.at.T2.set_torque(-0.1) #T2
				sleep(unwind_time)
				c.at.T2.set_torque(0)
				
				# wind T3 till deflection sensed =>				T3 slack removed
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components
				pos_old = c.at.T3.get_pos()
				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				# wind T2 till *small* deflection sensed =>		T2 slack removed
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components
				pos_old = c.at.T2.get_pos()
				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
				
		elif (grav_comps[1]>0): # leaning towards T1

			if(grav_comps[0]<0): #leaning away from T2
				#unwind T1 and T3 a bit
				c.at.T1.set_torque_mx(-0.05) #T1
				sleep(unwind_time)
				c.at.T1.set_torque_mx(0)
				c.at.T3.set_torque(-0.1) #T3
				sleep(unwind_time)
				c.at.T3.set_torque(0)
				
				#wind T2 till deflection sensed =>				T2 slack removed
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components
				pos_old = c.at.T2.get_pos()
				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
				
				
				#wind T3 till *small* deflection sensed => 		T3 slack removed
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components
				pos_old = c.at.T3.get_pos()
				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):
						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				
			elif(grav_comps[0]>0): #leaning away from T3
				#unwind T1 and T2 a bit
				c.at.T1.set_torque_mx(-0.05) #T1
				sleep(unwind_time)
				c.at.T1.set_torque_mx(0)
				c.at.T2.set_torque(-0.1) #T2
				sleep(unwind_time)
				c.at.T2.set_torque(0)
				
				# wind T3 till deflection sensed =>				T3 slack removed
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components
				pos_old = c.at.T3.get_pos()
				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				# wind T2 till *small* deflection sensed =>		T2 slack removed
				grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components
				pos_old = c.at.T2.get_pos()
				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = sz.measure_dtheta(pos_new, pos_old)
					print "Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
			
			#wind T1 till *small*deflection sensed =>		T1 slack removed
			grav_sensed_init = sz.get_sensor_readings() #read GRAVITY components
			pos_old = c.at.T1.get_pos()
			while(T1_slack):
				grav_sensed_curr = sz.get_sensor_readings() #read GRAVITY components 
				if(abs(grav_sensed_curr[1]-grav_sensed_init[1]) > deflection_small):
					T1_slack=False
					print "T1 Slack removed."
					break				
				c.at.T1.set_torque_mx(0.05)
				pos_new = c.at.T1.get_pos()
				change = sz.measure_dtheta(pos_new, pos_old)
				print "Delta Theta is : %f" % change
				pos_old = pos_new
			c.at.T1.set_torque_mx(0) #stop
		print "All slacks removed."
		#

		#STEP_2	center the column
		#STEP_3 Reset the accelerometer'''
		

	'''def measure_dtheta(sz, pos_new, pos_old):
		diff = pos_new - pos_old

		if abs(diff)>sz.limit_t:
			#calculate valid delta and switch signs to be correct direction
			if pos_new >= pos_old:# diff is +ve, so the solution should be -ve
				diff = ((sz.max_t-pos_new)+(pos_old-sz.min_t)+1)*(-1)
			else: # diff is negative, therfore the solution should be positive
				diff = ((sz.max_t-pos_old)+(pos_new-sz.min_t)+1)
		else:
			diff = diff #valid calculation
		return diff # convert ticks to radians'''

				


	def get_xyz_pos(sz):
		#TODO
	    #return end effector position.
  		#trianglulate the end effector position wikipedia this not x and y are swapped
		d = sz.rb*(3)**0.5 #from state init
		i = d/2
		j = (d**2-i**2)**0.5
	      
		x = (sz.L[3]**2-sz.L[2]**2+d**2)/(2*d)#+i**2+j**2)/(2*j)-i/j*x
		y = (sz.L[3]**2-sz.L[1]**2-x**2+(x-i)**2+j**2)/(2*j)
		#import pdb; pdb.set_trace()
		z = (sz.L[3]**2-y**2-x**2)**0.5
		position_xyz = np.array([x,y,z])+np.array(sz.p[3]) # shift offset
		return position_xyz

	def PI_control(sz,c):

		Ki_Integral = sz.get_Integral_Gains()
		print "Integral Gains :"
		print Ki_Integral
		Ki_Threshold = sz.get_Integral_Thresholds()
		print "Integral Thresholds :"
		print Ki_Threshold

		sz.kalman_predict(c)
		sz.update_state(c)


		sz.goal_changed = False

		if(not np.array_equal(sz.goal, sz.goal_prev)):
			sz.entered = True;
			sz.goal_changed = True
			sz.start_detected = False


		if(np.array_equal(sz.goal, sz.startposition) and sz.start_detected == False):
			sz.start_detected = True
			sz.goal_changed = True


		if(sz.goal_changed):
			sz.goal_prev = sz.goal[:]
			sz.errorsum = [0, 0]
			sz.goal_start = now()
			sz.target_achieved = [0, 0]
			sz.target_reached = False

		print "Goal is : "
		print sz.goal


		deltas_tether = sz.get_error_state()[1:]
		print "Errors are:"
		print deltas_tether
		K_Motor = sz.get_Motor_Gains()
		print "Motor Gains :"
		print K_Motor


		Ki_t = [0,0]

		for motor in range(0,2):
			level_set = -1
			for level in range(0,len(Ki_Threshold)):
				if level is len(Ki_Threshold)-1:
					if((abs(deltas_tether[motor]) <= Ki_Threshold[level]) and abs(deltas_tether[motor]) >= 0):
						level_set = level
						break
				elif (abs(deltas_tether[motor]) <= Ki_Threshold[level] and abs(deltas_tether[motor]) > Ki_Threshold[level+1]):
					level_set = level
					break


			if (level_set is len(Ki_Threshold) -2):
				Ki_t[motor] = Ki_Integral[level_set]
				# K_Motor[0] = sz.K_M[0]
				print " Integral Control Tether %d level %d (Target Acquired)" % (motor, level_set)
				sz.errorsum[motor] = sz.errorsum[motor] + deltas_tether[motor]
				sz.target_achieved[motor] = 1  # Assume target achieved

			elif(level_set is len(Ki_Threshold)-1):
				Ki_t[motor] = 0
				print "Target acquired tether %d" % motor
				sz.errorsum[motor] = 0
				K_Motor[motor] = 0
				sz.target_achieved[motor] = 1  # Assume target achieved

			elif(level_set in range(0,len(Ki_Threshold)-2)):
				Ki_t[motor] = Ki_Integral[level_set]
				# K_Motor[0]=sz.K_M[0]
				print " Integral Control Tether %d level %d" % (motor, level_set)
				sz.errorsum[motor] = sz.errorsum[motor] + deltas_tether[motor]
				sz.target_achieved[motor] = 0

			else:
				Ki_t[motor] = 0
				# K_Motor[0] = sz.K_M[0]
				print " No integral tether %d" % motor
				sz.target_achieved[motor] = 0


		# Check if reached target position and calculate time
		if ((sz.target_achieved[0] is 1) and (sz.target_achieved[1] is 1)):
			if (sz.target_reached is False):
				sz.goal_stop = now()
			print "Goal reached in %f seconds." % (sz.goal_stop - sz.goal_start)
			print "Goal stop: "
			print sz.goal_stop
			sz.target_reached = True


		t1 = (deltas_tether[0] + sz.errorsum[0] * Ki_t[0] ) * K_Motor[0]  # control input for motor 1 with gain factored in

		t2 = (deltas_tether[1] + sz.errorsum[1] * Ki_t[1] ) * K_Motor[1]  # control input for motor 2 with gain factored in

		#t3 = (deltas_tether[2] + sz.errorsum[2] * Ki_t[2] ) * K_Motor[2]  # control input for motor 3 with gain factored in

		command_torques=[t1, t2]


		return command_torques





	def actuate_Motors(sz, c, command_torques):

		if(m.isnan(command_torques[0])):
			command_torques[0]=0
		if(m.isnan(command_torques[1])):
			command_torques[1]=0
		#if(m.isnan(command_torques[2])):
		#	command_torques[2]=0

		sz.torque_threshold = 0.02

		#if((command_torques[1] is 0) and (command_torques[2] is 0)):
		#	command_torques[0] = 0.0

		if sz.tether_subtract_CCW:
			# t1=-0.0001


			'''if command_torques[0] < 0:
				c.at.T1.set_torque_mx(-1.1 * command_torques[0])
			else:
				if ((abs(command_torques[1]) < sz.torque_threshold) and (abs(command_torques[2]) < sz.torque_threshold) or (abs(command_torques[0]) < 0.007)):  # other torques are insignificant
					command_torques[0] = 0;
				c.at.T1.set_torque_mx(-0.5 * command_torques[0])'''
			if command_torques[0] < 0:
				c.at.T2.set_torque(-1.4 * command_torques[0])
			else:
				#if ((abs(command_torques[0]) < sz.torque_threshold) and (abs(command_torques[2]) < sz.torque_threshold)):  # other torques are insignificant
				#	command_torques[1] = 0;
				c.at.T2.set_torque(-command_torques[0])
			if command_torques[1] < 0:
				c.at.T3.set_torque(-1.4 * command_torques[1])
			else:
				#if ((abs(command_torques[0]) < sz.torque_threshold) and (abs(command_torques[1]) < sz.torque_threshold)):  # other torques are insignificant
				#	command_torques[2] = 0;
				c.at.T3.set_torque(-command_torques[1])

		else:
			#c.at.T1.set_torque(command_torques[0])
			c.at.T2.set_torque(command_torques[0])
			c.at.T3.set_torque(command_torques[1])


	def eval_xy(sz, l_input):
		xy = ap.zeros(2, dtype=l_input)
		a = sz.rb * np.cos(pi / 6)
		b = sz.rb * np.sin(pi / 6)
		xy[0] = ((l_input[1] ** 2) - (l_input[0] ** 2)) / (4 * a)
		xy[1] = -((sz.L[0] ** 2) - (l_input[0] ** 2) + (a ** 2) + (b ** 2)) / (2 * b) + ((l_input[1] ** 2) - (l_input[0] ** 2)) / (4 * b)
		#xyz[2] = abs(((sz.L[0] ** 2) - (xyz[0] ** 2) - (xyz[1] ** 2)) ** 0.5)
		#xyz = [x,y,z]
		print "xy eval_xy "
		print xy
		return xy

	def eval_hx(sz, l_input):
		a = sz.rb * np.cos(pi / 6)
		b = sz.rb * np.sin(pi / 6)
		x = ((l_input[1] ** 2) - (l_input[0] ** 2)) / (4 * a)
		y = -((sz.L[0] ** 2) - (l_input[0] ** 2) + (a ** 2) + (b ** 2)) / (2 * b) + ((l_input[1] ** 2) - (l_input[0] ** 2)) / (4 * b)
		#xyz[2] = abs(((sz.L[0] ** 2) - (xyz[0] ** 2) - (xyz[1] ** 2)) ** 0.5)
		xy = [x,y]
		print "hx eval_hx "
		print xy
		return xy


	def kalman_predict(sz, c):
		#theta1_k = c.at.T1.get_pos()
		theta2_k = c.at.T2.get_pos()
		theta3_k = c.at.T3.get_pos()
		#theta_reading_k = [theta1_k, theta2_k, theta3_k]
		theta_reading_k = [theta2_k, theta3_k]

		# update current theta
		sz.theta_curr_k = theta_reading_k
		# calculate the change in position since last call
		dtheta_k = np.matrix([sz.get_CR_delta_k()])  # see method for sign convention
		dtheta_k = dtheta_k.transpose()
		# update previous theta
		sz.theta_prev_k = sz.theta_curr_k

		F = np.matrix(np.eye(2))
		B = np.matrix(sz.r_winch*np.eye(2))
		#Process/System Model
		sz.l_predict = F*sz.l_predict - B*dtheta_k


		print "dTheta is "
		print dtheta_k
		print "L_predict is "
		print sz.l_predict
		print "L_Actual is "
		print sz.L

		#Covariance
		sz.P = F*sz.P*F.transpose() + sz.Q

		print "P "
		print sz.P

		#Calculating Jacobian
		#l_input = [sz.L[0], sz.L[2], sz.L[3]]
		l_input = np.array([sz.l_predict[0,0], sz.l_predict[1,0]])
		print "l_input before Jacobian "
		print l_input
		l_input = ap.UTPM.init_jacobian(l_input)
		H_k = ap.UTPM.extract_jacobian(sz.eval_xy(l_input))
		H = np.matrix(H_k)

		print "Jacobian "
		print H_k

		matrix_den = np.linalg.inv(((H*sz.P*H.transpose()) + sz.R))
		#Kalman_Gain = ((sz.P*H.transpose())/((H*sz.P*H.transpose()) + sz.R))
		Kalman_Gain = (sz.P * H.transpose())*matrix_den

		print "Kalman Gain "
		print Kalman_Gain

		hx = np.matrix(sz.eval_hx(np.array([sz.l_predict[0,0], sz.l_predict[1,0]])))
		hx = hx.transpose()
		print "HX "
		print hx
		sensed_grav = sz.get_sensor_readings()
		sensed_pos = sz.rotate(sensed_grav[0], sensed_grav[1])
		Kalman_Z = np.matrix([sensed_pos[:2]])
		print "Sensed Kalman "
		print Kalman_Z
		#Kalman_test = np.matrix(sz.cart2tether(sensed_pos, True)[2:])  # [sz.L[1],sz.L[2],sz.L[3]]) #goal minus current position
		Kalman_Z = Kalman_Z.transpose()

		sz.l_predict = sz.l_predict + Kalman_Gain*(Kalman_Z - hx)

		print "l_predict after update "
		print sz.l_predict

		sz.P = (np.eye(2) - Kalman_Gain*H)*sz.P

		print "P after update "
		print sz.P






















