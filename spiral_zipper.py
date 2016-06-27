import numpy as np
from numpy import pi
import serial
import os
import time 
import math as m

class SpiralZipper:
	


	def __init__(sz, start_position, r_winch, c):
			
		#Lengths set the initial configureation of the system.
        # Lengths: array 1x4 [L0 ,L1,L2,L3]
        #create spiral zipper object
        #physical notes
        #sz ckbot direction: CCW subtracts tether and CW adds/releases tether.
		sz.tether_subtract_CCW = True
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

		# set params for ckbot control
		sz.max_t = 65535
		sz.min_t = 0
		sz.limit_t = sz.max_t/2
		sz.radian_const = 0.06*np.pi/180
		sz.slack_remove(c)
		#sz.wakeup_configz(c)							!!!!!!!!!!!
	
         
		#get ckbot's current rotation position for each Ckbot
		theta1 = c.at.T1.get_pos()
		theta2 = c.at.T2.get_pos()
		theta3 = c.at.T3.get_pos()
		sz.theta0 = [theta1,theta2,theta3]
		sz.theta_prev = sz.theta0 # used to update delta position
		sz.theta_curr = sz.theta0 #will hold the current theta of the system


		#init position of tether sources
		sz.rb = 0.13 #circle containing spiral zipper
		p1 = sz.rb*np.array([0,1,0]) #Coordinates for motor 1
		p2 = sz.rb*np.array([np.cos(pi/6),-np.sin(pi/6),0]) #Coordinates for motor 2
		p3 = sz.rb*np.array([-np.cos(pi/6),-np.sin(pi/6),0]) #Coordinates for motor 3
		sz.p = [np.array(start_position),p1,p2,p3]
		
		sz.r_winch = r_winch# the radius of the winch used to wind tether.
         
		sz.L = sz.cart2tether(start_position,True)


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
		sensed_grav = sz.get_sensor_readings()
		sensed_pos = sz.rotate(sensed_grav[0],sensed_grav[1])
		state_err = np.array(tether_goal[1:])- np.array(sz.cart2tether(sensed_pos,True)[1:])#[sz.L[1],sz.L[2],sz.L[3]]) #goal minus current position
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
		theta1 = c.at.T1.get_pos()
		theta2 = c.at.T2.get_pos()
		theta3 = c.at.T3.get_pos()
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
		for i in range(1,4):
			if sz.tether_subtract_CCW:
				sz.L[i] = sz.L[i] - dtheta[i-1]*sz.r_winch # local update
				#do not use: sz.L[i] = -(theta_reading[i-1]-sz.theta0[i-1])*sz.r_winch # more global update
        	else:
				sz.L[i] = sz.L[i] + dtheta[i-1]*sz.r_winch # local update
	# end function


	def get_CR_delta(sz):
		#compare previous value and current value for modules
    	# returns the delta in radians assums CCW is positive and CCW is defined facing the servo front
      	#get current readings
      	#Assume the current value of the sensor is stored and the previous value is correct.
		diff = [0.0,0.0,0.0]
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
			# now set the old thetas as the new
      	 	sz.theta_prev = sz.theta_curr
		return np.array(diff)*sz.radian_const # convert ticks to radians




      

	def set_goal_xyz(sz,goal):
		#set goal in xyz goal in 1x3[x,y,z]
		sz.goal = goal
     

	def get_goal(sz):
		#get goal information
		return sz.goal


	def get_tether_goal(sz):
		#convert cartesian goal to tether length goal
		L_goal = sz.cart2tether(sz.goal,False)
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
        	
			L1 = np.linalg.norm(sz.p[1]-p0) - 0.0287
			L2 = np.linalg.norm(sz.p[2]-p0) - 0.0287
			L3 = np.linalg.norm(sz.p[3]-p0) - 0.0287
		elif col_fixed is False:
			#use x and y to solve for what z should be then solve for L
			z = (sz.L[0]**2-x**2-y**2)**0.5 #assume positive square root
			k = x**2+y**2+z**2 #radius of sphere
			p0 = [x,y,z]
			L0 = k**0.5 # should just be sz.L[0] if not there is a math mistake
			L1 = np.linalg.norm(sz.p[1]-p0) - 0.0287
			L2 = np.linalg.norm(sz.p[2]-p0) - 0.0287
			L3 = np.linalg.norm(sz.p[3]-p0) - 0.0287
		else:
			print 'error in cart2tether input for col_fixed'

		L = [L0,L1,L2,L3]
		return L
     
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
				time.sleep(0.25)
			#print "Sensor Readings:"
			#print l

			
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

	def slack_remove(sz, c):
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
			time.sleep(unwind_time)
			c.at.T2.set_torque(0)
			c.at.T3.set_torque(-0.1) #T3
			time.sleep(unwind_time)
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
				time.sleep(unwind_time)
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
				time.sleep(unwind_time)
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
				time.sleep(unwind_time)
				c.at.T1.set_torque_mx(0)
				c.at.T3.set_torque(-0.1) #T3
				time.sleep(unwind_time)
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
				time.sleep(unwind_time)
				c.at.T1.set_torque_mx(0)
				c.at.T2.set_torque(-0.1) #T2
				time.sleep(unwind_time)
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
		#STEP_3 Reset the accelerometer
		

	def measure_dtheta(sz, pos_new, pos_old):
		diff = pos_new - pos_old

		if abs(diff)>sz.limit_t:
			#calculate valid delta and switch signs to be correct direction
			if pos_new >= pos_old:# diff is +ve, so the solution should be -ve
				diff = ((sz.max_t-pos_new)+(pos_old-sz.min_t)+1)*(-1)
			else: # diff is negative, therfore the solution should be positive
				diff = ((sz.max_t-pos_old)+(pos_new-sz.min_t)+1)
		else:
			diff = diff #valid calculation
		return diff # convert ticks to radians

				


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

