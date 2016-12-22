#!/usr/bin/env python
'''
Set up and control a diff drive robot using a joystick

I recommend running this code in ipython via:
>> execfile('demo_diffdrive.py')
'''
# Import pygame, which handles joysticks and other inputs
import pygame
from pygame import joystick
from pygame import event
from pygame.locals import *
import ckbot.logical as L # CKBot interface
from time import time as now, sleep
from numpy import pi
import math
import sys
import numpy as np
import spiral_zipper_4M as sz
import ball_tracker as cm
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import collections

# Specify module ID and the name we want to give each of them:
modules = {0xE4: 'T1',0xEA:'T2',0xDF:'T3',0xF8:'T4',0xF1:'T5',0xF2:'T6', 0xE3:'T7',0x07:'T8'}

if __name__=="__main__":

	print "Initializing Spiral Zipper Demo"  

	##### INITIALIZE CKBOT CLUSTER ###########
	if len(sys.argv) == 2:
		modules = {int(sys.argv[1]) : 'T1', int(sys.argv[2]) : 'T2', int(sys.argv[3]) : 'T3',int(sys.argv[4]) : 'T4',int(sys.argv[5]) : 'T5',int(sys.argv[6]) : 'T6',int(sys.argv[7]) : 'T7',int(sys.argv[8]) : 'T8'}

	c = L.Cluster()
	c.populate( len(modules), modules )

	# Limit module speed and torque for safety
	torque_limit = 0.75
	for m in c.itermodules():
		m.set_torque_limit( torque_limit )
		# If module is a servo, then also limit speed
		if not m.get_mode():
			m.set_speed( 10.0 )

	##### INITIALIZE REMOTE CONTROLLER ######
	pygame.init()
	joystick.init()
	joystick.Joystick(0).init()

	drivable = True
	x1_move = 0 #joystick xy movement variables
	x2_move = 0
	y1_move = 0
	y2_move = 0

	z1_move = 0 #joystick z movement variables
	z2_move = 0
	z1_flag = False 
	z2_flag = False

	##### LIVE PLOT INITIALIZATION  #####
 	fig = plt.figure()
	ax = fig.add_subplot(111)

	y1array = collections.deque([None] * 200, maxlen=200)
	y2array = collections.deque([None] * 200, maxlen=200)
	y3array = collections.deque([None] * 200, maxlen=200)
	y1darray = collections.deque([None] * 200, maxlen=200)
	y2darray = collections.deque([None] * 200, maxlen=200)
	y3darray = collections.deque([None] * 200, maxlen=200)
	xarray = collections.deque([None] * 200, maxlen=200)

	l1, = ax.plot(xarray, y1array, 'r-', label = "sensed x")
	l2, = ax.plot(xarray, y2array, 'b-', label = "sensed y")
	l3, = ax.plot(xarray, y3array, 'g-', label = "sensed z")
	l1d, = ax.plot(xarray, y1darray, 'r--', label = "desired x")
	l2d, = ax.plot(xarray, y2darray, 'b--', label = "desired y")
	l3d, = ax.plot(xarray, y3darray, 'g--', label = "desired z")
	plt.legend(bbox_to_anchor=(.8, .9), loc=2, borderaxespad=0.)
	fig.canvas.draw()
	plt.show(block=False)

	########### INITIALIZE ARMS ############
	r_winch = 0.0199 #radius of the pulleys on the motors
	looptime = 0.1

	flag = 0 #toggles trajectory motion with joystick motion.  Need to get rid of.
	mode = 1  #State machine flag
	sz1 = sz.SpiralZipper(r_winch, c, looptime, 0)  #Arm Object Initialization
	sz2 = sz.SpiralZipper(r_winch, c, looptime, 1)

	#### SETS VELOCITY CONTROL GAINS ####
	Kp1 = 20 #proportional gains
	Kp2 = 5
	Kp3 = 5
	Kp = [Kp1,Kp2,Kp3]
    
	Ki1 = 1.25  #integral gains							
	Ki2 = 1.
	Ki3 = 1.
	Ki = [Ki1,Ki2,Ki3]

	Kd1 = 0#.005
	Kd2 = 0#.005
	Kd3 = 0#.005
	Kd = [Kd1,Kd2,Kd3]

	sz1.set_Kp_Gains(Kp,0)
	sz1.set_Ki_Gains(Ki,0)
	sz1.set_Kd_Gains(Kd,0)

	sz2.set_Kp_Gains(Kp,0)
	sz2.set_Ki_Gains(Ki,0)
	sz2.set_Kd_Gains(Kd,0)

	#### SETS POSITION CONTROL GAINS ####
	Kp1 = 300 #proportional gains
	Kp2 = 25
	Kp3 = 25
	Kp = [Kp1,Kp2,Kp3]
    
	Ki1 = 5  #integral gains
	Ki2 = 10.
	Ki3 = 10.
	Ki = [Ki1,Ki2,Ki3]

	Kd1 = 1.5
	Kd2 = 6
	Kd3 = 6
	Kd = [Kd1,Kd2,Kd3]
	sz1.set_Kp_Gains(Kp,1)
	sz1.set_Ki_Gains(Ki,1)
	sz1.set_Kd_Gains(Kd,1)

	sz2.set_Kp_Gains(Kp,1)
	sz2.set_Ki_Gains(Ki,1)
	sz2.set_Kd_Gains(Kd,1)

	#### SETS JOYSTICK INPUT GAINS ####
	Kx = 0.01 
	Ky = 0.01
	Kz = 0.02

	#randompoint = cm.camera()
	#print "the camera's target is :" 
	#print randompoint
	#end_effector_old = np.matrix([[.25],[0],[.4]])
	#end_effector_des = np.matrix([[.25],[0],[.4]])
	#sz1.slack_removal(c)

	print "Spiral Zipper demo initialized"
	old_report = now()

	#################### MAIN LOOP #########################
	while True: 
		try:
			time_elapsed = now() - old_report
			if time_elapsed >= looptime:
				print "loop time is :" 
				print time_elapsed
				old_report = now()
				################### Joystick Input Case Structure ##################
				for evt in event.get(): 
					if evt.type == JOYAXISMOTION:   # Controller Joystick Inputs. Sets inputs to the Velocity Controller
						if evt.axis == 0:
							x1_move = evt.value    #arm 1. xy joystick control inputs
 						elif evt.axis == 1:
							y1_move = evt.value
 						elif evt.axis == 2:
							x2_move = evt.value    #arm 2. xy joystick control inputs
 						elif evt.axis == 3:
							y2_move = evt.value

					elif evt.type == JOYBUTTONDOWN:  #Controller Button inputs. Sets inputs to Position Controller. Toggles between V and P control.
						if evt.button is 0: #toggles arm between Velocity to Position Control.
							mode = not mode 
							if mode == 1:
								#target_pos = np.matrix([[.25], [0.00], [.35]])
								sz2.update_goal([0.05,0.1,0.30],mode) 
								sz2.errorsumP = [0,0,0,0]
							if mode == 0:
								sz2.errorsumV = [0,0,0,0]
								sz2.update_goal([0,0,0],mode)

						elif evt.button is 1: #Reinitializes outer loop control using the IMU.
							mode = 1
							#sz1.slack_removal(c)
							rotation = sz2.get_sensor_readings(1)
							#sz1.L[0] = sz1.sensed_lidar(0)
							sz2.sensed_pos = sz2.rotate(rotation[0],rotation[1],sz2.L[0])
							sz2.L = sz2.cart2tether_actual(sz2.sensed_pos)  #tether length update based on IMU only
							sz2.goal = sz2.sensed_pos
							sz2.sensed_pos_prev = sz2.goal
							mode = 1

						elif evt.button is 2: #sets a desired position in Position control mode
							mode = 1 
							sz2.update_goal([-.1, 0, .35],mode) 
							#target_pos = np.matrix([[.25], [0.05], [.45]])
							#yaw_des = 0
							#pitch_des = 0

						elif evt.button is 3: #sets a desired position in Position control mode
							initial_time = now()
							flag = not flag
							#sz2.update_goal([.1, 0, .35],mode) 
							#target_pos = np.matrix([[.35], [-.05], [.35]])
							#yaw_des = 0
							#pitch_des = -.1

						elif evt.button is 4: #arm 1 up in Velocity control mode
							z1_flag = not z1_flag
							if z1_flag:
								z1_move = 1
							else:
								z1_move = 0

						elif evt.button is 6: #arm 1 down in Velocity control mode
							z1_flag = not z1_flag
							if z1_flag == True:
								z1_move = -1
							else:
								z1_move = 0

						elif evt.button is 5: #arm 2 up in Velocity control mode
							z2_flag = not z2_flag
							if z2_flag:
								z2_move = 1
							else:
								z2_move = 0

						elif evt.button is 7: #arm 2 down in Velocity control mode
							z2_flag = not z2_flag
							if z2_flag == True:
								z2_move = -1
							else:
								z2_move = 0

						elif evt.button is 8: #sets a desired position in Position control mode
							mode = 1
							sz2.update_goal([0, -0.1,0.35],mode) 

						elif evt.button is 9: #sets a desired position in Position control mode
							mode = 1 
							sz2.update_goal([0, 0.1,0.35],mode) 

						else:
							raise KeyboardInterrupt
				if drivable:
					#sz1.update_state(c,0)  #gets tether positions and velocities from encoders and IMU
					sz2.update_state(c,1)
					#sz1.Jacobian_Math()

					######################## ROBOT STATE MACHINE ##############################
					if mode == 0: #Velocity control 
						
						if flag ==1: #BAD AND EVIL FLAG.  set flag bypasses user input velocity commands for a set path using the velocity controller
							sz2.trajectory(initial_time)
						else :
							#sz1.update_goal([-x1_move*Kx ,y1_move*Ky, z1_move*Kz], mode)  #sets a desired velocity for the system in xyz				
							sz2.update_goal([-x2_move*Kx,y2_move*Ky,z2_move*Kz],mode)
						remote_or_auto = 0  	#flag toggles features in the functions called below that are calibrated to the two different controllers

					elif mode == 1: #Position control: 0 position
						yaw_des = 0
						pitch_des = 0
						#end_effector_des = target_pos
					elif mode == 2: #Camera Tracking mode.  Arm will follow target object and attempt to grab it. Not useable currently
						yaw_des = 0
						pitch_des = 0
						end_effector_des = cm.camera()	
						end_effector_des[0] = end_effector_des[0] + .02
						end_effector_des[1] = end_effector_des[1]# - .01
						end_effector_des[2] = end_effector_des[2]# + .01

						if abs(np.linalg.norm(end_effector_des - end_effector_old)) > 1: #throws away target positions too far away from the previously camera value
							end_effector_des = end_effector_old
						if abs(np.linalg.norm(end_effector_des - end_effector_old)) < .12: #throws away target positions too close  to the previous camera value
							end_effector_des = end_effector_old
						if abs(np.linalg.norm(end_effector_des - end_effector_pos)) < .02: #conditions that determines whether the arm has successfully grabbed an object
							mode = 3
							sz1.vacuum_cleaner()
						#print "end effector desired is :"
						#print end_effector_des

					elif mode == 3: #Grab and move object mode.  Arm grabs an object and moves it to a set location
						end_effector_des = np.matrix([[.35], [0.15], [.45]])
						if abs(np.linalg.norm(end_effector_des - end_effector_pos)) < .01: #conditions that determines whether the arm has successfully grabbed an object
							mode = 4
							sz1.vacuum_cleaner()

					elif mode == 4: #Drop object at target location. Return to zero position
						end_effector_des = np.matrix([[.40], [0.20], [.45]])
						yaw = .1
						pitch = -.1
						if abs(np.linalg.norm(end_effector_des - end_effector_pos)) < .01: #conditions that determines whether the arm has successfully grabbed an object
							mode = 1

					elif mode == 5: #Drop object at target location. Return to upright high position
						end_effector_des = np.matrix([[.25], [0.0], [.45]])
						yaw = 0
						if abs(np.linalg.norm(end_effector_des - end_effector_pos)) < .005: #conditions that determines whether the arm has successfully grabbed an object
							mode = 1
					
					if mode != 0: #When arm is in Velocity mode
						#arm_positions = sz1.end_effector_goal(end_effector_des, yaw_des, pitch_des) #commands to be sent to the controller
						#a = [arm_positions[0], arm_positions[1], arm_positions[2]]
						#b = [arm_positions[3], arm_positions[4], arm_positions[5]]
						#sz1.update_goal(a,1) #outputs desired position commands to the control system
						#sz2.update_goal(b,1)
						#print "end_effector goal is : "
						#print end_effector_des	
						remote_or_auto = 1		

					if flag == 0: #BAD AND EVIL FLAG. Flag is only 1 when the system is in trajectory mode.
						#sz1.set_tether_speeds(mode) #function that transforms the xyz desired vel into a set of tether velocities
						sz2.set_tether_speeds(mode) 

					#sz1.actuate_Motors(c,0,remote_or_auto) #function that transforms desired tether velocities into motor commands
					sz2.actuate_Motors(c,1,remote_or_auto) 
					
					##### DATA MONITORING #########'
					# prints important robot data to the screen for monitoring purposes. Uncomment as needed

					#A = sz1.sensed_pos + [0, 0, 0.09]  #end joint of arm 1
					#B = sz2.sensed_pos - [.51, 0, 0] + [0, 0, .09] #end joint of arm 2
					#print A
					#print B
					#end_effector_pos = sz1.end_effector_position(A,B) #end effector
					#end_effector_old = end_effector_des

					rotation = sz2.get_sensor_readings(1) 
					sensed_pos = sz2.rotate(rotation[0],rotation[1],sz2.L[0])
					print "position is "
					print sensed_pos

					#print "end effector position is : "
					#print end_effector_pos
					# print "goal is"
					# print sz1.get_goal()
					#print sz2.get_goal()

					# print "current tether lengths : "
					# print sz1.L
					#print sz2.L

					# print "arm tether velocities: "
					# print sz1.L_vel_actual
					#print sz2.L_vel_actual
					#print "arm 1 desired tether velocities: "			
					#print sz1.L_vel_desired
					#print sz2.L_vel_desired

					# Torques = [c.at.T1.get_torque(), 
					# 	   c.at.T2.get_torque(), 
					# 	   c.at.T3.get_torque()]

					# print "Torques are: "
					# print Torques

					########## Graphing ###########
					# updates liveplots based on IMU data and desired inputs.
					xarray.append(now())  
					y1array.append(sensed_pos[0])
					y2array.append(sensed_pos[1])
					y3array.append(sensed_pos[2])
					y1darray.append(sz2.goal[0])
					y2darray.append(sz2.goal[1])
					y3darray.append(sz2.goal[2])
					'''y1array.append(sz1.L_vel_actual[1])
					y2array.append(sz1.L_vel_actual[2])
					y3array.append(sz1.L_vel_actual[3])
					y1darray.append(sz1.L_vel_desired[1])
					y2darray.append(sz1.L_vel_desired[2])
					y3darray.append(sz1.L_vel_desired[3])'''
					if len(xarray) > 200:
						xarray.popleft()
						y1array.popleft()
						y2array.popleft()
						y3array.popleft()
						y3darray.popleft()
						y2darray.popleft()
						y1darray.popleft()
					l1.set_xdata(xarray)
					l1.set_ydata(y1array)
					l2.set_xdata(xarray)
					l2.set_ydata(y2array)
					l3.set_xdata(xarray)
					l3.set_ydata(y3array)

					l1d.set_xdata(xarray)
					l1d.set_ydata(y1darray)
					l2d.set_xdata(xarray)
					l2d.set_ydata(y2darray)
					l3d.set_xdata(xarray)
					l3d.set_ydata(y3darray)

					ax.relim() 
					ax.autoscale_view(True,True,True)
					fig.canvas.draw()
				else:
					for m in c.itermodules():
						m.set_torque(0) #not go_slack - causes arm to collapse
		#send 
		except KeyboardInterrupt or ValueError:
			# Break out of the loop
			print "Keyboard Interrupt detected"
			break
		'''except Exception,e:

			print "\n\nERROR DETECTED."
			print str(e)
			print "\n\n"
			sleep(1)
			# Break out of the loop
			pass
			#break'''

	#np.savetxt('/home/modlab/Desktop/data.txt', big_list, delimiter=' / ')
	print "Demo exiting, turning off all modules"
	# Turn the modules off before we exit for safety
	for m in c.itermodules():
		m.set_torque(0)

	print "Closing all data files..."
	sz1.data_store_measurement.close()
	sz1.data_store_estimate.close()
	sz1.data_store_predict.close()