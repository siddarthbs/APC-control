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
import numpy as np
import spiral_zipper_auto as sz
import ball_tracker as cm

# Specify module ID and the name we want to give each of them:
modules = {0xE4: 'T1',0xEA:'T2',0xDF:'T3',0xF8:'T4',0xF1:'T5',0xF2:'T6'}

if __name__=="__main__":

    import sys

    print "Initializing Spiral Zipper Demo"  
    # Initialize pygame and a joystick
    pygame.init()
    joystick.init()

    joystick.Joystick(0).init()

    # Optionally specify left and right modules through the command line.
    
    if len(sys.argv) == 2:
        modules = {int(sys.argv[1]) : 'T1', int(sys.argv[2]) : 'T2', int(sys.argv[3]) : 'T3',int(sys.argv[4]) : 'T4',int(sys.argv[5]) : 'T5',int(sys.argv[6]) : 'T6'}

    # Create a CKBot cluster
    c = L.Cluster()
    
    c.populate( len(modules), modules )

    # Limit module speed and torque for safety
    torque_limit = 0.75
    for m in c.itermodules():
        m.set_torque_limit( torque_limit )
        # If module is a servo, then also limit speed
        if not m.get_mode():
            m.set_speed( 10.0 )

    start_pos = [0,0,0.30]
    current_goal = start_pos
    r_winch = 0.022 #radius of the pulleys on the motors
    looptime = .1
    sz1 = sz.SpiralZipper(start_pos, r_winch, c, looptime, 0)
    sz2 = sz.SpiralZipper(start_pos, r_winch, c, looptime, 1)

    ### Gains for Remote Control ####
    Kp1 = 8 #proportional gains
    Kp2 = 4
    Kp3 = 4
    Kp = [Kp1,Kp2,Kp3]
    
    Ki1 = 10  #integral gains
    Ki2 = 2
    Ki3 = 2
    Ki = [Ki1,Ki2,Ki3]

    sz1.set_Kp_Gains(Kp,0)
    sz1.set_Ki_Gains(Ki,0)

    sz2.set_Kp_Gains(Kp,0)
    sz2.set_Ki_Gains(Ki,0)
    #### Gains for Auto Control ####
    Kp1 = 3 #proportional gains
    Kp2 = 1
    Kp3 = 1
    Kp = [Kp1,Kp2,Kp3]
    
    Ki1 = .5  #integral gains
    Ki2 = .2
    Ki3 = .2
    Ki = [Ki1,Ki2,Ki3]

    sz1.set_Kp_Gains(Kp,1)
    sz1.set_Ki_Gains(Ki,1)

    sz2.set_Kp_Gains(Kp,1)
    sz2.set_Ki_Gains(Ki,1)
    #### Gains for Joystick #####
    Kx = 0.025 
    Ky = 0.025 
    Kz = 0.0025
    
    print "Spiral Zipper demo initialized"
    drivable = True
    z1_flag = False
    z2_flag = False
    mode = 0  #State machine flag
    old_report = now()
    x1_move = 0
    x2_move = 0
    y1_move = 0
    y2_move = 0
    z1_move = 0
    z2_move = 0
    randompoint = cm.camera()
    print "the camera's target is :" 
    print randompoint
    end_effector_old = np.array([.25,0,.4])
    end_effector_des = np.array([.25,0,.4])
   # data = open('/home/modlab/Desktop/data.txt','w')
    #big_list = np.array([[1, 2, 3]])

    while True:
        try:
	    time_elapsed = now() - old_report

            if time_elapsed >= looptime:
       	        print "loop time is :" 
       	        print time_elapsed
            	old_report = now()
                for evt in event.get():
                	if evt.type == JOYAXISMOTION:   
                 	   # Look at joystick events and control values
		    		drivable = 1
				if evt.axis == 0:
					x1_move = evt.value
 			        elif evt.axis == 1:
					y1_move = evt.value
 			        elif evt.axis == 2:
					x2_move = evt.value
 			        elif evt.axis == 3:
					y2_move = evt.value
                	elif evt.type == JOYBUTTONDOWN:
                		# Pressing controller's main buttons toggles drivability
				if evt.button is 0: #toggles arm from remote control into autonomous control mode
					if mode == 0: 
						mode = 1
						target_pos = np.array([.25, 0.00, .35])
					else:
						mode = 0
				elif evt.button is 1: #switches arm into camera tracking mode
					mode = 2 
				elif evt.button is 2: #sets a desired position in auto control mode
					mode = 1 
					target_pos = np.array([.25, 0.05, .45])
					yaw_des = 0
					pitch_des = 0
				elif evt.button is 3: #sets a desired position in auto control mode
					mode = 1
					target_pos = np.array([.35, -.05, .35])
					yaw_des = 0
					pitch_des = -.1

                    		elif evt.button is 4: #arm 1 up in remote control mode
					z1_flag = not z1_flag
					if z1_flag:
						z1_move = 1
					else:
						z1_move = 0
       		    		elif evt.button is 6: #arm 1 down in remote control mode
					z1_flag = not z1_flag
					if z1_flag == True:
						z1_move = -1
					else:
						z1_move = 0
                    		elif evt.button is 5: #arm 2 up in remote control mode
					z2_flag = not z2_flag
					if z2_flag:
						z2_move = 1
					else:
						z2_move = 0
       		    		elif evt.button is 7: #arm 2 down in remote control mode
					z2_flag = not z2_flag
					if z2_flag == True:
						z2_move = -1
					else:
						z2_move = 0
				elif evt.button is 8: #toggles end effector power
					sz1.vacuum_cleaner()  
                                else:
            		            raise KeyboardInterrupt
            	if drivable:
			#print "xmove : %f" %x_move
			#print "ymove : %f" %y_move
			#print "zmove : %f" %z_move
			
			sz1.update_state(c,0)  #gets tether positions and velocities from encoders and IMU
			sz2.update_state(c,1)

			#The robot state machine
			if mode == 0: #remote control 
				sz1.update_goal([-x1_move*Kx ,y1_move*Ky, z1_move*Kz], mode)  #input sets a desired velocity for the system in xyz				
				sz2.update_goal([-x2_move*Kx,y2_move*Ky,z2_move*Kz],mode)
				remote_or_auto = 0

			elif mode == 1: #autonomous control: 0 position
				yaw_des = 0
				pitch_des = 0
				end_effector_des = target_pos

			elif mode == 2: #Camera Tracking mode.  Arm will follow target object and attempt to grab it
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
				print "end effector desired is :"
				print end_effector_des

			elif mode == 3: #Grab and move object mode.  Arm grabs an object and moves it to a set location
				end_effector_des = np.array([.35, 0.15, .45])
				if abs(np.linalg.norm(end_effector_des - end_effector_pos)) < .01: #conditions that determines whether the arm has successfully grabbed an object
					mode = 4
					sz1.vacuum_cleaner()

			elif mode == 4: #Drop object at target location. Return to zero position
				end_effector_des = np.array([.40, 0.20, .45])
				yaw = .1
				pitch = -.1
				if abs(np.linalg.norm(end_effector_des - end_effector_pos)) < .01: #conditions that determines whether the arm has successfully grabbed an object
					mode = 1

			elif mode == 5: #Drop object at target location. Return to upright high position
				end_effector_des = np.array([.25, 0.0, .45])
				yaw = 0
				if abs(np.linalg.norm(end_effector_des - end_effector_pos)) < .005: #conditions that determines whether the arm has successfully grabbed an object
					mode = 1
			
			if mode != 0: #When arm is in auto mode
				arm_positions = sz1.end_effector_goal(end_effector_des, yaw_des, pitch_des) #commands to be sent to the controller
				a = [arm_positions[0], arm_positions[1], arm_positions[2]]
				b = [arm_positions[3], arm_positions[4], arm_positions[5]]
				sz1.update_goal(a,1) #outputs desired position commands to the control system
				sz2.update_goal(b,1)
				print "end_effector goal is : "
				print end_effector_des	
				remote_or_auto = 1				

			sz1.set_tether_speeds() #function that transforms the xyz desired vel into a set of tether velocities
			sz2.set_tether_speeds() #function that transforms the xyz desired vel into a set of tether velocities

			sz1.actuate_Motors(c,0,remote_or_auto) #function that transforms desired tether velocities into motor commands
			sz2.actuate_Motors(c,1,remote_or_auto) #function that transforms desired tether velocities into motor commands
			
			
                	A = sz1.sensed_pos + [0, 0, 0.09]  #end joint of arm 1
			B = sz2.sensed_pos - [.51, 0, 0] + [0, 0, .09] #end joint of arm 2
			print A
			print B
			end_effector_pos = sz1.end_effector_position(A,B) #end effector
			end_effector_old = end_effector_des

			print "end effector position is : "
			print end_effector_pos
			print "goal is"
                	print sz1.get_goal()
			print sz2.get_goal()

			print "current tether lengths : "
			print sz1.L
			print sz2.L

			print "arm tether velocities: "
			print sz1.L_vel_actual
			print sz2.L_vel_actual
			print "arm 1 desired tether velocities: "			
			print sz1.L_vel_desired
			print sz2.L_vel_desired

			'''Torques = [c.at.T1.get_torque(), 
				   c.at.T2.get_torque(), 
				   c.at.T3.get_torque()]

                	print "Torques are: "
			print Torques'''
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

