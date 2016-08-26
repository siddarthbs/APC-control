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
import spiral_zipper_k as sz

# Specify module ID and the name we want to give each of them:
modules = {0xDF:'T2',0xE6:'T3'}

if __name__=="__main__":

    import sys

    print "Initializing Spiral Zipper Demo"  
    # Initialize pygame and a joystick
    pygame.init()
    joystick.init()

    joystick.Joystick(0).init()

    # Optionally specify left and right modules through the command line.
    
    if len(sys.argv) == 2:
        modules = {int(sys.argv[1]) : 'T2', int(sys.argv[2]) : 'T3'}

    # Create a CKBot cluster
    c = L.Cluster()
    
    c.populate( len(modules), modules )

    # Limit modufle speed and torque for safety
    torque_limit = 0.5
    for m in c.itermodules():
        m.set_torque_limit( torque_limit )
        # If module is a servo, then also limit speed
        if not m.get_mode():
            m.set_speed( 10.0 )

    # init spiral zipper TODO: set these to be closer to actual
    #Lengths = [0.1,0.1,0.1,0.1]
    start_pos = [0,0,0.3175]
    current_goal=start_pos
    r_winch = 0.02

    Motor_gain = 3.0
    #K1 = 0.1 * Motor_gain  # P- controller gain    MX CKbot requires less gain
    K2 = 1.5 * Motor_gain  # ^
    K3 = 2.0 * Motor_gain  # this module seems to be lagging behind
    K = [K2, K3]

    sz1 = sz.SpiralZipper(start_pos, r_winch, c)
    

    Kx = 0.005 #joystick sensitivity convert input into how much x to add
    Ky = 0.005 #^
    #gains to tune error and compensate for each module ability
    # be careful not to make these too large as the torque may saturate and the controller will not work

    print "Motor Gains SpiralJoy :"
    print K


    trial=1.5 #Common multiplier for Ki
    #Integral Control coefficients
    Ki_1 = 0.005*trial
    Ki_2 = 0.012*trial
    Ki_3 = 0.015*trial
    Ki_4 = 0.027
    #Ki_5 = 0.035
    Ki = [Ki_1, Ki_2, Ki_3, Ki_4]

    #sz1.set_Integral_Gains([Ki_1, Ki_2, Ki_3, Ki_4])

    #Integral control accuracy thresholds
    Ki_1_Threshold=0.02 # error at which integral control is activated
    Ki_2_Threshold = 0.01
    Ki_3_Threshold = 0.005
    Ki_4_Threshold=0.003
    Ki_5_Threshold = 0.001
    #Ki_6_Threshold = 0.0005
    Ki_Thresholds= [Ki_1_Threshold, Ki_2_Threshold, Ki_3_Threshold, Ki_4_Threshold, Ki_5_Threshold]
    sz1.set_Integral_Data(Ki, Ki_Thresholds)
    #sz1.set_Integral_Thresholds([Ki_1_Threshold, Ki_2_Threshold, Ki_3_Threshold, Ki_4_Threshold, Ki_5_Threshold])


    print "Spiral Zipper demo initialized"
    drivable = False
    drive = 0.0
    turn = 0.0
    driveGain = 0.5
    turnGain = 0.4
    last_report = now()
    report_interval = 10
    x_move = 0
    y_move = 0

   # data = open('/home/modlab/Desktop/data.txt','w')
    #big_list = np.array([[1, 2, 3]])

    #TODO: put this loop on a timer
    while True:
        try:
            sleep(0.1)  # set loop rate
            # A 5 cell lipo shouldn't be used below 16 V.
            # Check to make sure the voltage is still good.
            if now()-last_report > report_interval:
                batt_v = c.at.T2.get_voltage()
                last_report = now()
                #if batt_v < 10: # EDIT: set below 16 if using wired power
                #    print 'Battery voltage less than 16V, please charge battery. Shutting off robot.'
                #    break
                #else:
                #    print 'Battery voltage: %2.2f V' % batt_v
            # Get and handle events
            for evt in event.get():
                if evt.type == JOYAXISMOTION:   
                    # Look at joystick events and control values
                    if evt.axis == 2:
                        x_move = evt.value
                        #print "X motion: %f" % x_move
                    elif evt.axis == 1:
                        y_move = evt.value
                        #print "Y motion: %f" % y_move
                elif evt.type == JOYBUTTONDOWN:
                # Pressing controller's main buttons toggles drivability
                    if evt.button is 4:
                        drivable = not drivable
                        print 'Drivability status: %s' % str(drivable)
                        print evt.button
                        if(drivable):
                            '''target_achieved = [0, 0, 0] #Reset target achieved
                            goal_start=now()
                            target_reached = False #Reset target reached
                        # Pressing the trigger buttons ends demo.'''
                    elif evt.button is 0:
                        print "Position 1"
                        #errorsum=[0,0,0]
                        sz1.set_goal_xyz(sz1.rotate(-4.0200, 0.0800))
                        '''goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached=False'''

                    elif evt.button is 1:
                        print "Position 2"
                        #errorsum = [0, 0, 0]
                        sz1.set_goal_xyz(sz1.rotate(-0.0200, -3.9900))
                        '''goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached = False'''

                    elif evt.button is 2:
                        print "Position 3"
                        #errorsum = [0, 0, 0]
                        sz1.set_goal_xyz(sz1.rotate(4.1200, 0.1300))
                        '''goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached = False'''

                    elif evt.button is 3:
                        print "Position 4"
                        #errorsum = [0, 0, 0]
                        sz1.set_goal_xyz(sz1.rotate(-0.0200, 4.1400))
                        '''goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached = False'''

                    elif evt.button is 6:
                        was_true=drivable
                        drivable=False
                        print "Requested slack removal..."
                        #sz1.slack_remove(c)
                        drivable=was_true
                    else:
                        raise KeyboardInterrupt
            if drivable:
		
                # Set the torque values on the CR modules based on the 
                # inputs that we got from the joystick 
                #c.at.T1.set_torque( -(drive*driveGain - turn*turnGain) )
                #c.at.T2.set_torque(  drive*driveGain + turn*turnGain )
                #print "\n"*5

                #winchforce_1 = c.at.T1.get_winchforce()[0] 
                #winchforce_2 = c.at.T2.get_winchforce()[0]
                #winchforce_3 = c.at.T3.get_winchforce()[0]
                #winch_force_list = [winchforce_1, winchforce_2, winchforce_3]
                #big_list = np.append(big_list, [winch_force_list], axis=0)


                #sz1.update_goal([x_move*Kx,y_move*Ky,0.0])
                print "goal is"
                print sz1.get_goal()
		
                #sz1.update_state (c)
                """print "Theta_1 is %f" %sz1.theta_curr[0]
                print "Theta_2 is %f" %sz1.theta_curr[1]
                print "Theta_3 is %f" %sz1.theta_curr[2]"""
                #print "xyz error:"
                #print sz1.get_error_xyz()
                # controller for moving CkBots.
                #deltas_tether = sz1.get_error_state()
                #print "delta tether lengths:"
                #print deltas_tether #to debug


                sz1.set_Motor_Gains(K)
                t = sz1.PI_control(c)


                print "cmd torques"
                #print "%f" %t[0]
                print "%f" %t[0]
                print "%f" %t[1]
                print "current position is:"		
                print sz1.get_xyz_pos()

                sz1.actuate_Motors(c,t)


                #print 'Tor1 %f' %c.at.T1.get_torque()
                print 'Tor2 %f' %c.at.T2.get_torque()
                print 'Tor3 %f' %c.at.T3.get_torque()

                #print sz1.get_state
            else:
                for m in c.itermodules():
                    m.set_torque(0) #may want to change this to torque is zero
                #send 
        except KeyboardInterrupt or ValueError:
            # Break out of the loop
            print "Keyboard Interrupt detected"
            break
        except Exception,e:

            print "\n\nERROR DETECTED."
            print str(e)
            print "\n\n"
            sleep(1)
            # Break out of the loop
            pass
            #break

    #np.savetxt('/home/modlab/Desktop/data.txt', big_list, delimiter=' / ')
    print "Demo exiting, turning off all modules"
    # Turn the modules off before we exit for safety
    for m in c.itermodules():
        m.go_slack()


#TODO
def trajectory(final_goal):
    # Calculate length of trajectory to final goal
    quantize=2
    position_grav = sz1.get_sensor_readings()
    position_sensed=sz1.rotate(position_grav[0],position_grav[1])
    distance = np.linalg.norm(position_sensed - final_goal)
    len_intermediates=int(distance/quantize)






# Determine No' of mini-trajectories required to reach final goal
# determine array of intermediate goals
# in main while loop:
# goal=current goal.
# check error state
# 	if error<threshold: update goal.



#machine 1 replaced with MX CKbot.
#minor tweaks in torque values is required for robustness in manipulation.
