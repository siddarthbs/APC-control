#!/usr/bin/env python
'''
Set up and control a diff drive robot using a joystick

I recommend running this code in ipython via:
>> execfile('demo_diffdrive.py')
'''

import ckbot.logical as L # CKBot interface
# Import pygame, which handles joysticks and other inputs
import pygame
from pygame import joystick
from pygame import event
from pygame.locals import *
from time import time as now, sleep
from numpy import pi
import math
import numpy as np
import spiral_zipper as sz

# Specify module ID and the name we want to give each of them:
modules = { 0x46: 'T1', 0xDF: 'T2',0xE6:'T3' } 

if __name__=="__main__":

    import sys

    print "Initializing Spiral Zipper Demo"  
    # Initialize pygame and a joystick
    pygame.init()
    joystick.init()
    joystick.Joystick(0).init()

    # Optionally specify left and right modules through the command line.
    
    if len(sys.argv) == 3:
        modules = { int(sys.argv[1]) : 'T1', int(sys.argv[2]) : 'T2', int(sys.argv[3]) : 'T3' }

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
    r_winch = 0.044

    sz1 = sz.SpiralZipper(start_pos, r_winch, c)
    

    Kx = 0.005 #joystick sensitivity convert input into how much x to add
    Ky = 0.005 #^
    #gains to tune error and compensate for each module ability
    # be careful not to make these too large as the torque may saturate and the controller will not work

    K1 = 0.5 # P- controller gain    MX CKbot requires less gain
    K2 = 1.5 #^
    K3 = 2.0 # this module seems to be lagging behind
    trial=1.5 #Common multiplier for Ki
    #Integral Control coefficients
    Ki_1 = 0.005*trial
    Ki_2 = 0.012*trial
    Ki_3 = 0.015*trial
    Ki_4 = 0.023
    #Integral control accuracy thresholds
    Ki_1_Threshold=0.02 # error at which integral control is activated
    Ki_2_Threshold = 0.01
    Ki_3_Threshold = 0.005
    Ki_4_Threshold=0.003
    Ki_5_Threshold = 0.001
    errorsum=[0,0,0]
    deltas_tether_prev = [0, 0, 0] #initialize for differential control
    Kd=0.00 #0.05 #Differential Control coefficient
    time_prev=now()

    #Keeping track of time to quantify time taken to reach goal
    goal_start = now()
    goal_stop = now()
    target_achieved=[0, 0, 0] #Truth values for each cartesian target achieved
    target_reached = False #Truth value for overall target reached


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
    
    

    #TODO: put this loop on a timer
    while True:
        sleep(0.1)# set loop rate
        try:
            # A 5 cell lipo shouldn't be used below 16 V.
            # Check to make sure the voltage is still good.
            if now()-last_report > report_interval:
                batt_v = c.at.T1.get_voltage()
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
                            target_achieved = [0, 0, 0] #Reset target achieved
                            goal_start=now()
                            target_reached = False #Reset target reached
                        # Pressing the trigger buttons ends demo.
                    elif evt.button is 0:
                        print "Position 1"
                        errorsum=[0,0,0]
                        sz1.set_goal_xyz(sz1.rotate(-4.0200, 0.0800))
                        goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached=False

                    elif evt.button is 1:
                        print "Position 2"
                        errorsum = [0, 0, 0]
                        sz1.set_goal_xyz(sz1.rotate(-0.0200, -3.9900))
                        goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached = False

                    elif evt.button is 2:
                        print "Position 3"
                        errorsum = [0, 0, 0]
                        sz1.set_goal_xyz(sz1.rotate(4.1200, 0.1300))
                        goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached = False

                    elif evt.button is 3:
                        print "Position 4"
                        errorsum = [0, 0, 0]
                        sz1.set_goal_xyz(sz1.rotate(-0.0200, 4.1400))
                        goal_start = now()
                        target_achieved = [0, 0, 0]
                        target_reached = False

                    elif evt.button is 6:
                        was_true=drivable
                        drivable=False
                        print "Requested slack removal..."
                        sz1.slack_remove(c)
                        drivable=was_true
                    else:
                        raise KeyboardInterrupt
            if drivable:
		
                # Set the torque values on the CR modules based on the 
                # inputs that we got from the joystick 
                #c.at.T1.set_torque( -(drive*driveGain - turn*turnGain) )
                #c.at.T2.set_torque(  drive*driveGain + turn*turnGain )
                #print "\n"*5
                sz1.update_goal([x_move*Kx,y_move*Ky,0.0])
                print "goal is"
                print sz1.get_goal()
		
                sz1.update_state (c)
                """print "Theta_1 is %f" %sz1.theta_curr[0]
                print "Theta_2 is %f" %sz1.theta_curr[1]
                print "Theta_3 is %f" %sz1.theta_curr[2]"""
                #print "xyz error:"
                #print sz1.get_error_xyz()
                # controller for moving CkBots.
                deltas_tether = sz1.get_error_state()
                #print "delta tether lengths:"
                #print deltas_tether #to debug

                print "Errors are:"
                print deltas_tether
                dt = now()-time_prev
                time_prev=now()


                K = [K1, K2, K3] #Motor gains

                #Integral Control for tether 1
                if(abs(deltas_tether[0])<Ki_1_Threshold and abs(deltas_tether[0])>Ki_2_Threshold): #Integral Control Accuracy level 1
                    Ki_t1 = Ki_1
                    print " Integral Control Tether 1 level 1"
                    errorsum[0] = errorsum[0] + deltas_tether[0]
                    target_achieved[0] = 0

                elif(abs(deltas_tether[0])<= Ki_2_Threshold and abs(deltas_tether[0])>Ki_3_Threshold): #Integral Control Accuracy level 2
                    Ki_t1 = Ki_2
                    print " Integral Control Tether 1 level 2"
                    errorsum[0] = errorsum[0] + deltas_tether[0]
                    target_achieved[0] = 0

                elif (abs(deltas_tether[0]) <= Ki_3_Threshold and abs(deltas_tether[0]) > Ki_4_Threshold): #Integral Control Accuracy level 3
                    Ki_t1 = Ki_3
                    print " Integral Control Tether 1 level 3"
                    errorsum[0] = errorsum[0] + deltas_tether[0]
                    target_achieved[0] = 0

                elif (abs(deltas_tether[0]) <= Ki_4_Threshold and abs(deltas_tether[0]) > Ki_5_Threshold): #Integral Control Accuracy level 4
                    Ki_t1 = Ki_4
                    print " Integral Control Tether 1 level 4 (Target Acquired)"
                    errorsum[0] = errorsum[0] + deltas_tether[0]
                    target_achieved[0]=1 #Assume target achieved


                elif (abs(deltas_tether[0]) <= Ki_5_Threshold and abs(deltas_tether[0]) >= 0): #Integral Control Accuracy level 5
                    Ki_t1 = 0
                    print "Target acquired tether 1"
                    errorsum[0] = 0
                    K[0] = 0
                    target_achieved[0] = 1 #Assume target achieved

                else: #No Integral Control yet
                    Ki_t1 = 0
                    print " No integral tether 1"
                    target_achieved[0] = 0

                #Integral Control for tether 2
                if (abs(deltas_tether[1]) < Ki_1_Threshold and abs(deltas_tether[1]) > Ki_2_Threshold):
                    Ki_t2 = Ki_1
                    print " Integral Control Tether 2 level 1"
                    errorsum[1] = errorsum[1] + deltas_tether[1]
                    target_achieved[1] = 0

                elif (abs(deltas_tether[1]) <= Ki_2_Threshold and abs(deltas_tether[1]) > Ki_3_Threshold):
                    Ki_t2 = Ki_2
                    print " Integral Control Tether 2 level 2"
                    errorsum[1] = errorsum[1] + deltas_tether[1]
                    target_achieved[1] = 0

                elif (abs(deltas_tether[1]) <= Ki_3_Threshold and abs(deltas_tether[1]) > Ki_4_Threshold):
                    Ki_t2 = Ki_3
                    print " Integral Control Tether 2 level 3"
                    errorsum[1] = errorsum[1] + deltas_tether[1]
                    target_achieved[1] = 0

                elif (abs(deltas_tether[1]) <= Ki_4_Threshold and abs(deltas_tether[1]) > Ki_5_Threshold):
                    Ki_t2 = Ki_4
                    print " Integral Control Tether 2 level 4 (Target Acquired)"
                    errorsum[1] = errorsum[1] + deltas_tether[1]
                    target_achieved[1] = 1

                elif (abs(deltas_tether[1]) <= Ki_5_Threshold and abs(deltas_tether[1]) >= 0):
                    Ki_t2 = 0
                    print "Target acquired tether 2"
                    errorsum[1] = 0
                    K[1] = 0
                    target_achieved[1] = 1

                else:
                    Ki_t2 = 0
                    print "No integral tether 2"
                    target_achieved[1] = 0

                #Integral control for tether 3
                if (abs(deltas_tether[2]) < Ki_1_Threshold and abs(deltas_tether[2]) > Ki_2_Threshold):
                    Ki_t3 = Ki_1
                    print " Integral Control Tether 3 level 1"
                    errorsum[2] = errorsum[2] + deltas_tether[2]
                    target_achieved[2] = 0

                elif (abs(deltas_tether[2]) <= Ki_2_Threshold and abs(deltas_tether[2]) > Ki_3_Threshold):
                    Ki_t3 = Ki_2
                    print " Integral Control Tether 3 level 2"
                    errorsum[2] = errorsum[2] + deltas_tether[2]
                    target_achieved[2] = 0

                elif (abs(deltas_tether[2]) <= Ki_3_Threshold and abs(deltas_tether[2]) > Ki_4_Threshold):
                    Ki_t3 = Ki_3
                    print " Integral Control Tether 3 level 3"
                    errorsum[2] = errorsum[2] + deltas_tether[2]
                    target_achieved[2] = 0

                elif (abs(deltas_tether[2]) <= Ki_4_Threshold and abs(deltas_tether[2]) > Ki_5_Threshold):
                    Ki_t3 = Ki_4
                    print " Integral Control Tether 3 level 4 (Target Acquired)"
                    errorsum[2] = errorsum[2] + deltas_tether[2]
                    target_achieved[2] = 1

                elif (abs(deltas_tether[2]) <= Ki_5_Threshold and abs(deltas_tether[2]) >= 0):
                    Ki_t3 = 0
                    print "Target acquired tether 3"
                    errorsum[2] = 0
                    K[2] = 0
                    target_achieved[2] = 1

                else:
                    Ki_t3 = 0
                    print "No integral tether 3"
                    target_achieved[2] = 0

                #Check if reached target position and calculate time
                if((target_achieved[0] is 1)and(target_achieved[1] is 1)and(target_achieved[2] is 1)):
                    if(target_reached is False):
                        goal_stop=now()
                    print "Goal reached in %f seconds."%(goal_stop-goal_start)
                    target_reached=True


                t1 = (deltas_tether[0]+errorsum[0]*Ki_t1+(deltas_tether[0]-deltas_tether_prev[0])*Kd/dt)*K[0] #control input for motor 1 with gain factored in

                t2 = (deltas_tether[1]+errorsum[1]*Ki_t2+(deltas_tether[1]-deltas_tether_prev[1])*Kd/dt)*K[1] #control input for motor 2 with gain factored in

                t3 = (deltas_tether[2]+errorsum[2]*Ki_t3+(deltas_tether[2]-deltas_tether_prev[2])*Kd/dt)*K[2] #control input for motor 3 with gain factored in
                #deltaT_prev=deltas_tether
                print "cmd torques"
                print "%f" %t1
                print "%f" %t2
                print "%f" %t3
                print "current position is:"		
                print sz1.get_xyz_pos()
                #command ckbots to move. Note positive value is CCW
                if math.isnan(t1):
                    print "NaN Error!!!"
                    t1=0
                if math.isnan(t2):
                    print "NaN Error!!!"
                    t2=0
                if math.isnan(t3):
                    print "NaN Error!!!"
                    t3=0

                torque_threshold=0.02

                if sz1.tether_subtract_CCW:
			
			
                    if t1<0 :
                        c.at.T1.set_torque_mx(-1.1*t1)
                    else:
                        if((abs(t2)<torque_threshold)and(abs(t3)<torque_threshold)or(abs(t1)<0.007)):#other torques are insignificant
                            t1=0;
                        c.at.T1.set_torque_mx(-0.2*t1)
                    if t2<0:
                        c.at.T2.set_torque(-1.4*t2)
                    else:
                        if ((abs(t1) < torque_threshold) and (abs(t3) < torque_threshold)):  # other torques are insignificant
                            t2 = 0;
                        c.at.T2.set_torque(-t2)
                    if t3<0:
                        c.at.T3.set_torque(-1.4*t3)
                    else:
                        if ((abs(t1) < torque_threshold) and (abs(t2) < torque_threshold)):  # other torques are insignificant
                            t3 = 0;
                        c.at.T3.set_torque(-t3)
		    
                else:
                    c.at.T1.set_torque(t1)
                    c.at.T2.set_torque(t2)
                    c.at.T3.set_torque(t3)
		    
		    
                print 'Tor1 %f' %c.at.T1.get_torque()
                print 'Tor2 %f' %c.at.T2.get_torque()
                print 'Tor3 %f' %c.at.T3.get_torque()

                #print sz1.get_state
            else:
                for m in c.itermodules():
                    m.set_torque(0) #may want to change this to torque is zero
                #send 
        except KeyboardInterrupt or ValueError:
            # Break out of the loop
            break
        except Exception,e:

            print "\n\nERROR DETECTED."
            print str(e)
            print "\n\n"
            sleep(1)
            # Break out of the loop
            pass
            #break


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
