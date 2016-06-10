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
import numpy as np
import spiral_zipper as sz

# Specify module ID and the name we want to give each of them:
modules = { 0xDF: 'T1', 0xF7: 'T2',0xE1:'T3' } 

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

    # Limit module speed and torque for safety
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

    K1 = 2.0 # P- controller gain
    K2 = 2.0 #^
    K3 = 2.1 # this modlue seems to be lagging behind         

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
                    if evt.button in range(6):
                        drivable = not drivable
                        print 'Drivability status: %s' % str(drivable)
                    # Pressing the trigger buttons ends demo.
                    else:
                        raise KeyboardInterrupt
            if drivable:
                # Set the torque values on the CR modules based on the 
                # inputs that we got from the joystick 
                #c.at.T1.set_torque( -(drive*driveGain - turn*turnGain) )
                #c.at.T2.set_torque(  drive*driveGain + turn*turnGain )
                #print "\n"*5
                sz1.update_goal([x_move*Kx,y_move*Ky,0.0])
                #print "goal is"
                #print sz1.get_goal()
                sz1.update_state (c)
                print "xyz error:"
                print sz1.get_error_xyz()
                # controller for moving CkBots.
                deltas_tether = sz1.get_error_state()
                #print "delta tether lengths:"
                #print deltas_tether #to debug
                
                t1 = deltas_tether[0]*K1 #control input with gain factored in
                t2 = deltas_tether[1]*K2
                t3 = deltas_tether[2]*K3
                #print "cmd torques"
                #print "%f" %t1
                #print "%f" %t2
                #print "%f" %t3
                #print "current position is:"
                #print sz1.get_xyz_pos()
                #command ckbots to move. Note positive value is CCW
                if sz1.tether_subtract_CCW:
                    c.at.T1.set_torque(-t1)
                    c.at.T2.set_torque(-t2)
                    c.at.T3.set_torque(-t3)
                else:
                    c.at.T1.set_torque(t1)
                    c.at.T2.set_torque(t2)
                    c.at.T3.set_torque(t3)

                #print sz1.get_state
            else:
                for m in c.itermodules():
                    m.go_slack() #may want to change this to torque is zero
                #send 
        except KeyboardInterrupt:
            # Break out of the loop
            break

    print "Demo exiting, turning off all modules"
    # Turn the modules off before we exit for safety
    for m in c.itermodules():
        m.go_slack()