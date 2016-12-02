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

def slack_remove(sz, c, armflag):
		print "Removing slack..."
		T7_slack=True
		T2_slack=True
		T3_slack=True
		unwind_time=0.3
		deflection_small=0.05
		deflection_large=.10

		grav = sz.get_sensor_readings(armflag) # read GRAVITY components
		grav_comps = [grav[1],grav[2],grav[0]]
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
			grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
			#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 

			pos_old = c.at.T7.get_pos()
			while(T7_slack):
				grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components 
				#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]

				if(grav_sensed_curr[1]-grav_sensed_init[1] > deflection_small):
					T4_slack=False
					print "T4 Slack removed."
					break				
				c.at.T7.set_torque(0.1) 
				pos_new = c.at.T7.get_pos()
				change = measure_dtheta(sz,pos_new, pos_old)
				print "1Delta Theta is : %f"%change
				pos_old=pos_new
			c.at.T7.set_torque(0) #stop
			
			if(grav_comps[0]<0): #leaning away from T2
				#unwind T3 a bit
				c.at.T3.set_torque(-0.1) #T3
				sleep(unwind_time)
				c.at.T3.set_torque(0)
				#wind T2 till deflection sensed =>				T2 slack removed
				pos_old = c.at.T2.get_pos()
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components 
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 

				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components 
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]

					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "2Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
				#wind T3 till *small*deflection sensed =>		T3 slack removed
				pos_old = c.at.T3.get_pos()
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components 
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 

				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components 
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]
			
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):
						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "3Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				
			elif(grav_comps[0]>0): #leaning away from T3
				#unwind T2 a bit
				c.at.T2.set_torque(-0.1) #T2
				sleep(unwind_time)
				c.at.T2.set_torque(0)
				
				# wind T3 till deflection sensed =>				T3 slack removed
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 
			
				pos_old = c.at.T3.get_pos()
				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]
				 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "4Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				# wind T2 till *small* deflection sensed =>		T2 slack removed
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 
		
				pos_old = c.at.T2.get_pos()
				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]
				 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "5Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
				
		elif (grav_comps[1]>0): # leaning towards T7

			if(grav_comps[0]<0): #leaning away from T2
				#unwind T7 and T3 a bit
				c.at.T7.set_torque(-0.1) #T7
				sleep(unwind_time)
				c.at.T7.set_torque(0)
				c.at.T3.set_torque(-0.1) #T3
				sleep(unwind_time)
				c.at.T3.set_torque(0)
				
				#wind T2 till deflection sensed =>				T2 slack removed
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 
			
				pos_old = c.at.T2.get_pos()
				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]
				 
					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "6Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
				
				
				#wind T3 till *small* deflection sensed => 		T3 slack removed
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 
			
				pos_old = c.at.T3.get_pos()
				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components 
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]

					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):

						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "7Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				
			elif(grav_comps[0]>0): #leaning away from T3
				#unwind T7 and T2 a bit
				c.at.T7.set_torque(-0.1) #T7
				sleep(unwind_time)
				c.at.T7.set_torque(0)
				c.at.T2.set_torque(-0.1) #T2
				sleep(unwind_time)
				c.at.T2.set_torque(0)
				
				# wind T3 till deflection sensed =>				T3 slack removed
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 
			
				pos_old = c.at.T3.get_pos()
				while(T3_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components 
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]

					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_large):
						T3_slack=False
						print "T3 Slack removed."
						break				
					c.at.T3.set_torque(0.1)
					pos_new = c.at.T3.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "8Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T3.set_torque(0) #stop
				# wind T2 till *small* deflection sensed =>		T2 slack removed
				grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
				#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 
			
				pos_old = c.at.T2.get_pos()
				while(T2_slack):
					grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components 
					#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]

					if(abs(grav_sensed_curr[0]-grav_sensed_init[0]) > deflection_small):
						T2_slack=False
						print "T2 Slack removed."
						break				
					c.at.T2.set_torque(0.1)
					pos_new = c.at.T2.get_pos()
					change = measure_dtheta(sz,pos_new, pos_old)
					print "9Delta Theta is : %f" % change
					pos_old = pos_new
				c.at.T2.set_torque(0) #stop
			
			#wind T7 till *small*deflection sensed =>		T7 slack removed
			grav_sensed_init = sz.get_sensor_readings(armflag) #read GRAVITY components
			#grav_sensed_init = [grav_sensed_init[1],-1 * grav_sensed_init[2],grav_sensed_init[0]] 

			pos_old = c.at.T7.get_pos()
			while(T7_slack):
				grav_sensed_curr = sz.get_sensor_readings(armflag) #read GRAVITY components 
				#grav_sensed_curr = [grav_sensed_curr[1],-1 * grav_sensed_curr[2],grav_sensed_curr[0]]

				if(abs(grav_sensed_curr[1]-grav_sensed_init[1]) > deflection_small):
					T7_slack=False
					print "T7 Slack removed."
					break				
				c.at.T7.set_torque(0.1)
				pos_new = c.at.T7.get_pos()
				change = measure_dtheta(sz,pos_new, pos_old)
				print "10Delta Theta is : %f" % change
				pos_old = pos_new
			c.at.T7.set_torque(0) #stop
		print "All slack removed."

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
