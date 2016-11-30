# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import freenect
import Rectification as rec

#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
 
#function to get depth image from kinect
def get_depth():
	array,_ = freenect.sync_get_depth()
	return array

# construct the argument parse and parse the arguments
def camera():
	ap = argparse.ArgumentParser()
	ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
	ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
	args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points

	#greenLower = (15, 0, 86) #yellow ball
	#greenUpper = (64, 255, 255)
	#greenLower = (0, 0, 86) #white ball
	#greenUpper = (25, 255, 255)
	greenLower = (50, 125, 150) #blue ball
	greenUpper = (150,255,255)
 
	# resize the frame, blur it, and convert it to the HSV
	# color space
        frame = get_video()
        depth = get_depth()
	# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
 	oldavg = 0 
	p_0 = np.matrix([[.25],[0],[.40]])			
	if len(cnts) > 0: 	# only proceed if at least one contour was found
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 		#print center
		#print x,y
		if radius > 5: # only proceed if the radius meets a minimum size

			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2) 	# draw the circle and centroid on the frame,
			depth = rec.Rectify(depth)
			xc = 0
			yc = 0
			if x > 634:  #if contour is too far to one corner of the screen
				#D = depth[479,center[1]]
				#D = .1236 * np.tan((depth[y,479]/2842.5) + 1.1863)
				#D = 100/(-.00307*depth[479,center[1]] + 3.33)
				print "raw,real depth is :"
				print depth[y,479], D
				xc = 479
				yc = y
				xw = 479
				yw = y

			else:				
				#D = 355.6/(1090.5 - depth[center[0],center[1]])
				avg = 3000
				k = 2
				for i in range(2*k):
					for j in range(2*k):
						if (depth[y+k-i,x+k-j] < avg) and (depth[y+k-i,x+k-j] > 0):
							avg = depth[y+k-i,x+k-j]
							xc = x+k-i
							yc = y+k-j
				#for i in range(480):
				#	for j in range(640):
				#		if depth[i,j] < avg:
				#			avg = depth[i,j]
				#			xc = x+k-i
				#			yc = y+k-j
				D = avg
				#D = .1236 * np.tan((avg/2842.5) + 1.1863)
				#D = 1/(-.00307*depth[xc,yc] + 3.33)
				print "raw,real depth is :"
				print avg, D
				a = .00173667
				xw = (xc-320)*a*D
				yw = (yc-240)*a*D
				print "camera coordinates are: "
				print xw,yw,D
			if (avg > 1.2) or (avg < 0):
				D = .75
				xw = 0
				yw = 0
			p_1 = np.matrix([[xw],[yw],[D],[1]])
			psi = np.pi/3
			phi = np.pi/2 + 27.5*np.pi/180
			H_10 = np.matrix([[ np.cos(psi), np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(psi), -0.32],\
	        		  [-np.sin(psi), np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(psi),-0.265],\
	          		  [ 0	       ,-np.sin(phi)	        , np.cos(phi)		 , 0.988],\
		  		  [ 0	       , 0                      , 0                      , 1   ]])

			p_0 = H_10 * p_1
			p_0 = np.matrix([[p_0.item(0)],[p_0.item(1)],[p_0.item(2)]])			
			print "world coordinates are: "
			print p_0		

			thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
			cv2.line(frame,(int(x),int(y)),(int(x),int(y)),(255,0,0),thickness)
 			cv2.line(frame,(int(xc),int(yc)),(int(xc),int(yc)),(0,255,0),thickness)
 			cv2.line(frame,(320,240),(320,240),(0,255,0),thickness)
			cv2.line(depth,(320,240),(320,240),(255,0,0),thickness)

			# show the frame to our screen
			cv2.imshow("Frame", frame)

	return p_0				
