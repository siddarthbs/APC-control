import freenect
import cv2
import numpy as np
import pdb
import time

fx_rgb = 5.2921508098293293e+02
fy_rgb = 5.2556393630057437e+02
cx_rgb = 3.2894272028759258e+02
cy_rgb = 2.6748068171871557e+02
k1_rgb = 2.6451622333009589e-01
k2_rgb = -8.3990749424620825e-01
p1_rgb = -1.9922302173693159e-03
p2_rgb = 1.4371995932897616e-03
k3_rgb = 9.1192465078713847e-01

fx_d = 5.9421434211923247e+02
fy_d = 5.9104053696870778e+02
cx_d = 3.3930780975300314e+02
cy_d = 2.4273913761751615e+02
k1_d = -2.6386489753128833e-01
k2_d = 9.9966832163729757e-01
p1_d = -7.6275862143610667e-04
p2_d = 5.0350940090814270e-03
k3_d = -1.3053628089976321e+00

R = np.matrix([[ 9.9984628826577793e-01, 1.2635359098409581e-03, -1.7487233004436643e-02],\
	       [ -1.4779096108364480e-03,9.9992385683542895e-01, -1.2251380107679535e-02],\
	       [1.7470421412464927e-02, 1.2275341476520762e-02,9.9977202419716948e-01 ]])

T = np.matrix([[ 1.9985242312092553e-02],\
       	       [-7.4423738761617583e-04],\
               [-1.0916736334336222e-02 ]])


H = np.hstack([R,T])

x = np.linspace(0, 639, 640)
y = np.linspace(0, 479, 480)
xv, yv = np.meshgrid(x,y)

def Rectify(depth):
	depth = .1236 * np.tan((depth/2842.5) + 1.1863)
	#depth = 1/(-.00307*depth + 3.33)

	P3Dx = (xv - cx_d) * depth / fx_d
	P3Dy = (yv - cy_d) * depth / fy_d
	P3D = np.vstack([P3Dx.flatten(),P3Dy.flatten(),depth.flatten(),np.ones([1,np.size(depth)])])

	P3D_proj = H.dot(P3D)

	P2D_rgbx = P3D_proj[0,:].flatten()
	P2D_rgby = P3D_proj[1,:].flatten()
	P2D_rgbz = P3D_proj[2,:].flatten()
	
	P2D_rgbx = (P2D_rgbx*fx_rgb/P2D_rgbz) + cx_rgb
	P2D_rgby = (P2D_rgby*fx_rgb/P2D_rgbz) + cy_rgb
	
	depth_rect = np.zeros(np.shape(depth))
	#print np.shape(P2D_rgbx)
	# print np.shape(P3D_proj)
	depth_rect[P2D_rgby.astype(int),P2D_rgbx.astype(int)] = P2D_rgbz

	#print depth_rect.max()
	#print depth_rect.min()
	#print depth.max()
	#print depth.min()
	
	#depth_rect = depth_rect/depth_rect.max()*255
	return depth_rect
		
