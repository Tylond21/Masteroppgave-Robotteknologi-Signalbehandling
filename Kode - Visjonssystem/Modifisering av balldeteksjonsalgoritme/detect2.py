#!/usr/bin/env python3
import sys
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy
import rospy
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge

from std_msgs.msg import String, Float64MultiArray, UInt8MultiArray, Int8, Float32
from sensor_msgs.msg import Image
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
from utils import *
from scipy import spatial
import image_tools.ImageFunctions
from image_tools import ImageFunctions, camera_correction, Camera
from image_tools.ImageFunctions import *
import threading
import cv2
import numpy as np
import time
from random import randint



img_width = 1280

t0 = None
t1 = None
dt = None

first = True

cam_mtx = None

velocity = {}
last_velocity = {}
dists = [(0.0, 0.0)]

last_box = []

velsx = []
velsy = []
x_mean = []
y_mean = []
# x1, y1, x2, y2, vx, vy, depth

# Initialize uEye XS camera
cam = Camera.Camera()
cam.init()
cam.set_parameters()
cam.allocate_memory()
cam.capture_video()


rospy.init_node('vision', anonymous=True)
pub = rospy.Publisher('vision', Float64MultiArray, queue_size=1)
rospy.sleep(2)
msg = Float64MultiArray(data=[0, 0, 0, 0, 0])

#temp_img = 'white-template.jpg'
#temp_img = 'orange-template.jpg'
temp_img = 'blue-template.jpg';

#temp_img = "orange-template.jpg"
template = cv2.imread(temp_img)

temp_hist = hsv_hist(template)

diff_thresh = 130;

method = [cv2.HISTCMP_CORREL, cv2.HISTCMP_CHISQR, cv2.HISTCMP_INTERSECT, cv2.HISTCMP_BHATTACHARYYA]




def detect_circles(img, frame): 

  boxes = []
  #img = cv2.cvtColor(img, cv2.COLOR_HSV2GRAY)
  img = img.astype("uint8")
  circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 
                           minDist=100,
                           param1=70,
                           param2=11,
                           minRadius=40,
                           maxRadius=80)

  # convert circles into expected type
  #circles_img = np.copy(frame)

  if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
      center = (i[0],i[1])
      r = i[2]

      x = center[0] - r; y = center[1] - r;
      w = 2 * r; h = w;

      #cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
      #if x > 0.3 * img_width and x < 0.75 * img_width:
      if True:
      	#cv2.circle(circles_img, (i[0],i[1]),i[2],(255,0,0), 4)

      	box = np.array([x, y, x + w, y + h])
      	boxes.append(box)



  #cv2.imwrite('blue-circles.jpg', circles_img)

  return np.array(boxes)  

def detect(img):


	

    global t1, last_box, pub, first, t0, dt, cam_mtx, msg
    t0 = time.time()
    #print('first state: ', first);

    if t1 is not None:
        dt = t0 - t1;
        first = False;
        
    img = unsharp(img)
    #img = cv2.GaussianBlur(img,(5,5),0)
    #cv2.imwrite(sdir + 'sharp.jpg', img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imwrite(sdir + 'gray.jpg', gray)
    boxes = detect_circles(gray, img)
    t1 = time.time()
    boxes = nms(boxes, 0.2)

    for box in boxes:

    	x1, y1, x2, y2 = box
    	if abs(x1 - x2) < 5 or abs(y1 - y2) < 5:
    		continue
    		
    	try:
    		roi = img[y1: y2, x1: x2]
    		#print(roi.shape)
    
    		roi_hist = hsv_hist(roi)
    		diff = cv2.compareHist(temp_hist, roi_hist, method[2]);

    
    		#if diff > 120.0 and diff < 400.0:
    		#if cv2.compareHist(temp_hist, roi_hist, method[2]) > 165:
    		if True:
    		
    			center = get_box_center(x1, y1, x2, y2)
    			box_width = get_box_width(x1, x2)
    			pixels_mm_ratio = pixel_to_mm(box_width)
    			depth = get_depth(pixels_mm_ratio)  # in mm
    			
    			  		
    			if first:
    				old_center = center
    				dx = center[0] -  old_center[0]
    				dy = center[1] -  old_center[1]
    				dists.append((dx, dy))

    				#first = False;

		  			
    			
    			if not first and len(last_box) > 0:

    				old_center = last_box[-1]
    				dx = center[0] -  old_center[0];
    				dy = center[1] -  old_center[1]
    				old_dx, old_dy = dists[-1]
    				dists.append((dx, dy))
    				Vx = compute_velocity(dx, old_dx, dt) * pixels_mm_ratio # in mm/s
    				Vy = compute_velocity(dy, old_dy, dt) * pixels_mm_ratio # in mm/s
    				'''
    				velsx.append(Vx)
    				velsy.append(Vy)
    				if len(velsx) > 12:
    					Vx = np.mean(velsx[-10: ]) 
    					Vy = np.mean(velsy[-10: ])
    				else:
    					Vx = np.mean(velsx)
    					Vy = np.mean(velsy)
    				'''  				
						
						
    				cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
    				cv2.putText(img, "Vx: " + str(np.around(Vx, 2)) + " "  + "Vy: " + str(np.around(Vy, 2)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
    				cv2.putText(img, 'diff: ' + str(np.around(depth, 2) / 10) + ' cm' , (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)

    				uv_depth = np.array([center[0], center[1], depth])

    				x, y , z = project_image_to_cam(uv_depth, cam_mtx).squeeze()

    				if (y >= -300 and y <=300):
							  if(x >= -300 and x <=300):
							    msg = Float64MultiArray(data=[x, y, z, Vx, Vy])
							  else:
							    pass
    				else:
							  pass	

    			last_box = []
    			last_box.append(center)
    			pub.publish(msg)
    				#cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
    				#cv2.putText(img, str(diff), (x1,y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0))



    	except Exception as e:
    		#pub.publish(msg)
    		print(str(e))
		
  
    		


    # Show dets 
    #ix = randint(0, 100)
    cv2.imshow("img", img)
    cv2.waitKey(1)
    #cv2.imwrite('./img' + str(ix) + '.jpg', img);




if __name__=='__main__':


	#global temp


	
	cam_mtx = load_calibration(calibration_file = './cam_calibration.npy')


	while not rospy.is_shutdown():
		frame = cam.get_image()  
		detect(frame)


