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



frames = []
img_width = 1280;
size = (960, 1280) 


t0 = None
t1 = None
dt = None

first = True

cam_mtx = None
#mot_tracker = Sort(max_age = max_age, min_hits= min_hits, iou_threshold = iou_threshold) 

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



def detect_circles(img): 

  boxes = []
  #img = cv2.cvtColor(img, cv2.COLOR_HSV2GRAY)
  img = img.astype("uint8")
  circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 
                           minDist=300,
                           param1=70,
                           param2=11,
                           minRadius=50,
                           maxRadius=150)

  # convert circles into expected type
  if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
      center = (i[0],i[1])
      r = i[2]

      x = center[0] - r; y = center[1] - r;
      w = 2 * r; h = w;

      #cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
      if x > 0.25 * img_width and x < 0.75 * img_width:
      	box = np.array([x, y, x + w, y + h])
      	boxes.append(box)

      #cv2.circle(image, (i[0],i[1]),i[2],(0,255,0), 4)

  return np.array(boxes)  
  

def detect(frame):

    global t1, last_box, pub, first, t0, dt, cam_mtx
    t0 = time.time()

    if t1 is not None:
    	dt = t0 - t1
    	first = False

    img = cv2.GaussianBlur(frame, (7, 7), 0)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


    boxes = detect_circles(gray)
    t1 = time.time()
    boxes = nms(boxes, 0.3)


    for box in boxes:

    	x1, y1, x2, y2 = box
    	roi = img[y1: y2, x1: x2]

   
    	if verify(roi, 30):

    		center = get_box_center(x1, y1, x2, y2)
    		box_width = get_box_width(x1, x2)
    		pixels_mm_ratio = pixel_to_mm(box_width)
    		depth = get_depth(pixels_mm_ratio)  # in mm
    		#print("depth: ", depth, ' m' )
    		
    		if first:
    			old_center = center
    			dx = center[0] -  old_center[0]
    			dy = center[1] -  old_center[1]

    			dists.append((dx, dy))
    			#old_dist = dist
    			
    			
    		if not first and len(last_box) > 0:
    			old_center = last_box[-1]
    			#dist_x = distance(center[0], center[1], old_center[0], old_center[1])
    			dx = center[0] -  old_center[0]
    			dy = center[1] -  old_center[1]

    			old_dx, old_dy = dists[-1]
    			dists.append((dx, dy))
    			Vx = compute_velocity(dx, old_dx, dt) * pixels_mm_ratio # in mm/s
    			Vy = compute_velocity(dy, old_dy, dt) * pixels_mm_ratio # in mm/s

    				
    			cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
    			cv2.putText(img, "Vx: " + str(np.around(Vx, 2)) + " "  + "Vy: " + str(np.around(Vy, 2)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
    			cv2.putText(img, "Depth: " + str(np.around(depth, 3) / 10) + " cm", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
    			#print("Vx: ", Vx, " cm/s", " Vy: ", Vy, " cm/s")
    			
    			uv_depth = np.array([center[0], center[1], depth])

    			x, y , z = project_image_to_cam(uv_depth, cam_mtx).squeeze()
					
    			if (y >= -300 and y <=300):
    			  if(x >= -300 and x <=300):
    			    msg = Float64MultiArray(data=[x, y, z, Vx, Vy])
    			    pub.publish(msg)
    			  else:
    			    pass
    			else:
    			  pass	

    		last_box = []
    		last_box.append(center)
    		


    # Show dets    
    cv2.imshow("img", img)
    cv2.waitKey(1)

    #frames.append(img)
    #cv2.imwrite('./img.jpg', img);




if __name__=='__main__':


	#global temp


	
	cam_mtx = load_calibration(calibration_file = './cam_calibration.npy')


	while not rospy.is_shutdown():

		  frame = cam.get_image()  

		  detect(frame)


	'''
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	
	imgg = frames[0]

	vw = imgg.shape[1]
	vh = imgg.shape[0]
	print ("Video size", vw,vh)
	outvideo = cv2.VideoWriter("rec.mp4",fourcc,20.0,(vw,vh))


	
	#for frame in frames:
	#	outvideo.write(frame)

	#outvideo.release()
	'''
	print('released')

