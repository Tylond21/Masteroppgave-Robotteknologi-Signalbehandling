
import cv2
import numpy as np
import time




def load_calibration(calibration_file):

	calibs = {}

	print('Loading Camera Parameters ...')
	_, calibs["mtx"], calibs["dist"] = np.load(calibration_file, allow_pickle=True)

	mtx = np.hstack((calibs['mtx'], np.zeros((3,1))))
	
	return mtx

def unsharp(image):

	blurred= cv2.GaussianBlur(image, (3, 3), 0)
	sharp = cv2.addWeighted(image, 1.5, blurred, -0.5, 0)
	return sharp
	
	
def hsv_hist(img):

	img = cv2.resize(img, (50, 50))
	hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	hist = cv2.calcHist([hsv], [0, 1, 2], None, [180, 256, 256], [0, 180, 0, 256, 0, 256])
	#hist = cv2.normalize(hist, hist).flatten()
	hist = hist.flatten()
	return hist



def project_image_to_cam(uv_depth, cam_mtx):
	''' Input: nx3 first two channels are uv, 3rd channel
      is depth in rect camera coord.
        Output: nx3 points in rect camera coord.
	''' 
	
	
	c_u = cam_mtx[0,2]
	c_v = cam_mtx[1,2]
	f_u = cam_mtx[0,0]
	f_v = cam_mtx[1,1]

	n = uv_depth.shape[0]
	x = ((uv_depth[0]-c_u)*uv_depth[2]) / f_u
	y = ((uv_depth[1]-c_v)*uv_depth[2]) / f_v
	pts_3d_rect = np.zeros((1,3))
	pts_3d_rect[0, 0] = x
	pts_3d_rect[0, 1] = y
	pts_3d_rect[0, 2] = uv_depth[2]
	
	return pts_3d_rect
		  
		  
def distance(x1, y1, x2, y2):

	dist = np.sqrt( ((x1 - x2) ** 2) + ((y1 - y2) ** 2) )
	return dist

def get_box_center(x1, y1, x2, y2):
	center = ((x1 + x2) // 2, (y1 + y2) // 2)
	return center

def compute_velocity(dist1, dist2, dt):
	return (dist1 - dist2) / dt
        
def calc_color(img, channel, rang):
    hist = cv2.calcHist([img], [channel], None, [rang], [0, rang])
    color = np.argmax(hist)

    return color
        

def get_dominant_color(roi):
    #black = (np.array([0,0,0],np.uint8), np.array([180,255,100],np.int8))
    #image_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    #print('hsv shape: ', image_hsv.shape)

    r = calc_color(roi, 0, 256)
    g = calc_color(roi, 1, 256)
    b = calc_color(roi, 2, 256)

    
    
    return (r, g, b)


def verify(roi, value):

    roi_color = np.array(get_dominant_color(roi))
    #temp_color = get_dominant_color()
    r, g, b = roi_color.squeeze()
    h, s, v = rgb2hsv(r,g, b)
    #print(h, s, v)
    
    #return (h, s, v)
    
    
    if v < value:
    	return True
    else:
    	return False

    #diff = np.mean(abs(roi_color - temp_color))
    #diffs.append(diff)

    #return diff


def get_hsv(roi):

    roi_color = np.array(get_dominant_color(roi))
    #x = roi.shape[0] // 2; y = roi.shape[1] // 2
    #roi_color = roi[y, x, :]
    #temp_color = get_dominant_color()
    r, g, b = roi_color.squeeze()
    h, s, v = rgb2hsv(r,g, b)
    #print(h, s, v)
    
    return (h, s, v)
  
  
def similarity(im1, im2):
	result = spatial.distance.cosine(im2, im1)
	return result




def nms(boxes, overlapThresh):
    # if there are no boxes, return an empty list
    if len(boxes) == 0:
        return []
    # if the bounding boxes integers, convert them to floats --
    # this is important since we'll be doing a bunch of divisions
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")
    # initialize the list of picked indexes 
    pick = []
    # grab the coordinates of the bounding boxes
    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]
    # compute the area of the bounding boxes and sort the bounding
    # boxes by the bottom-right y-coordinate of the bounding box
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)
    # keep looping while some indexes still remain in the indexes
    # list
    while len(idxs) > 0:
        # grab the last index in the indexes list and add the
        # index value to the list of picked indexes
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)
        # find the largest (x, y) coordinates for the start of
        # the bounding box and the smallest (x, y) coordinates
        # for the end of the bounding box
        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])
        # compute the width and height of the bounding box
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)
        # compute the ratio of overlap
        overlap = (w * h) / area[idxs[:last]]
        # delete all indexes from the index list that have
        idxs = np.delete(idxs, np.concatenate(([last],
            np.where(overlap > overlapThresh)[0])))
    # return only the bounding boxes that were picked using the
    # integer data type
    return boxes[pick].astype("int")


def apply_mask(matrix, mask, fill_value):
    masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
    return masked.filled()

def apply_threshold(matrix, low_value, high_value):
    low_mask = matrix < low_value
    matrix = apply_mask(matrix, low_mask, low_value)

    high_mask = matrix > high_value
    matrix = apply_mask(matrix, high_mask, high_value)

    return matrix

def white_balance(img, percent):
    assert img.shape[2] == 3
    assert percent > 0 and percent < 100

    half_percent = percent / 200.0

    channels = cv2.split(img)

    out_channels = []
    for channel in channels:

        assert len(channel.shape) == 2
        # find the low and high precentile values (based on the input percentile)
        height, width = channel.shape
        vec_size = width * height
        flat = channel.reshape(vec_size)

        assert len(flat.shape) == 1

        flat = np.sort(flat)

        n_cols = flat.shape[0]

        low_val  = flat[math.floor(n_cols * half_percent)]
        high_val = flat[math.ceil( n_cols * (1.0 - half_percent))]

        #print ("Lowval: ", low_val)
        #print ("Highval: ", high_val)


        # saturate below the low percentile and above the high percentile
        thresholded = apply_threshold(channel, low_val, high_val)
        # scale the channel
        normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
        out_channels.append(normalized)

    return cv2.merge(out_channels)
    
  
def rgb2hsv(r, g, b):
	r, g, b = r/255.0, g/255.0, b/255.0
	mx = max(r, g, b)
	mn = min(r, g, b)
	df = mx-mn
	if mx == mn:
		  h = 0
	elif mx == r:
		  h = (60 * ((g-b)/df) + 360) % 360
	elif mx == g:
		  h = (60 * ((b-r)/df) + 120) % 360
	elif mx == b:
		  h = (60 * ((r-g)/df) + 240) % 360
	if mx == 0:
		  s = 0
	else:
		  s = df/mx
	v = mx
	return h / 2, s * 255, v * 255
	
	
	


def pixel_to_mm(box_width):

   
	ball_diameter = 41.7 ## hardcoded, for the black ball

   
	pixel_to_mm = ball_diameter / box_width  # mm_width / px_width

	return pixel_to_mm


def get_depth(pixel_to_mm):

	focal_length = 3.7  # mm (+/- 5 percent)
	sensor_width = 3.6288
	# sensor_height = 2.7216 (not used here)
	resolution_width = 1280

	fov_width = pixel_to_mm * resolution_width

	working_distance = (fov_width / sensor_width) * focal_length


	return working_distance
    
    
def get_box_width(x1, x2):

	width = x2 - x1
    
	return width
	


