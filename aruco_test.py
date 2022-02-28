
import cv2
import cv2.aruco as aruco
import numpy as np
import pyrealsense2 as rs

# Defines the path of the calibration file and the dictonary used
calibration_path = "realsense_d435.npz"
dictionary = aruco.DICT_6X6_250
#cv2.aruco.DICT_APRILTAG_36h11 
#aruco.DICT_APRILTAG_36H11 #

# Load calibration from file
mtx = None
dist = None
with np.load(calibration_path) as X:
	mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

# Initialize communication with intel realsense
pipeline = rs.pipeline()
realsense_cfg = rs.config()
realsense_cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)
pipeline.start(realsense_cfg)

# Check communication
print("Test data source...")
try:
	np.asanyarray(pipeline.wait_for_frames().get_color_frame().get_data())
except:
	raise Exception("Can't get rgb frame from data source")

print("Press [ESC] to close the application")
while True:
	# Get frame from realsense and convert to grayscale image
	frames = pipeline.wait_for_frames()
	img_rgb = np.asanyarray(frames.get_color_frame().get_data())
	img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)
	
	# Detect markers on the gray image
	res = aruco.detectMarkers(img_gray, aruco.getPredefinedDictionary(dictionary))


	aruco.drawDetectedMarkers(img_rgb, res[0], res[1]) 
	cv2.imshow("AR-Example", cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR))
	key = cv2.waitKey(10)
	# Draw each marker 
	for i in range(len(res[0])):
		# Estimate pose of the respective marker, with matrix size 1x1
		rvec_list_all, tvec_list_all, _ = aruco.estimatePoseSingleMarkers(res[0][i], 1, mtx, dist)
#### Yash: Feb 28

		rvecs = rvec_list_all[0][0]
		tvecs = tvec_list_all[0][0]	
				
		#aruco.drawAxis(img_rgb, mtx, dist, rvecs, tvecs, 100)
		tvec_str = "x=%4.2f y=%4.2f z=%4.2f"%(tvecs[0],tvecs[1],tvecs[2])	
		cv2.putText(img_rgb, tvec_str, (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2, cv2.LINE_AA)		
	# Display the result
	cv2.imshow("AR-Example", cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR))	
	
	# If [ESC] pressed, close the application
	if cv2.waitKey(100) == 27:
		print("Application closed")
		break
# Close all cv2 windows
cv2.destroyAllWindows()
