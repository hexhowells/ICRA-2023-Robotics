import cv2
import numpy as np


def filter_lines(img):
	# Convert the image to HSV color space
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# Create a mask to filter out the red color
	lower_red = np.array([0,150,150])
	upper_red = np.array([10,255,255])
	mask1 = cv2.inRange(hsv, lower_red, upper_red)

	lower_red = np.array([170,50,50])
	upper_red = np.array([180,255,255])
	mask2 = cv2.inRange(hsv, lower_red, upper_red)

	mask = mask1 + mask2

	# Apply a Gaussian blur to the mask to reduce noise
	blur = cv2.GaussianBlur(mask, (5,5), 0)

	# Use Canny edge detection to detect edges in the image
	edges = cv2.Canny(blur, threshold1=50, threshold2=150, apertureSize=3)

	# Use the HoughLinesP function to detect lines in the image
	lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=40, minLineLength=50, maxLineGap=100)

	height, width = img.shape[:2]
	final_image = np.zeros((height, width, 3), dtype=np.uint8)

	if lines is not None:
		
		# Draw each line in white on the black image
		for line in lines:
			x1, y1, x2, y2 = line[0]
			cv2.line(final_image, (x1, y1), (x2, y2), (255, 255, 255), thickness=2)

	return final_image
