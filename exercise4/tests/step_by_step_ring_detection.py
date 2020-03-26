#!/usr/bin/env python
from __future__ import print_function

import sys
import cv2
import numpy as np


# Load the image from a file
cv_image = cv2.imread('/home/vicos/ROS/src/exercise5/imgs/ring1.png')

cv_original = cv_image.copy()

cv2.imshow("Image window", cv_image)
cv2.waitKey(0)

# Set the dimensions of the image
dims = cv_image.shape

# Tranform image to gayscale
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

cv2.imshow("Image window", gray)
cv2.waitKey(0)

# Do histogram equlization
img = cv2.equalizeHist(gray)

cv2.imshow("Image window", img)
cv2.waitKey(0)

# Binarize the image
ret, thresh = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)

cv2.imshow("Image window", thresh)
cv2.waitKey(0)

# Extract contours
img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
#print(contours)
#print(hierarchy)
#print(_)

# Example how to draw the contours
cv2.drawContours(cv_image, contours, -1, (255, 0, 0), 3)

cv2.imshow("Image window", cv_image)
cv2.waitKey(0)

# Fit elipses to all extracted contours
elps = []
for cnt in contours:
    if cnt.shape[0] >= 20:
        ellipse = cv2.fitEllipse(cnt)
        elps.append(ellipse)
	#
	cv2.ellipse(cv_image, ellipse, (0, 255, 0))

cv2.imshow("Image window", cv_image)
cv2.waitKey()

# Find two elipses with same centers
candidates = []
for n in range(len(elps)):
    for m in range(n + 1, len(elps)):
        e1 = elps[n]
        e2 = elps[m]
        dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
        # print dist
        if dist < 5:
            candidates.append((e1,e2))


# Preparing to extract the depth from the image
for c in candidates:

    e1 = c[0]
    e2 = c[1]

    # Draw the detections
    cv2.ellipse(cv_original, e1, (0, 255, 0), 2)
    cv2.ellipse(cv_original, e2, (0, 255, 0), 2)

    size = (e1[1][0]+e1[1][1])/2
    center = (e1[0][1], e1[0][0])

    x1 = int(center[0] - size / 2)
    x2 = int(center[0] + size / 2)
    x_min = x1 if x1>0 else 0
    x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

    y1 = int(center[1] - size / 2)
    y2 = int(center[1] + size / 2)
    y_min = y1 if y1 > 0 else 0
    y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]

    ### Here is the code for looking in the depth image


cv2.imshow("Image window", cv_original)
cv2.waitKey(0)

cv2.destroyAllWindows()
