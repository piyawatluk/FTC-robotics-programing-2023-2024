import cv2
import numpy as np
from matplotlib import pyplot as plt

file = "/Users/User/Documents/GitHub/FTC-robotics-programing/opencv_HSV/test_pix2.png"

# Load the image
image = cv2.imread(file)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# red
red_lower = np.array([170, 100, 100])
red_upper = np.array([180, 255, 255])

mask = cv2.inRange(hsv, red_lower, red_upper)

res = cv2.bitwise_and(image, image, mask=mask)
c, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
for c in c:
    area = cv2.contourArea(c)
    if area > 1000:
        cv2.drawContours(image, [c], -1, (255, 0, 0), 2)
        print(area)

cv2.imshow('res', res)
cv2.imshow('image', image)
cv2.imshow('mask', mask)


cv2.waitKey(0)
cv2.destroyAllWindows()
