import cv2
from matplotlib import pyplot as plt
import numpy as np


file = "/Users/User/Documents/GitHub/FTC-robotics-programing/opencv_HSV/test_pix2.png"
# Load the image
image = cv2.imread(file)

# Convert the image to the HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define the lower and upper color thresholds for red
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])

# Create a mask based on the color thresholds
mask = cv2.inRange(hsv_image, lower_red, upper_red)

# Apply morphological operations to remove noise
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# Find contours of the red areas
contours, _ = cv2.findContours(
    mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Draw bounding boxes around the red areas
for contour in contours:
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 3)


plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.show()
