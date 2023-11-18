import cv2
from matplotlib import pyplot as plt
import numpy as np
from PIL import Image

np.set_printoptions(threshold=np.inf)

file = r"C:\Users\piyaw\PycharmProjects\suffering\test_pix3.jpg"

size_x = 960
size_y = 960

# Load the image
image = cv2.imread(file)

# Convert the image to HSV
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Extract the Value (brightness) channel from the image
value_channel = image[:, :, 2]

image_array = np.array(value_channel)
hsv_array = np.array(hsv_image)

# Display the images
plt.subplot(1, 3, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('Original Image')

plt.subplot(1, 3, 2)
plt.imshow(hsv_image)
plt.title('HSV Image')

plt.subplot(1, 3, 3)
plt.imshow(value_channel, cmap='gray')
plt.title('Value Channel (Grayscale)')

# Thresholding on the Value channel
value_threshold_min, value_threshold_max = 230, 240
value_thresholded = np.where((value_channel >= value_threshold_min) & (value_channel <= value_threshold_max), 255, 0)

# HSV range-based thresholding
hsv_range_min = np.array([0, 0, 100])
hsv_range_max = np.array([30, 166, 260])
hsv_thresholded = np.where(((hsv_array >= hsv_range_min) & (hsv_array <= hsv_range_max)).all(axis=2), 0, 255)

# Create an alpha channel
alpha_channel = np.ones_like(value_thresholded) * 255

# Combine thresholded images with the same color for HSV and background
combined_image = cv2.merge([value_thresholded, hsv_thresholded, alpha_channel])

# Convert to 8-bit unsigned integer
combined_image = combined_image.astype(np.uint8)

# Convert to grayscale
combined_image_gray = cv2.cvtColor(combined_image, cv2.COLOR_RGBA2GRAY)

# Display the thresholded images
plt.imshow(combined_image)
plt.title('Combined Image')
img_2 = Image.fromarray(combined_image_gray)
img_2.show()
plt.imshow(combined_image_gray, cmap='gray')
plt.title('Grayscale Image')
plt.show()
