import cv2
from matplotlib import pyplot as plt
import numpy as np

file = "/Users/User/Documents/GitHub/FTC-robotics-programing/opencv_HSV/test_pix2.png"

# Load the image
image = cv2.imread(file)

# Display the the images
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('RGB')
plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_RGB2HSV))
plt.title('HSV')
plt.tight_layout()
plt.show()
