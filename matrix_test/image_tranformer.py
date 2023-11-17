import cv2
from matplotlib import pyplot as plt
import numpy as np
import sys
import logging
from PIL import Image

logging.basicConfig(filename="log.txt", filemode="w")
logger = logging.getLogger()
logger.setLevel(logging.INFO)



np.set_printoptions(threshold=sys.maxsize)

file = "test_pix3.jpg"

# Load the image
image = cv2.imread(file)

new_width = 45
new_height = 45

# Resize the image
resized_image = cv2.resize(image, (new_width, new_height))

# Convert the image to HSV
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Extract the Value (brightness) channel from image
value_channel = image[:, :, 2]

image_array = np.array(value_channel)
img = Image.fromarray(image_array)
img.show()

size = value_channel.shape

# Display the images
plt.subplot(1, 3, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('Color Image')

plt.subplot(1, 3, 2)
plt.imshow(hsv_image)
plt.title('HSV Image')

plt.subplot(1, 3, 3)
plt.imshow(value_channel, cmap='gray')
plt.title('Value Channel (Grayscale)')



print(image_array)
print(size)

logger.info(image_array)
logger.info(size)
plt.show()
