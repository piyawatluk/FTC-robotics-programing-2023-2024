import cv2
from matplotlib import pyplot as plt
import numpy as np
import sys
from PIL import Image


np.set_printoptions(threshold=sys.maxsize)

file = r"C:\Users\piyaw\PycharmProjects\suffering\test_pix3.jpg"

size_x = 960
size_y = 960


# Load the image
image = cv2.imread(file)

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
plt.title('Original Image')

plt.subplot(1, 3, 2)
plt.imshow(hsv_image)
plt.title('HSV Image')

plt.subplot(1, 3, 3)
plt.imshow(value_channel, cmap='gray')
plt.title('Value Channel (Grayscale)')

for i in range(size_x):
    for j in range(size_y):
        num = image_array[i, j]
        if num in range(230, 240):
            image_array[i, j] = 255
        else:
            image_array[i, j] = 0
img_2 = Image.fromarray(image_array)
img_2.show()
plt.show()