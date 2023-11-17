import numpy
from numpy import random
import logging
import sys
from datetime import datetime

start_time = datetime.now()

numpy.set_printoptions(threshold=sys.maxsize)

logging.basicConfig(filename="lgt.txt", filemode="w")
logger = logging.getLogger()
logger.setLevel(logging.INFO)

size_x = 270
size_y = 270

size_sum = size_y * size_x

pic = random.randint(10, size=(size_x, size_y))
lg_c = len(pic)
lg_r = len(pic[0])
logger.info(pic)

print(pic)

for i in range(size_x):
    for j in range(size_y):
        num = pic[i, j]
        if num in range(0, 4):
            pic[i, j] = 0
        if num in range(4, 7):
            pic[i, j] = 1
        if num in range(7, 10):
            pic[i, j] = 2

end_time = datetime.now()
time_r = format(end_time - start_time)
logger.info(time_r)

print(pic)
