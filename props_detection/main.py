import cv2
import os
import numpy as np
cap = cv2.VideoCapture(1)
path = "./props_detection/image/"
while True:
    _, frame = cap.read()
    y = []
    D = []
    for fname in os.listdir(path):
        if '.png' in fname:
            x = cv2.imread(path + fname)
            y.append(fname.split('_')[0])
            D.append(np.sum((x-frame)**2))
    if len(D) > 0:
        ans = y[D.index(min(D))]
        cv2.putText(frame, ans, (10, 20),
                    cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255))
    cv2.imshow('frame', frame)
    cv2.waitKey(1)

    print("the object is: ", ans)
