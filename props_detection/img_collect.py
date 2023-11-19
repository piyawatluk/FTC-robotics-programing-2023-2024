import cv2
cap = cv2.VideoCapture(1)
path = "./props_detection/image/"
r, b, s = (0, 0, 0)
while True:
    _, frame = cap.read()
    key = cv2.waitKey(1) & 0xFF

    # press "r" to save prop_red
    if key == ord('r'):
        r += 1
        cv2.imwrite(path + 'propred_' + str(r) + '.png', frame)

    # press "b" to save prop_blue
    if key == ord('b'):
        b += 1
        cv2.imwrite(path + 'propblue_' + str(b) + '.png', frame)

    if key == ord('s'):
        s += 1
        cv2.imwrite(path + 'skittles_' + str(s) + '.png', frame)
    cv2.imshow('frame', frame)
