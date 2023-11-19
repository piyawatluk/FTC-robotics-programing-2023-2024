import cv2
cap = cv2.VideoCapture(1)
path = './img/'
r, p = (0, 0, 0, 0)
while True:
    _, frame = cap.read()
    key = cv2.waitKey(1) & 0xFF
    if key == ord('r'):
        r += 1
        cv2.imwrite(path + 'prop_red_' + str(r) + '.png', frame)
    if key == ord('p'):
        p += 1
        cv2.imwrite(path + 'prop_blue' + str(p) + '.png', frame)

    cv2.imshow('frame', frame)
