import cv2
cap = cv2.VideoCapture(1)
path = './img/'
r, b = (0, 0)
while True:
    _, frame = cap.read()
    key = cv2.waitKey(1) & 0xFF

    # press "r" to save prop_red
    if key == ord('r'):
        r += 1
        cv2.imwrite(path + 'prop_red_' + str(r) + '.png', frame)

    # press "b" to save prop_blue
    if key == ord('b'):
        b += 1
        cv2.imwrite(path + 'prop_blue_' + str(b) + '.png', frame)
    cv2.imshow('frame', frame)
