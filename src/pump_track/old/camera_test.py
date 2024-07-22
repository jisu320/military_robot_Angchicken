import cv2

cap1 = cv2.VideoCapture(4)
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


cap2 = cv2.VideoCapture(6)
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while cv2.waitKey(33) < 0:
    ret1, frame1 = cap1.read()
    cv2.imshow("VideoFrame1", frame1)
    ret2, frame2 = cap2.read()
    cv2.imshow("VideoFrame2", frame2)

cap1.release()
cap2.release()
cv2.destroyAllWindows()
