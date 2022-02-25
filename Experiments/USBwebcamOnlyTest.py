import cv2

vs = cv2.VideoCapture(0)  # First camera found


loop = True
while loop and vs.isOpened():
    retval, screenshot = vs.read()
    if not retval:
        break

    cv2.imshow("Live Feed fbi", screenshot)
    # Allows frame to be displayed, and waits 5ms per frame. Close window with q
    if cv2.waitKey(2) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows
