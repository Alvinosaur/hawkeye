import cv2
import numpy as np




def process(cap):
    while (1):
        error, frame = cap.read()
        if not error:
            print("canmot read frame")
            break

        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        # rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


        #normal rgb color range
        lower_red = np.array([0,0,50])
        upper_red = np.array([100,50,255])

        mask = cv2.inRange(frame, lower_red, upper_red)

        #noise filtering
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.erode(mask, kernel, iterations=2)

        #object detection
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #object cannot be detected
        if(contours==None):
            print("object cannot be detected")
            break

        main_cnt = max(contours, key=cv2.contourArea)
        [x, y, w, h] = cv2.boundingRect(main_cnt)

        res = cv2.bitwise_and(frame, frame, mask=mask)
        res1 = cv2.rectangle(res, (x, y), (x + w, y + h), (0, 0, 255), 5)
        centerX = (int)(x+(w/2))
        centerY = (int)(y+(h/2))

        #draw the centerpoint
        res2 = cv2.circle(res, (centerX, centerY), 3, (255, 0, 0), 2)


        # cv2.imshow('mask', mask)
        cv2.imshow('frame', frame)
        cv2.imshow("res", res)
        # cv2.imshow('res1', res1)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break


video = "testOpenCv.mov"
cap = cv2.VideoCapture(video)
process(cap)
cv2.destroyAllWindows()
cap.release()
