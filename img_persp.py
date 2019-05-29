import cv2
import numpy as np
import picamera
import picamera.array
import time




def blackThresh(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blk = np.array([0, 0, 0])
    upper_blk = np.array([255, 248, 100])

    mask = cv2.inRange(hsv, lower_blk, upper_blk)

    return mask
def rayas(img,qin, qout):
    #cv2.imshow('Original', img)
    
    rows, cols, chn = img.shape

    rows_2 = int(rows / 2)
    #print("ROWS_2: ", rows_2)
    cols_2 = int(cols / 2)
    #print("COLS_2: ", cols_2)
    x = 180

    pts1 = np.float32([[0, rows_2 - 50], [0, rows], [cols, rows_2 - 50], [cols, rows]])
    pts2 = np.float32([[0, 0], [x, rows], [cols, 0], [cols - x, rows]])

    res = np.zeros((210, 280))

    h, status = cv2.findHomography(pts1, pts2)
    img_pers = cv2.warpPerspective(img, h, (cols, rows))

    res = img_pers[:, 180:540]

    

    blackLine = blackThresh(res)
    #cv2.imshow('BlackLine'+str(time.time()), blackLine)

    kernel = np.ones((5, 5), np.uint8)
    # kernel = np.ones((3,3), np.uint8)
    blackLine = cv2.erode(blackLine, kernel, iterations=5)
    #cv2.imshow('Erode', blackLine)
    blackLine = cv2.dilate(blackLine, kernel, iterations=9)
    #cv2.imshow('Dilate', blackLine)

    edges = cv2.Canny(blackLine, 100, 200)
    #cv2.imshow('Edges', edges)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            cv2.line(res, (x1, y1), (x2, y2), (0, 255, 0), 5)
    

    h, contours, hierarchy = cv2.findContours(blackLine, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    if len(contours) > 0:
        
        c = max(contours, key=cv2.contourArea)
        #new_img = np.empty(img_pers.shape)
        #print(contours)
        #cv2.drawContours(new_img, c, -1, (0, 255, 0), 1)
    
        M = cv2.moments(c)

        cx = int(M['m10'] / M['m00'])

        cy = int(M['m01'] / M['m00'])


        cv2.line(img_pers, (cx + 180, 0), (cx + 180, 720), (255, 0, 0), 1)

        cv2.line(img_pers, (0, cy), (1280, cy), (255, 0, 0), 1)
        for i in range(len(contours[0])):
            contours[0][i] = contours[0][i] + (180, 0)

        cv2.drawContours(img_pers, contours, -1, (0, 255, 0), 1)

        #print((cx, cy))

        if cx + 180 >= cols_2 + 50:
            #print(cx + 180)
            #cv2.circle(img_pers, (cx + 180, cy), int(10), (0, 0, 255), -1, cv2.LINE_AA)
            #cv2.circle(img_pers, (cx, cy), int(10), (25, 103, 255), -1, cv2.LINE_AA)
            #cv2.circle(img_pers, (cols_2 + 20, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            #cv2.circle(img_pers, (cols_2 - 20, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            #print("Turn Right")
            qout.put('R')

        elif cx + 180 <= cols_2 - 50:
            #print(cx + 180)
            #cv2.circle(img_pers, (cx + 180, cy), int(10), (0, 0, 255), -1, cv2.LINE_AA)
            #cv2.circle(img_pers, (cx, cy), int(10), (25, 103, 255), -1, cv2.LINE_AA)
            #cv2.circle(img_pers, (cols_2 + 50, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            #cv2.circle(img_pers, (cols_2 - 50, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            #print("Turn Left")
            qout.put('L')


        else:
            #print(cx + 180)
            #cv2.circle(img_pers, (cx + 180, cy), int(10), (0, 0, 255), -1, cv2.LINE_AA)
            #print("On Track!")
            qout.put('W')



    else:

        #print("I don't see the line")
        qout.put('S')



    # cv2.imshow('Foto Original', img)
    # cv2.imshow('Foto Perspect', img_pers)
    #cv2.waitKey(0)
    # cv2.destroyAllWindows()
