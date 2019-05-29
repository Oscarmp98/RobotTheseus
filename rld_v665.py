from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
#codigo steppers--------------------------
#-*- coding: UTF-8 -*-
# Librerias
import RPi.GPIO as GPIO
import time
import numpy as np
import time

#GPIO.setmode(GPIO.BCM)
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)

pins = [7,11,13,15,31,33,35,37]

for pin in pins:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)

# Variables predefinidas
ratio = float(512)/float(360) #Ratio de ciclos por grado
radius = 4.36  #Radio de la rueda en cm
center = 13.53  #Distancia entre la rueda y el centro del robot en cm
pi = 3.1415926535


move_seq = np.array([
  [1,0,0,1,1,0,0,1],
  [0,0,0,1,0,0,0,1],
  [0,0,1,1,0,0,1,1],
  [0,0,1,0,0,0,1,0],
  [0,1,1,0,0,1,1,0],
  [0,1,0,0,0,1,0,0],
  [1,1,0,0,1,1,0,0],
  [1,0,0,0,1,0,0,0]
])
rotate_seq = np.array([
  [1,0,0,1,1,0,0,0],
  [0,0,0,1,1,1,0,0],
  [0,0,1,1,0,1,0,0],
  [0,0,1,0,0,1,1,0],
  [0,1,1,0,0,0,1,0],
  [0,1,0,0,0,0,1,1],
  [1,1,0,0,0,0,0,1],
  [1,0,0,0,1,0,0,1]
])


# Funciones de movimiento

def move(dist):
  
  
  # Se calcula la direccion
  dir = True
  if (dist < 0):
    dist = abs(dist)
    dir = False

  # Se calculan los ciclos que deben girar los motores
  deg = (dist/radius)*(180/pi)
  cycles = deg * ratio
  cycles = int(cycles)
  print("moving "+str(dist)+" cm")

  # Se calcula la matriz de movimiento en funcion de la direccion
  seq = move_seq
  if(dir == False):
    seq = np.flip(seq,0)

  # Se manda la matriz de movimiento por los pines de la GPIO
  for i in range(cycles):
    for halfstep in range(8):
      for pin in range(8):
        GPIO.output(pins[pin], int(seq[halfstep][pin]))
      time.sleep(0.001)
  time.sleep(0.2)

def rotate(rot):
  # Se calcula la direccion
  dir = True
  if (rot < 0):
    rot = abs(rot)
    dir = False

  # Se calcula el numero de ciclos que deben girar los motores
  deg = ((rot*center)/radius)/2
  cycles = deg * ratio
  cycles = int(cycles)
  print("rotating " + str(rot) + " degrees")

  # Se calcula la matriz de rotacion en funcion de la rotacion
  seq = rotate_seq.copy()
  if (dir == False):
    seq = np.flip(seq, 0)

  # Se manda la matriz de rotacion por los pines de la GPIO
  for i in range(cycles):
    for halfstep in range(8):
      for pin in range(8):
        GPIO.output(pins[pin], int(seq[halfstep][pin]))
      time.sleep(0.001)
  time.sleep(0.2)

def followOrders(order):
  rotate(order[0])
  move(order[1])
#-----------------------------------------

camera = PiCamera()

#camera.resolution = (640,360)
camera.resolution = (720,480)
camera.rotation = 270
#rawCapture = PiRGBArray(camera, size=(640,360))
rawCapture = PiRGBArray(camera, size=(720,480))

actual_order = np.array([0,17])
next_order = np.array([0,0])

def blackThresh(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blk = np.array([0, 0, 0])
    upper_blk = np.array([255, 248, 100])

    mask = cv2.inRange(hsv, lower_blk, upper_blk)

    return mask
    
    
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
    image = frame.array
    
    rows, cols, ch = image.shape
    
    rows_2 = int(rows / 2)
    cols_2 = int(cols / 2)
    
    image = cv2.GaussianBlur(image,(7,7),0)
    
    x = 180

    pts1 = np.float32([[0, rows_2 - 50], [0, rows], [cols, rows_2 - 50], [cols, rows]])
    pts2 = np.float32([[0, 0], [x, rows], [cols, 0], [cols - x, rows]])

    res = np.zeros((210, 280))

    h, status = cv2.findHomography(pts1, pts2)
    img_pers = cv2.warpPerspective(image, h, (cols, rows))

    res = img_pers[:, 180:540]
    rrows, rcols, rch = res.shape
    res = res[0:(rrows-1),:]
    blackLine = blackThresh(res)
    #cv2.imshow('p',res)
    #cv2.imshow('x',blackLine)
    #kernel = np.ones((5, 5), np.uint8)
    kernel = np.ones((3,3), np.uint8)
    blackLine = cv2.erode(blackLine, kernel, iterations=5)
    #cv2.imshow('Erode', blackLine)
    blackLine = cv2.dilate(blackLine, kernel, iterations=9)
    #cv2.imshow('Dilate', blackLine)

    edges = cv2.Canny(blackLine, 100, 200)
    #cv2.imshow('edges', edges)
    #lines = cv2.HoughLinesP(blackLine, 1, np.pi / 180, 50, maxLineGap=50)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)
    angles = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angles.append( (np.arctan((x2-x1)/(y2-y1)))* (180 / np.pi))
            cv2.line(res, (x1, y1), (x2, y2), (0, 255, 0), 5)
    alphas = []
    for alpha in angles:
        if alpha <= 60 and alpha >=-60:
            alphas.append(alpha)
    print(alphas)
    #cv2.imshow('ed',res)
    #img_2 = img.copy()

    img_blk, contours, hierarchy = cv2.findContours(blackLine, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.circle(img_pers, (cols_2, rows_2), int(10), (255, 0, 0), -1, cv2.LINE_AA)

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        M = cv2.moments(c)

        cx = int(M['m10'] / M['m00'])

        cy = int(M['m01'] / M['m00'])

        cv2.line(img_pers, (cx + 180, 0), (cx + 180, 720), (255, 0, 0), 1)

        cv2.line(img_pers, (0, cy), (1280, cy), (255, 0, 0), 1)
        for i in range(len(contours[0])):
            contours[0][i] = contours[0][i] + (180, 0)

        cv2.drawContours(img_pers, contours, -1, (0, 255, 0), 1)

        print((cx, cy))

        if cx + 180 >= cols_2 + 50:
            #print(cx + 180)
            cv2.circle(img_pers, (cx + 180, cy), int(10), (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(img_pers, (cx, cy), int(10), (25, 103, 255), -1, cv2.LINE_AA)
            cv2.circle(img_pers, (cols_2 + 20, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            cv2.circle(img_pers, (cols_2 - 20, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            #print("Turn right!")
            next_order[0] = -90
            next_order[1] = 0
            #rotate(-10)

        # elif cx+180 < cols_2 + 30 and cx > cols_2 - 50:
        #    print(cx+180)
        #    cv2.circle(img_pers, (cx+180, cy), int(10), (0, 0, 255), -1, cv2.LINE_AA)
        #    print("On Track!")

        elif cx + 180 <= cols_2 - 30:
            #print(cx + 180)
            cv2.circle(img_pers, (cx + 180, cy), int(10), (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(img_pers, (cx, cy), int(10), (25, 103, 255), -1, cv2.LINE_AA)
            cv2.circle(img_pers, (cols_2 + 30, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            cv2.circle(img_pers, (cols_2 - 50, cy), int(10), (200, 103, 255), -1, cv2.LINE_AA)
            #print("Turn left")
            next_order[0] = 90
            next_order[1] = 0
            #rotate(10)

        else:
            #print(cx + 180)
            cv2.circle(img_pers, (cx + 180, cy), int(10), (0, 0, 255), -1, cv2.LINE_AA)
            #print("On Track!")
            next_order[0] = 0
            next_order[1] = 2
            #move(5)


    else:
        #rotate(45)
        print("I don't see the line")
    #cv2.imshow('Original', image)
    # movimiento
    #followOrders(actual_order)
    #actual_order = next_order
    # visualiacion
    cv2.imshow('x', img_pers)
    rawCapture.truncate(0)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
        

cv2.waitKey(0)
cv2.destroyAllWindows()
GPIO.cleanup()
