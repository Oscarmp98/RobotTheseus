import numpy as np
import cv2
from operator import itemgetter


def show(img, name='image', save=False):
    pass
    # Show Original
    # cv2.imshow(name, img)
    # if save:
    #      cv2.imwrite(name + '.png', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


def getCircles(thresh):
    mask = np.zeros((thresh.shape[0], thresh.shape[1], 1), np.uint8)  # Máscara para extraer el círculo

    _,contours, h = cv2.findContours(thresh, 1, 2)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 20:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            minR = cv2.contourArea(np.int0(box))
            ratio = minR/area
            if ratio < 1.5 and ratio > 1.2:
                approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

                if len(approx) > 10 :
                    cv2.drawContours(mask, [cnt], 0, (255, 255, 255), -1)

    return mask


def getRectangle(thresh):
    mask = np.zeros((thresh.shape[0], thresh.shape[1], 1), np.uint8)  # Máscara para extraer el círculo

    _,contours, h = cv2.findContours(thresh, 1, 2)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 20:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            minR = cv2.contourArea(np.int0(box))
            if minR / area < 1.5:
                approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
                if len(approx) == 4 :
                    cv2.drawContours(mask, [cnt], 0, (255, 255, 255), -1)

    return mask

def getSelected(node,shape):
    mask = np.zeros((shape[0], shape[1], 1), np.uint8)
    cv2.drawContours(mask, [node], 0, (255, 255, 255), -1)
    return mask

def getRectVec(thresh):
    _,contours, h = cv2.findContours(thresh, 1, 2)

    coords = []

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4 and cv2.contourArea(cnt) > 50:
            coords.append(approx)

    return coords


def nodeSelect(nodelist, im_shape):
    cam_x = im_shape[1] / 2
    cam_y = im_shape[0]
    cam = np.array([[cam_x, cam_y]], dtype=float)
    
    

    closest = []
    c_pos = []
    c_dist = np.Infinity
    
    for node in nodelist:
        
        
        center = (node[0] + node[1] + node[2] + node[3])/4
        #print('center: ',center)
        coords = center - cam
        #print('coords:',coords)        
        dist = np.linalg.norm(coords)
        if dist < c_dist:
            closest = node
            c_dist = dist
            c_pos = coords
    #print('c_pos:',c_pos)
    return closest, c_pos


def getHomo(img, node):
    # pts_dst = np.array([(30, 30), (130, 30), (130, 130), (30, 130)], dtype=float)
    # h, status = cv2.findHomography(node, pts_dst)
    # im_dst = cv2.warpPerspective(img, h, (160, 160))
    # show(im_dst, 'Homo', True)

    pts_dst = np.array([(0, 0), (100, 0), (100, 100), (0, 100)], dtype=float)
    h, status = cv2.findHomography(node, pts_dst)
    im_dst = cv2.warpPerspective(img, h, (100, 100))
    show(im_dst, 'Homo', True)
    return im_dst

def getContent(img, mask):
    # Obtener contenido
    mask_RGB = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # change mask to a 3 channel image
    content = cv2.subtract(mask_RGB, img)
    return cv2.subtract(mask_RGB, content)


def blackThresh(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blk = np.array([0, 0, 0])
    upper_blk = np.array([255, 248, 130])

    mask = cv2.inRange(hsv, lower_blk, upper_blk)

    return mask


def purpleThresh(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blk = np.array([110, 30, 0])
    upper_blk = np.array([200, 255, 220])

    mask = cv2.inRange(hsv, lower_blk, upper_blk)
    return mask


def greenThresh(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 40, 35])
    upper_green = np.array([100, 255, 240])

    mask = cv2.inRange(hsv, lower_green, upper_green)
    return mask


def colorThresh(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_green = np.array([0, 50, 0])
    upper_green = np.array([255, 255, 255])

    mask = cv2.inRange(hsv, lower_green, upper_green)
    return mask


def updateThresh(thresh, mask):
    # Obtener contenido

    return cv2.multiply(mask, thresh)


def codeValue(content):
    thresh = purpleThresh(content)  # Obtener Grayscale para los contornos

    show(thresh, 'code',True)

    kernel = np.ones((4, 4), np.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=1)

    show(thresh, 'erodes', True)

    _,contours, h = cv2.findContours(thresh, 1, 2)

    value = 0



    for cnt in contours:
        value += 1

    return value
