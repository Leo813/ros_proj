#!/usr/bin/env python
# coding: utf-8

  

import imutils
import cv2
import time
import numpy as np
import matplotlib.pyplot as plt

from IPython.display import clear_output


plt.rcParams['figure.figsize'] = [18, 8]

def shapedetector(c):
    shape = "N.A"
    perimeter = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)

    if len(approx) == 3:
        shape = "triangle"
    elif len(approx) == 4:
        (x, y, w, h) = cv2.boundingRect(approx)
        ar = w / float(h)

        if ar >= 0.95 and ar <= 1.05:
            shape = "square" 
        else:
            ux = approx.item(2) - approx.item(0)
            uy = approx.item(3) - approx.item(1)
            vx = approx.item(6) - approx.item(0)
            vy = approx.item(7) - approx.item(1)
            vector_1 = [ux, uy]
            vector_2 = [vx, vy]
            unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
            unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arccos(dot_product)
            angle = angle*180/np.pi

            if angle >= 88 and angle <= 92:
                shape = "rectangle"
            else:
                #TODO (Homework): parallelogram, trapezoid and diamond
                shape = "4SIDES" 

    elif len(approx) == 5:
        shape = "pentagon"
    elif len(approx) == 6:
        shape = "hexagon"
        # otherwise, we assume the shape is a circle
    elif len(approx) == 10:
        shape = "star"
    else:
        shape = "circle"

    return shape

def main():
    image = cv2.imread("shapes.jpg")
    resized = cv2.imread("shapes.jpg")
#     resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])

    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    for i,c in enumerate(cnts):
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio)
        cY = int((M["m01"] / M["m00"]) * ratio)
        shape = shapedetector(c)
        c = c.astype("float")
        c *= ratio
        c = c.astype("int")
        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.23, (255, 255, 255), 2)
        
        print("Controur {}. I found one controur with the shape of a {}.".format(i+1, shape))
        clear_output(wait=True)
        plt.clf()
        plt.imshow(image)
        plt.show()
        print("\n\n\n")
        time.sleep(0.5)
        

main()
 
