from imageai.Detection import ObjectDetection  
import os
from pprint import pprint
import cv2
import numpy as np
from matplotlib import pyplot as plt
import ctypes
from pathlib import Path
from typing import List, Union, Callable

def detectShapes():
    # reading image
    img = cv2.imread('snapshot.jpg')
    
    # converting image into grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    # using a findContours() function
    contours, _ = cv2.findContours(
        threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    i = 0
    
    # list for storing names of shapes
    for contour in contours:
    
        # here we are ignoring first counter because 
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue
    
        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.01 * cv2.arcLength(contour, True), True)
        
        # using drawContours() function
        cv2.drawContours(img, [contour], 0, (0, 0, 255), 5)
        x=0
        y=0
        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])
    
        # putting shape name at center of each shape
        if len(approx) == 3:
            cv2.putText(img, 'Triangle', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        elif len(approx) == 4:
            cv2.putText(img, 'Quadrilateral', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        elif len(approx) == 5:
            cv2.putText(img, 'Pentagon', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        elif len(approx) == 6:
            cv2.putText(img, 'Hexagon', (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        else:
            cv2.putText(img, 'circle', (x, y),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
        # displaying the image after drawing contours
    cv2.imshow('shapes', img)
        
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test():
    execution_path = os.getcwd()
    
    detector = ObjectDetection()
    detector.setModelTypeAsYOLOv3()
    detector.setModelPath( os.path.join(execution_path , "Modelle/yolov3.pt"))
    detector.loadModel()
    detections = detector.detectObjectsFromImage(input_image=os.path.join(execution_path , "snapshot.jpg"), output_image_path=os.path.join(execution_path , "imagenew.jpg"), minimum_percentage_probability=30)
    
    for eachObject in detections:
        print(eachObject["name"] , " : ", eachObject["percentage_probability"], " : ", eachObject["box_points"] )
        print("--------------------------------")


def test2():
    # Laden das Bild
    img = cv2.imread('snapshot.jpg')

    # Konvertieren das Bild in HSV-Farbraum
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define range of blue color in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    # Mask the image to only select blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Find the contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterieren durch die Konturen
    for cnt in contours:
        # Überprüfen die Anzahl der Ecken der Kontur
        if len(cnt) == 4:
            # Rechteck erkannt
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            mittelpunktX = x + w/2
            mittelpunktY = y + h/2
            print("Rechteck - Mittelpunkt: (", mittelpunktX, ",", mittelpunktY, ")")
        elif len(cnt) > 4:
            # Polygon erkannt
            polygon = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            cv2.drawContours(img, [polygon], 0, (0, 255, 0), 2)
            M = cv2.moments(polygon)
            mittelpunktX = int(M["m10"] / M["m00"])
            mittelpunktY = int(M["m01"] / M["m00"])
            print("Polygon - Mittelpunkt: (", mittelpunktX, ",", mittelpunktY, ")")
        else:
            # Kreis erkannt
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(img,center,radius,(0,255,0),2)
            mittelpunktX, mittelpunktY = center
            print("Kreis - Mittelpunkt: (", mittelpunktX, ",", mittelpunktY, ")")
    cv2.putText(img, 'Text bei 0,0', (0, 0),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    # Anzeigen Sie das bearbeitete Bild
    cv2.imshow("Detected Shapes", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

