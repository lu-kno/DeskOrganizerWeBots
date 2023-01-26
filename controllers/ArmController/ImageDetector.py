from imageai.Detection import ObjectDetection  
import os
from pprint import pprint
import cv2
import numpy as np
from matplotlib import pyplot as plt
import ctypes
from pathlib import Path
from typing import List, Union, Callable
import json
import math
import warnings
warnings.filterwarnings("ignore", category=UserWarning) 

def imageAiTest():
    imageWidth = 2560
    imageHeight = 1422
    print('imageAiTest() called')
    execution_path = os.getcwd()
    detector = ObjectDetection()
    detector.setModelTypeAsRetinaNet()
    #detector.setModelPath( os.path.join(execution_path , "Modelle/yolov3.pt"))
    detector.setModelPath( os.path.join(execution_path , "Modelle/retinanet_resnet50_fpn_coco-eeacb38b.pth"))
    detector.loadModel()
    custom = detector.CustomObjects(apple=True, orange=True,fork=True,knife=True,spoon=True,mouse=True,bottle=True)
    detections = detector.detectObjectsFromImage(custom_objects=custom,input_image=os.path.join(execution_path , "snapshot.jpg"), output_image_path=os.path.join(execution_path , "imagenew.jpg"), minimum_percentage_probability=1)
    image = cv2.imread("snapshot.jpg")

    for detection in detections:
        xmin, ymin, xmax, ymax = detection['box_points']

        center = getRectangleCenter(xmin,xmax,ymin,ymax)
        relativeCoords = getRelativeCoords(imageWidth, imageHeight, center[0], center[1])
        print(detection["name"] , " : ", detection["percentage_probability"], " : ", relativeCoords )

        # Crop object from the image
        objectImage = image[ymin:ymax, xmin:xmax]
        angle = getAngle(objectImage)
        if angle != 999:
            print(f"Object with class ID {detection['name']} has an average rotation angle of {angle} degrees")
        else:
            print(f"Object with class ID {detection['name']} does not have a clear rotation angle")
        

def getAngle(objectImage):
    color_ranges = [
            (np.array([35, 50, 50]), np.array([70, 255, 255])), # green
            (np.array([90, 50, 50]), np.array([120, 255, 255])), # orange
            (np.array([20, 100, 100]), np.array([30, 255, 255])), # yellow
            (np.array([100, 50, 50]), np.array([140, 255, 255])), # blue
            (np.array([0, 0, 200]), np.array([180, 30, 255])), # white
            (np.array([0, 0, 100]), np.array([180, 255, 150])) # gray
        ]
    total_angle = 0
    counter = 0
    for lower_color, upper_color in color_ranges:
            # Convert the image to the HSV color space
            hsv_image = cv2.cvtColor(objectImage, cv2.COLOR_BGR2HSV)
            # Create a mask for the selected color range
            mask = cv2.inRange(hsv_image, lower_color, upper_color)
            # Apply Gaussian blur to reduce noise
            blur = cv2.GaussianBlur(mask, (3, 3), 0)
            # Apply Canny edge detection
            edges = cv2.Canny(blur, 50, 150)
            # Apply HoughLines to detect lines in the edges
            lines = cv2.HoughLines(edges, 1, np.pi/180, 50)
            # Initialize list to store angles
            angles = []
            if lines is not None:
                for line in lines:
                    for rho, theta in line:
                        # Convert theta from radians to degrees
                        angle = np.degrees(theta)
                        angles.append(angle)
            # Find the average angle
            if len(angles) > 0:
                avg_angle = sum(angles) / len(angles)
                total_angle += avg_angle
                counter += 1
            
    # Calculate the average angle of all color ranges combined
    if counter > 0:
        final_angle = total_angle / counter
        return final_angle
    else:
       return -999


def openCvTest():
    pictureData = findObject("snapshot.jpg","all")



def findObject(imageName, targetColor) -> json:
    # Lade das Bild
    img = cv2.imread(imageName)
    # Konvertieren das Bild in HSV-Farbraum
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if(targetColor == 'red'):
        low_red = np.array([161, 155, 84])
        high_red = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, low_red, high_red)
        #red = cv2.bitwise_and(img, img, mask=mask)  
    elif(targetColor == 'green'):
        low_green = np.array([25, 52, 72])
        high_green = np.array([102, 255, 255])
        mask = cv2.inRange(hsv, low_green, high_green)
        #green = cv2.bitwise_and(img, img, mask=mask)
    elif(targetColor == 'blue'):
        low_blue = np.array([110,50,50])
        high_blue = np.array([130,255,255])
        mask = cv2.inRange(hsv, low_blue, high_blue)
        #blue = cv2.bitwise_and(img, img, mask=mask)
    else:
        # Every color except white
        low = np.array([0, 42, 0])
        high = np.array([179, 255, 255])
        mask = cv2.inRange(hsv, low, high)
        allColors = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow("allColors", allColors)

    # Define range of blue color in HSV
    #lower_blue = np.array([110,50,50])
    #upper_blue = np.array([130,255,255])

    # Mask the image to only select blue colors
    # mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Find the contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Iterieren durch die Konturen
    for cnt in contours:
      # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        # Überprüfen die Anzahl der Ecken der Kontur
        if len(approx) == 3:
      # Dreieck erkannt
            polygon = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            cv2.drawContours(img, [approx], 0, (255, 255, 255), 2)
            M = cv2.moments(cnt)
            if M['m00'] != 0.0:
                mittelpunktX = int(M["m10"] / M["m00"])
                mittelpunktY = int(M["m01"] / M["m00"])
            print("Dreieck - Mittelpunkt: (", mittelpunktX, ",", mittelpunktY, ")")
        elif len(approx) == 4:
            # Rechteck erkannt
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 255), 2)
            mittelpunktX = x + w/2
            mittelpunktY = y + h/2
            print("Rechteck - Mittelpunkt: (", mittelpunktX, ",", mittelpunktY, ")")
        elif len(approx) > 4:
            # Polygon erkannt
            polygon = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            cv2.drawContours(img, [polygon], 0, (255, 255, 255), 2)
            M = cv2.moments(cnt)
            if M['m00'] != 0.0:
                mittelpunktX = int(M["m10"] / M["m00"])
                mittelpunktY = int(M["m01"] / M["m00"])
            print("Polygon - Mittelpunkt: (", mittelpunktX, ",", mittelpunktY, ")")
        else:
            # Kreis erkannt
            (x,y),radius = cv2.minEnclosingCircle(cnt)
            center = (int(x),int(y))
            radius = int(radius)
            cv2.circle(img,center,radius,(255,255,255),2)
            mittelpunktX, mittelpunktY = center
            print("Kreis - Mittelpunkt: (", mittelpunktX, ",", mittelpunktY, ")")

    # 0,0 oben links
    # Anzeigen Sie das bearbeitete Bild
    cv2.imshow("Detected Shapes", img)
    #cv2.imwrite("snapshot_new.jpg", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # do stuff here

    result = dict(position = [mittelpunktX, mittelpunktY],
                category = 'square', # or circle or triangle or polygon
                color = 'targetcolor',
                )

   # result.keys() -> list
    #result.values() -> list 

    return result

def crop_jpg(img, top_percent, bottom_percent, left_percent, right_percent):
    # Get the image height and width
    height, width = img.shape[:2]

    # Calculate the number of pixels to crop from the top, bottom, left and right
    top = int(height * (top_percent / 100))
    bottom = int(height * (bottom_percent / 100))
    left = int(width * (left_percent / 100))
    right = int(width * (right_percent / 100))

    # Crop the image
    img = img[top:-bottom, left:-right]

    return img


def callWeBotsRecognitionRoutine(camera):
    print('callRecognitionRoutine called')
    recObjs = camera.getRecognitionObjects()
    for obj in recObjs:
        print(obj.getModel())

def getRectangleCenter(x1,x2,y1,y2):
    return [ (x1 + x2) / 2, (y1 + y2) / 2 ]

def getRelativeCoords(imageWidth, imageHeight, pointX, pointY):
    return [pointX / imageWidth, pointY / imageHeight]

def edge_detection(img, blur_ksize=5, threshold1=100, threshold2=200):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_gaussian = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
    img_canny = cv2.Canny(img_gaussian, threshold1, threshold2)

    return img_canny