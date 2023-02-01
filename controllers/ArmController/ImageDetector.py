from imageai.Detection import ObjectDetection  
import TrainingsHelper
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
import random
from scipy.ndimage import zoom
import yaml
from logger import logger

warnings.filterwarnings("ignore", category=UserWarning) 
imageWidth = 2560
imageHeight = 1422
SAVEFIGS=True
categories = ['','apple', 'orange', 'bottle','can','computer_mouse','knife','fork','hammer','wooden_spoon','beer_bottle']


class ImageScanner(logger):
    def __init__(self, master, model='webots', logging='D', logName='ImageScanner', **kwargs):
        super().__init__(logging=logging, logName=logName, **kwargs)
        if model=='webots':
            self.scanImage = self.webotsScan
        else:
            self.scanImage = self.imageAIScan
        
        self.master=master
        self.camera=master.camera
        self.imageAImodel=TrainingsHelper.MyModel(logging=logging)
        
    def imageAIScan(self):
        
        self.camera.saveImage('snapshot.jpg',100)
        img = cv2.imread('snapshot.jpg').astype('uint8')
        # plt.imshow(img)
        # plt.suptitle('using saveImage and imread')
        # plt.show()
        
        # # The following block is bugged
        # img = np.array(list(self.camera.getImageArray()),dtype='uint8')[:,:,:3]
        # plt.imshow(img)
        # plt.suptitle('using getImageArray and removing alpha')
        # plt.show()
        
        # imgraw = np.array(list(self.camera.getImageArray()),dtype='uint8').reshape((1066, 1920, 3))
        # img = imgraw[:,:,::-1]
        # plt.imshow(img)
        # plt.suptitle('using getImageArray, removing alpha and reshape((HEIGTH, WIDTH, 3)')
        # plt.show()
        # plt.imshow(imgraw)
        # plt.suptitle('using getImageArray, removing alpha and reshape((HEIGTH, WIDTH, 3)')
        # plt.show()
        
        
        self.log(f"img.shape: {img.shape}")
        self.log(f"img.dtype: {img.dtype}")
        self.log(f"img.unique: {np.unique(img)}")
        if not np.any(img):
            return []
        objectsRaw = self.imageAImodel.getObjectsFromImage(img)
        
        objects = []
        
        for obj in objectsRaw:
            
            boxPoints = np.reshape(obj['box_points'],[2,2]) # reshape from [4] to [2x2]
            
            posAbsolute = boxPoints.mean(0)
            pos = posAbsolute/np.array(img.shape[1::-1])
            
            # boxPointsMargin = boxPoints+ np.array([[-5,-5],[5,5]])
            
            
            objImage = img[boxPoints[0,1]:boxPoints[1,1],boxPoints[0,0]:boxPoints[1,0]]
            
            # objImage = img[max(boxPoints[0,0]-5,0):min(boxPoints[1,0]+5,img.shape[0]),max(boxPoints[0,1]-5,0):min(boxPoints[1,1]+5,img.shape[1]),:3]
            self.logD(f"Name = {obj['name']}")
            self.logD(f"img.shape [y,x,f] = {img.shape}")
            self.logD(f"boxPoints  [x1,y1],[x2,y2] = {boxPoints.tolist()}")
            self.logD(f"pos x,y = {pos}")
            self.logD(f"objImage.shape [y,x,f] = {objImage.shape}")
            self.logD(f"np.max(objImage) = {np.max(objImage)}")
            self.logD(f"====================================")
            
            
            oValues = dict(name = obj['name'],
                           position = pos.tolist(),
                           boxPoints = boxPoints.tolist(),
                           orientation = getAngle(objImage, name=obj['name'], savefig=SAVEFIGS))
            objects.append(oValues)

        with open('recognitionObject.yaml','w+') as f:
            f.write(yaml.dump(objects))
        self.logD(yaml.dump(objects))
        return objects
                           
        
    def webotsScan(self):
        
        self.camera.saveImage('snapshot.jpg', 100)
        img = cv2.imread('snapshot.jpg')
        # img = np.array(list(self.camera.getImageArray()),dtype='uint8')[:,:,:3]
        if not np.any(img):
            return []

        objectsRes = self.camera.getRecognitionObjects()
        # print(img)
        objects=[]
        for o in objectsRes:
            size=np.array(list(o.getSizeOnImage()))
            
            pos = np.array(list(o.getPositionOnImage()))
            
            minCorner = (pos-size/2).astype('int')
            maxCorner = (minCorner+size).astype('int')
            
            boxPoints = np.array([minCorner, maxCorner])
            
            objImage = img[max(minCorner[0]-5,0):min(maxCorner[0],size[0]),max(minCorner[1],0):min(maxCorner[1],size[1]),:3]
            self.logD(f"Name = {o.getModel()}")
            self.logD(f"size = {size}")
            self.logD(f"pos  = {pos}")
            self.logD(f"objImage.shape = {objImage.shape}")
            self.logD(f"np.max(objImage) = {np.max(objImage)}")
            # self.logD(f"np.max(img) = {np.max(img)}")
            # self.logD(f"np.shape(img) = {np.shape(img)}")
            
            # objImage = img[]
            oValues = dict(  id=o.getId(), 
                            name=o.getModel(), 
                            position=(pos/img.shape[:2]).tolist(), 
                            boxPoints=boxPoints.tolist(),
                            orientation=getAngle(objImage),
                            )
            objects.append(oValues)
        
        # print(json.dumps(objects,indent=4))
        with open('recognitionObject.yaml','w+') as f:
            f.write(yaml.dump(objects))
        self.logD(yaml.dump(objects))
    
        return objects
        



def imageAiTest(filename="snapshot.jpg"):
    imageWidth = 2560
    imageHeight = 1422
    print('imageAiTest() called')
    # execution_path = os.getcwd()
    execution_path = os.path.dirname(__file__)
    detector = ObjectDetection()
    #detector.setModelTypeAsYOLOv3()
    #detector.setModelPath( os.path.join(execution_path , "Modelle/yolov3.pt"))
    detector.setModelTypeAsRetinaNet()
    detector.setModelPath( os.path.join(execution_path , "Modelle/retinanet_resnet50_fpn_coco-eeacb38b.pth"))
    detector.loadModel()
    custom = detector.CustomObjects(apple=True, orange=True,fork=True,knife=True,spoon=True,mouse=True,bottle=True)
    detections = detector.detectObjectsFromImage(custom_objects=custom,input_image=os.path.join(execution_path , filename), output_image_path=os.path.join(execution_path , "imagenew.jpg"), minimum_percentage_probability=1)
    image = cv2.imread(os.path.join(execution_path , filename))

    for detection in detections:
        xmin, ymin, xmax, ymax = detection['box_points']

        center = getRectangleCenter(xmin,xmax,ymin,ymax)
        relativeCoords = getRelativeCoords(imageWidth, imageHeight, center[0], center[1])
        print(detection["name"] , " : ", detection["percentage_probability"], " : ", relativeCoords )

        # Crop object from the image
        margin=5
        objectImage = image[ymin-margin:ymax+margin, xmin-margin:xmax+margin]
        angle = getAngle(objectImage, name=detection['name'], savefig=SAVEFIGS)
        if angle != 999:
            print(f"Object with class ID {detection['name']} has an average rotation angle of {angle} degrees")
        else:
            print(f"Object with class ID {detection['name']} does not have a clear rotation angle")
        

def getAngle(objectImage, name=None, savefig=None):
    # Convert the image to the HSV color space
    hsv_image = cv2.cvtColor(objectImage, cv2.COLOR_BGR2HSV)
    objectImage = objectImage[...,::-1]
    # Apply Canny edge detection
    edges = cv2.Canny(hsv_image, 50, 150)
    
    # Get position of true pixels
    pos = [i for i in zip(*np.where(edges>100))]
    
    # Init masks
    leftEdgeMask=np.full(np.shape(edges),0)
    rightEdgeMask=np.full(np.shape(edges),0)
    topEdgeMask=np.full(np.shape(edges),0)
    bottomEdgeMask=np.full(np.shape(edges),0)
    
    # Init Boundary
    leftEdge=[1000 for i in range(np.shape(edges)[0])]
    rightEdge=[0 for i in range(np.shape(edges)[0])]
    topEdge=[1000 for i in range(np.shape(edges)[1])]
    bottomEdge=[0 for i in range(np.shape(edges)[1])]
    
    # Position Boundary
    for y,x in pos:
        leftEdge[y] = min(leftEdge[y],x)
        rightEdge[y] = max(rightEdge[y],x)
        topEdge[x] = min(topEdge[x],y)
        bottomEdge[x] = max(bottomEdge[x],y)
        
    # Make Masks from Boundary
    for y,x in enumerate(leftEdge):
        leftEdgeMask[y,x:] = 255
        
    for y,x in enumerate(rightEdge):
        rightEdgeMask[y,:x] = 255
        
    for x,y in enumerate(topEdge):
        topEdgeMask[y:,x] = 255
        
    for x,y in enumerate(bottomEdge):
        bottomEdgeMask[:y,x] = 255
        
    combined_mask = (leftEdgeMask*rightEdgeMask*bottomEdgeMask*topEdgeMask/255**3)
    blur = cv2.GaussianBlur(combined_mask, (11,11), 0)
    
    cleanEdges = cv2.Canny(blur.astype('uint8'),50,150)
    
    orientation, contourNangle = getOrientationPCA(cleanEdges,objectImage)
    
    
    if savefig:
        if not name:
            name=random.randrange(999)
            
        imagePath=os.path.join(os.getcwd(),'savedImages')
        pathPrefix=os.path.join(imagePath,name)
        os.makedirs(imagePath, exist_ok=True)
        
        plt.imshow(objectImage)
        plt.savefig(f'{pathPrefix}_0original.png')
        plt.imshow(hsv_image)
        plt.savefig(f'{pathPrefix}_1HSVspace.png')
        plt.imshow(edges)
        plt.savefig(f'{pathPrefix}_2roughEdges.png')
        plt.imshow(combined_mask)
        plt.savefig(f'{pathPrefix}_3objectArea.png')
        plt.imshow(blur)
        plt.savefig(f'{pathPrefix}_4blurredObjectArea.png')
        plt.imshow(cleanEdges)
        plt.savefig(f'{pathPrefix}_5cleanEdges.png')
        plt.imshow(contourNangle)
        plt.savefig(f'{pathPrefix}_6contourNangle.png')
    
    return orientation


def getOrientationPCA(edges, img):
    '''returns orientation from the contour of an object'''
    pts = np.transpose(np.where(edges>1),[1,0]).astype(np.float64)

    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv2.PCACompute2(pts, mean)
    angle = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
    
    cntr = (int(mean[0,0]), int(mean[0,1]))
    p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
    p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
    # img = drawAxis(img, cntr, p1, (255, 255, 0), 1)
    # img = drawAxis(img, cntr, p2, (0, 0, 255), 5)
    # img[edges>1]= [225,0,225]
    
    return angle, img

def drawAxis(img, p_, q_, color, scale):
  p = list(p_)
  q = list(q_)
 
  ## [visualization1]
  angle = math.atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
  hypotenuse = math.sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
 
  # Here we lengthen the arrow by a factor of scale
  q[0] = p[0] - scale * hypotenuse * math.cos(angle)
  q[1] = p[1] - scale * hypotenuse * math.sin(angle)
  img = cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
 
  # create the arrow hooks
  p[0] = q[0] + 9 * math.cos(angle + math.pi / 4)
  p[1] = q[1] + 9 * math.sin(angle + math.pi / 4)
  img = cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
 
  p[0] = q[0] + 9 * math.cos(angle - math.pi / 4)
  p[1] = q[1] + 9 * math.sin(angle - math.pi / 4)
  img = cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
  
  return img



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
        print('Object detected: '+obj.getModel())


    
def getRectangleCenter(x1,x2,y1,y2):
    return [ (x1 + x2) / 2, (y1 + y2) / 2 ]

def getRelativeCoords(imageWidth, imageHeight, pointX, pointY):
    return [pointX / imageWidth, pointY / imageHeight]

def edge_detection(img, blur_ksize=5, threshold1=100, threshold2=200):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_gaussian = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize), 0)
    img_canny = cv2.Canny(img_gaussian, threshold1, threshold2)

    return img_canny


import matplotlib
matplotlib.use('TKAgg')


if __name__=="__main__":
    imageAiTest()
    print('DONE')



