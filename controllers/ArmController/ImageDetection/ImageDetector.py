# from __future__ import annotations
import typing
from typing import Optional, Iterable, Literal, Any

import ctypes
import json
import math
import os
import random
import warnings
from pathlib import Path
from pprint import pprint
from typing import Callable, List, Union
from imageai.Detection.Custom import (CustomObjectDetection,
                                      DetectionModelTrainer)
import cv2
import matplotlib
import numpy as np
import yaml
from imageai.Detection import ObjectDetection
from matplotlib import pyplot as plt
matplotlib.use('TKAgg')
from scipy.ndimage import zoom
from utils import logger, OUTPUT_DIR

from . import TrainingsHelper


warnings.filterwarnings("ignore", category=UserWarning) 
imageWidth = 2560
imageHeight = 1422
SAVEFIGS=False
MINIMUM_PERCENTAGE_PROBABILITY = 95
categories = ['','apple', 'orange', 'bottle','can','computer_mouse','knife','fork','hammer','wooden_spoon','beer_bottle']


class ImageScanner(logger):
    def __init__(self, master, model: Any|Literal['webots'] = 'webots', logging: str = 'D', logName: str = 'ImageScanner', **kwargs) -> None:
        super().__init__(logging=logging, logName=logName, **kwargs)
        if model=='webots':
            self.scanImage = self.webotsScan
        else:
            self.scanImage = self.imageAIScan
        
        self.master=master
        self.camera=master.camera
        self.customModel= CustomModel(logging=logging)
        
    def imageAIScan(self) -> Iterable[dict]:
        snapshot_path = os.path.join(OUTPUT_DIR, 'snapshot.jpg')
        
        self.camera.saveImage(snapshot_path,100)
        img = cv2.imread(snapshot_path).astype('uint8')
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
        
        
        self.logV(f"img.shape: {img.shape}")
        self.logV(f"img.dtype: {img.dtype}")
        self.logVV(f"img.unique: {np.unique(img)}")
        if not np.any(img):
            return []
        objectsRaw: list[dict[str,Any]] = self.customModel.getObjectsFromImage(img)
        
        objects: list[dict[str,Any]] = []
        
        for obj in objectsRaw:
            
            boxPoints = np.reshape(obj['box_points'],[2,2]) # reshape from [4] to [2x2]
            
            posAbsolute = boxPoints.mean(0)
            pos = posAbsolute/np.array(img.shape[1::-1])
            
            # boxPointsMargin = boxPoints+ np.array([[-5,-5],[5,5]])
            
            
            objImage = img[boxPoints[0,1]:boxPoints[1,1],boxPoints[0,0]:boxPoints[1,0]]
            
            # objImage = img[max(boxPoints[0,0]-5,0):min(boxPoints[1,0]+5,img.shape[0]),max(boxPoints[0,1]-5,0):min(boxPoints[1,1]+5,img.shape[1]),:3]
            printout = f"Name = {obj['name']}\n"
            printout+= f"img.shape [y,x,f] = {img.shape}\n"
            printout+= f"boxPoints  [x1,y1],[x2,y2] = {boxPoints.tolist()}\n"
            printout+= f"pos x,y = {pos}\n"
            printout+= f"objImage.shape [y,x,f] = {objImage.shape}\n"
            printout+= f"np.max(objImage) = {np.max(objImage)}\n"
            printout+= f"===================================="
            self.logD(printout)
            
            
            oValues = dict(name = obj['name'],
                           position = pos.tolist(),
                           boxPoints = boxPoints.tolist(),
                           orientation = self.getAngle(objImage, name=obj['name'], savefig=SAVEFIGS))
            objects.append(oValues)

        with open(os.path.join(OUTPUT_DIR,'recognitionObject.yaml'),'w+') as f:
            f.write(yaml.dump(objects))
        self.logV(yaml.dump(objects))
        return objects
                           
        
    def webotsScan(self) -> Iterable[dict]:
        
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
                            orientation=self.getAngle(objImage),
                            )
            objects.append(oValues)
        
        # print(json.dumps(objects,indent=4))
        with open(os.path.join(OUTPUT_DIR,'recognitionObject.yaml'),'w+') as f:
            f.write(yaml.dump(objects))
        self.logV(yaml.dump(objects))
    
        return objects
        
    def getAngle(self, objectImage, name: str|None = None, savefig: bool|None = None) -> float:
        try:
            # Convert the image to the HSV color space
            hsv_image = cv2.cvtColor(objectImage, cv2.COLOR_BGR2HSV)
            # Apply Canny edge detection
            edges = cv2.Canny(hsv_image, 50, 150)
            
            # Get position of true pixels
            pos: list[tuple[int, int]] = [tuple(i) for i in zip(*np.where(edges>100))]
            
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
            
            orientation, contourNangle = self.getOrientationPCA(cleanEdges,objectImage)
            
            
            if savefig:
                if not name:
                    name=str(random.randrange(999))
                    
                imagePath=os.path.join(OUTPUT_DIR,'savedImages')
                os.makedirs(imagePath, exist_ok=True)
                pathPrefix=os.path.join(imagePath,name)
                os.makedirs(imagePath, exist_ok=True)
                
                plt.imshow(objectImage[...,::-1])
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
        except cv2.error as e:
            self.logE(e)
            return 0

    def getOrientationPCA(self, edges, img) -> tuple:
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

class CustomModel(logger):
    """ Class to create training data as well as train and test the custin nidek """
    def __init__(self, logging: str = 'D', logName: str = 'ImageAImodel'):
        super().__init__(logging=logging, logName=logName)
        
        self.execution_path = os.path.dirname(__file__)
        self.detector = CustomObjectDetection()
        self.detector.setModelTypeAsYOLOv3()
        self.modelPath = os.path.join(self.execution_path , "../Modelle/first/yolov3_DataSet_last.pt")
        self.jsonPath = os.path.join(self.execution_path , "../Modelle/first/DataSet_yolov3_detection_config.json")
        self.detector.setModelPath(self.modelPath) # path to custom trained model
        self.detector.setJsonPath(self.jsonPath) # path to corresponding json
        self.detector.loadModel()

    def getObjectsFromImage(self, image) -> list[dict[str,Any]]:
        
        detections = self.detector.detectObjectsFromImage(input_image=image, 
                                                    output_image_path=os.path.join(self.execution_path ,'output','snapshot-detected.jpg'),
                                                    nms_treshold = 0.05,
                                                    objectness_treshold = 0.5,
                                                    minimum_percentage_probability = MINIMUM_PERCENTAGE_PROBABILITY)
        
        return detections



