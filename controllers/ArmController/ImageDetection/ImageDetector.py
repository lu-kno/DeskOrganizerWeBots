# from __future__ import annotations
from typing import Iterable, Literal, Any
import math
import os
import random
import warnings
from imageai.Detection.Custom import (CustomObjectDetection)
import cv2
import matplotlib
import numpy as np
import yaml
from matplotlib import pyplot as plt
matplotlib.use('TKAgg')
from utils import logger, OUTPUT_DIR

warnings.filterwarnings("ignore", category=UserWarning) 
imageWidth = 2560
imageHeight = 1422
SAVEFIGS=False
MINIMUM_PERCENTAGE_PROBABILITY = 95
categories = ['','apple', 'orange', 'bottle','can','computer_mouse','knife','fork','hammer','wooden_spoon','beer_bottle']


class ImageScanner(logger):
    """
    Class for scanning images for objects and returning their position and orientation. Which extends from the Logger class.
    """
    def __init__(self, master, model: Any = 'webots', logging: str = 'D', logName: str = 'ImageScanner', **kwargs) -> None:
        """
        Initializes an instance of the ImageScanner class.
        Parameters:
            master (RobotArm): reference to the robot arm.
            model (str):  the model of the image scanner. Default value is 'webots'.
            logging (str):  the logging level. Default value is 'D'.
            logName (str):  the name of the log file. Default value is 'ImageScanner'.
            **kwargs (Any):  additional keyword arguments.
            
        """
        super().__init__(logging=logging, logName=logName, **kwargs)
        if model=='webots':
            self.scanImage = self.webotsScan
        else:
            self.scanImage = self.imageAIScan
        self.master=master
        self.camera=master.camera
        self.customModel= CustomModel(logging=logging)
        
    def imageAIScan(self) -> Iterable[dict]:
        """
        Runs image recognition, using the custom model, on the current image and returns a list of recognized objects.
                
        Parameters:
            None
            
        Returns:
            objects (Iterable[dict]): A list of dictionaries representing objects in the environment.
            Each dictionary contains the following key-value pairs:
            - 'name': name of the recognized object
            - 'position': position of the object in the image, as a list of two floats in the range [0, 1], representing the x and y coordinates respectively.
            - 'boxPoints': position of the object in the image, as a list of four integers [x1, y1, x2, y2]
            - 'orientation': orientation of the object in degrees, relative to the x-axis of the image.
        """
        snapshot_path = os.path.join(OUTPUT_DIR, 'snapshot.jpg')
        self.camera.saveImage(snapshot_path,100)
        img = cv2.imread(snapshot_path).astype('uint8')
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
        
            objImage = img[boxPoints[0,1]:boxPoints[1,1],boxPoints[0,0]:boxPoints[1,0]]
            
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
        """
        Performs a scan of the environment using the webots object regocnition.

        Returns:
        objects (Iterable[dict]): A list of dictionaries representing objects in the environment.
        Each contains the following key-value pairs:
        - id: The object ID
        - name: The object model name
        - position: The normalized (x,y) position of the object on the image, where (0,0) is the top-left corner and (1,1) is the bottom-right corner
        - boxPoints: The corner points of the bounding box that encloses the object, in the format [[x1,y1], [x2,y2]]
        - orientation: The orientation of the object in degrees, calculated using the `getAngle` method
        """
        self.camera.saveImage('snapshot.jpg', 100)
        img = cv2.imread('snapshot.jpg')
        if not np.any(img):
            return []

        objectsRes = self.camera.getRecognitionObjects()
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
            oValues = dict(  id=o.getId(), 
                            name=o.getModel(), 
                            position=(pos/img.shape[:2]).tolist(), 
                            boxPoints=boxPoints.tolist(),
                            orientation=self.getAngle(objImage),
                            )
            objects.append(oValues)

        with open(os.path.join(OUTPUT_DIR,'recognitionObject.yaml'),'w+') as f:
            f.write(yaml.dump(objects))
        self.logV(yaml.dump(objects))
    
        return objects
        
    def getAngle(self, objectImage, name: str|None = None, savefig: bool|None = None) -> float:
        """
        Calculates the orientation angle of an object in an image.

        Parameters:
            objectImage (ndarray): The input image containing the object.
            name (str): (optional) The name to be used as a prefix for saved images. Defaults to None.
            savefig (bool): (optional) If True, save images showing the different steps of the process. Defaults to None.

        Returns:
            float: The orientation angle of the object in degrees.

        Raises:
            cv2.error: If an error occurs during the image processing.
        """
        
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
        """
        Returns the main orientation from the contours of an object,

        Parameters:
            img (ndarray): The original input image containing the object.
            edges (ndarray): Image of the identified edges.
   
        Returns:
            float: The orientation angle of the object in radians.

        """
        pts = np.transpose(np.where(edges>1),[1,0]).astype(np.float64)

        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(pts, mean)
        angle = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        
        cntr = (int(mean[0,0]), int(mean[0,1]))
        p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
        p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
        
        return angle, img

class CustomModel(logger):
    """
    Class to use the custom trained model for object detection. Extends from the Logger class. 
    """
    def __init__(self, logging: str = 'D', logName: str = 'ImageAImodel'):
        """
        Initializes an instance of the CustomModel class.
        Parameters:
            logging (str):  the logging level. Default value is 'D'.
            logName (str):  the name of the log file. Default value is 'ImageAImodel'.
            
        """
        super().__init__(logging=logging, logName=logName)
        self.execution_path = os.path.dirname(__file__)
        self.detector = CustomObjectDetection()
        self.detector.setModelTypeAsYOLOv3()
        self.modelPath = os.path.join(self.execution_path , "../Modelle/yolov3_DataSet_last.pt")
        self.jsonPath = os.path.join(self.execution_path , "../Modelle/DataSet_yolov3_detection_config.json")
        self.detector.setModelPath(self.modelPath) # path to custom trained model
        self.detector.setJsonPath(self.jsonPath) # path to corresponding json
        self.detector.loadModel()

    def getObjectsFromImage(self, image) -> list[dict[str,Any]]:
        """
        Performs object detection on an image, using the custom trained model. 

        Parameters:
            image (ndarray): The input image containing the objects to be detected.
   
        Returns:
            detections (Iterable[dict]): A list of dictionaries representing detected objects in image.
            Each dictionary contains the following key-value pairs:
            - 'name': name of the recognized object
            - 'position': position of the object in the image, as a list of two floats in the range [0, 1], representing the x and y coordinates respectively.
            - 'boxPoints': position of the object in the image, as a list of four integers [x1, y1, x2, y2]
        """
        detections = self.detector.detectObjectsFromImage(input_image=image, 
                                                    output_image_path=os.path.join(self.execution_path ,'output','snapshot-detected.jpg'),
                                                    nms_treshold = 0.05,
                                                    objectness_treshold = 0.5,
                                                    minimum_percentage_probability = MINIMUM_PERCENTAGE_PROBABILITY)
        
        return detections



