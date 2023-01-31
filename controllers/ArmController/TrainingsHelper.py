from imageai.Detection import ObjectDetection  
from imageai.Detection.Custom import DetectionModelTrainer
from imageai.Detection.Custom import CustomObjectDetection
import os
from pprint import pprint
import cv2
import numpy as np
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
import ctypes
from pathlib import Path
from typing import List, Union, Callable
import json
import math
import warnings
import random
from scipy.ndimage import zoom
import random
from logger import logger
warnings.filterwarnings("ignore", category=UserWarning) 
# imageWidth = 2560
# imageHeight = 1422
SAVEFIGS=True
categories = ['apple', 'orange','can','computer_mouse','hammer','beer_bottle','Cylinder','Cube']

class MyModel(logger):
    def __init__(self, logging='D', logName='ImageAImodel'):
        super().__init__(logging=logging, logName=logName)
        
        self.execution_path = os.path.dirname(__file__)
        self.detector = CustomObjectDetection()
        self.detector.setModelTypeAsYOLOv3()
        self.modelPath = os.path.join(self.execution_path , "Modelle/first/yolov3_DataSet_last.pt")
        self.jsonPath = os.path.join(self.execution_path , "Modelle/first/DataSet_yolov3_detection_config.json")
        self.detector.setModelPath(self.modelPath) # path to custom trained model
        self.detector.setJsonPath(self.jsonPath) # path to corresponding json
        self.detector.loadModel()

    def getObjectsFromImage(self, image):
        
        detections = self.detector.detectObjectsFromImage(input_image=image, 
                                                    output_image_path=os.path.join(self.execution_path ,'snapshot-detected.jpg'),
                                                    nms_treshold = 0.05,
                                                    objectness_treshold = 0.5,
                                                    minimum_percentage_probability = 90)
        
        return detections
        


def startTraining():
    execution_path = os.path.dirname(__file__)
    data_dir_path = os.path.join(execution_path , "DataSet")
    model_path = os.path.join(execution_path , "Modelle/last/yolov3_DataSet_last.pt")
    createClassFiles(categories) 
    trainer = DetectionModelTrainer()
    trainer.setModelTypeAsYOLOv3()
    trainer.setDataDirectory(data_directory=data_dir_path)
    trainer.setTrainConfig(object_names_array=categories, batch_size=32, num_experiments=50, train_from_pretrained_model=model_path)
    trainer.trainModel()

def testModel():
    execution_path = os.path.dirname(__file__)
    detector = CustomObjectDetection()
    detector.setModelTypeAsYOLOv3()
    modelPath = os.path.join(execution_path , "Modelle/first/yolov3_DataSet_last.pt")
    jsonPath = os.path.join(execution_path , "Modelle/first/DataSet_yolov3_detection_config.json")
    detector.setModelPath(modelPath) # path to custom trained model
    detector.setJsonPath(jsonPath) # path to corresponding json
    detector.loadModel()
    detections = detector.detectObjectsFromImage(input_image=os.path.join(execution_path ,'snapshot.jpg'), output_image_path=os.path.join(execution_path ,'snapshot-detected.jpg'),
    nms_treshold = 0.05,
    objectness_treshold = 0.5,
    minimum_percentage_probability = 90)
    for detection in detections:
        print(detection["name"], " : ", detection["percentage_probability"], " : ", detection["box_points"])

def createClassFiles(classes):
    execution_path = os.path.dirname(__file__)
    annotationPathTrain = os.path.join(execution_path , "DataSet/train/annotations/classes.txt")
    annotationPathValidation = os.path.join(execution_path , "DataSet/validation/annotations/classes.txt")
    with open(annotationPathTrain, "w") as file:
        for obj in classes:
            file.write(obj+"\n")
    with open(annotationPathValidation, "w") as file:
        for obj in classes:
            file.write(obj+"\n")

def makeSnapshot(camera,type='train'):
    print('Create training image and corresponding files in mode: '+type)
    recObjs = camera.getRecognitionObjects()
    createTrainingFiles(recObjs,camera,type)


def createTrainingFiles(recognizedObjectes,camera,type):
    fileNamePostfix = 1
    imageWidth = camera.getWidth()
    imageHeight = camera.getHeight()
    execution_path = os.path.dirname(__file__)
    if(type=='train'):
        dir = 'train'
    if(type=='validation'):
        dir = 'validation'
    annotationPath = os.path.join(execution_path , "DataSet/"+dir+"/annotations/")
    jsonPath = os.path.join(execution_path , "DataSet/"+dir+"/raw_data/")
    imagePath = os.path.join(execution_path , "DataSet/"+dir+"/images/")
    if not os.path.exists(annotationPath):
        os.makedirs(annotationPath)
    if not os.path.exists(jsonPath):
        os.makedirs(jsonPath)
    if not os.path.exists(imagePath):
        os.makedirs(imagePath)
    # get current fileName
    
    while True:
        filename = f"image_{fileNamePostfix}.txt"
        filepath = os.path.join(annotationPath, filename)
        if not os.path.isfile(filepath):
            break
        fileNamePostfix += 1
    fileName = f"image_{fileNamePostfix}"
    jsonData = []
    yoloData = []
    for obj in recognizedObjectes:
        id = obj.getId()
        name = obj.getModel()
        if name not in categories:
            continue
        position = list(obj.getPosition())
        positionOnImage = list(obj.getPositionOnImage())
        orientation = list(obj.getOrientation())
        size = list(obj.getSize())
        sizeOnImage = list(obj.getSizeOnImage())
        relativeSize = [sizeOnImage[0]/imageWidth, sizeOnImage[1]/imageHeight]
        relativePosition = [positionOnImage[0]/imageWidth, positionOnImage[1]/imageHeight]
        yoloData.append(f"{categories.index(name)} {relativePosition[0]} {relativePosition[1]} {relativeSize[0]} {relativeSize[1]}\n")
        jsonData.append({
            "id": id,
            "name": name,
            "position": position,
            "positionOnImage": positionOnImage,
            "orientation": orientation,
            "size": size,
            "sizeOnImage": sizeOnImage,
            "relativeSize": relativeSize,
            "relativePosition": relativePosition
        })

    camera.saveImage(imagePath+fileName+".jpg",100)
    with open(jsonPath+fileName+".json", 'w') as file:
        json.dump(jsonData, file, indent=4)   
    with open(annotationPath+fileName+".txt", 'w') as file:
        file.writelines(yoloData)
    print('File: '+fileName+' created')

def moveTableNodes(master,table):
    print('Randomize objects position and rotation on table')
    margin = 0.1
    bottomLeft = table.local2world([0,1,0])
    topLeft = table.local2world([0,0,0])
    bottomRight =table.local2world([1,1,0])
    topRight = table.local2world([1,0,0])
    x_min = bottomLeft[0] + (topRight[0] - bottomLeft[0]) * margin
    x_max = topRight[0] - (topRight[0] - bottomLeft[0]) * margin
    y_min = bottomLeft[1] + (topRight[1] - bottomLeft[1]) * margin
    y_max = topRight[1] - (topRight[1] - bottomLeft[1]) * margin
    for cat in categories:
        obj = master.supervisor.getFromDef(cat)
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        z = bottomLeft[2]#+0.1
        obj.getField('translation').setSFVec3f([x, y, z])
        xRotation = random.uniform(1, 360)
        yRotation = random.uniform(1, 360)
        zRotation = random.uniform(1, 360)
        angle = random.uniform(1, 360)
        obj.getField('rotation').setSFRotation([xRotation,yRotation,zRotation,angle])
    
    # master.sleep(2)
    
    # for cat in categories:
    #     obj = master.supervisor.getFromDef(cat)
    #     obj.getField('linearVelocity').setSFVec3f([0,0,0])
    #     obj.getField('angularVelocity').setSFVec3f([0,0,0])
    #     self.log(f"{cat} linearVelocity: {obj.getField('linearVelocity').setSFVec3f([0,0,0])}")
    #     self.log(f"{cat} angularVelocity: {obj.getField('angularVelocity').setSFVec3f([0,0,0])}")
    

        

lastViewPointPos = 0
count = 0
currentNode = 0
def single_objectImage_setup(supervisor,table,imagesPerViewpoint):
    global count, currentNode, lastViewPointPos
    amountViewpoints = 4
    # init Viewpoint pos for first run
    if(count==0): 
        moveViewPoint(supervisor,lastViewPointPos)
        swapObj(currentNode,table,supervisor)
    # change Viewpoint if given imagesPerViewpoint is met
    if((count % imagesPerViewpoint) == 0):
        lastViewPointPos = (lastViewPointPos+1)%4
        moveViewPoint(supervisor,lastViewPointPos)
    # change object if given parameter is met
    if((count % (imagesPerViewpoint*amountViewpoints))==0):
        currentNode = (currentNode+1) % len(categories)
        swapObj(currentNode,table,supervisor)

    spinTableNode(supervisor,table,currentNode)
    count += 1
    
def moveViewPoint(supervisor,index):
    print(f'moveViewPoint with index: {index} called')
    positionsOld = [[1.1844, -0.101363, 1.13083],
                    [1.50827,-0.565378,1.0785],
                    [1.8847,-0.0885509,1.08804],
                    [1.48313,0.243925,1.06886]]
    orientationsOld = [[0.0315591,0.999032,0.0306494,0.832904],
                    [-0.264641,0.28607,0.920939,1.6946],
                    [-0.358425,0.00291554,0.933554,3.21961],
                    [0.327652,0.330169,-0.88523,1.57763]]

    positions = [[1.02803, -0.117245, 1.67074],
                    [1.53629,-0.679258,1.98548],
                    [2.17341,-0.04583,2.05814],
                    [1.4363,0.745896,2.09031]]

    orientations = [[0.0265831,0.999473,0.0186489,1.08249],
                    [-0.448692,0.473003,0.758251,1.86527],
                    [0.532384,-0.000623067,-0.846503,3.08085],
                    [0.445525,0.437674,-0.780992,1.75545]] 

    viewPoint = supervisor.getFromDef('Viewpoint')
    orientation = viewPoint.getField('orientation')
    position = viewPoint.getField('position')
    orientation.setSFRotation(orientations[index])
    position.setSFVec3f(positions[index])
  

def swapObj(index,table,supervisor):
    print(f'swapObj with index: {index} called')
    baseX = -0.115024
    baseY = -2.20312
    baseZ = 0.9
    for cat in categories:
        obj = supervisor.getFromDef(cat)
        obj.getField('translation').setSFVec3f([baseX, baseY, 0])
    tableCenter = table.local2world([0.5,0.5,0])
    x = tableCenter[0]
    y = tableCenter[1]
    print(tableCenter)    
    supervisor.getFromDef(categories[index]).getField('translation').setSFVec3f([x,y,baseZ])

def spinTableNode(supervisor,table,index):
    obj = supervisor.getFromDef(categories[index])
    tableCenter = table.local2world([0.5,0.5,0])
    x = tableCenter[0]
    y = tableCenter[1]
    z = 0.9
    xRotation = random.uniform(1, 360)
    yRotation = random.uniform(1, 360)
    zRotation = random.uniform(1, 360)
    angle = random.uniform(1, 360)
    obj.getField('rotation').setSFRotation([xRotation,yRotation,zRotation,angle])
    obj.getField('translation').setSFVec3f([x, y, z])

def compare_directories(path1, path2):
    files1 = []
    for file in os.listdir(path1):
        if file != "classes":
            files1.append(os.path.splitext(file)[0])
    files1 = set(files1)
    files2 = []
    for file in os.listdir(path2):
        if file != "classes":
            files2.append(os.path.splitext(file)[0])
    files2 = set(files2)
    only_in_path1 = files1.difference(files2)
    only_in_path2 = files2.difference(files1)
    if 'classes' in only_in_path1:
        only_in_path1.remove('classes')
    if 'classes' in only_in_path2:
        only_in_path2.remove('classes')
    if only_in_path1:
        print("These files are only in", path1, ":", only_in_path1)
    if only_in_path2:
        print("These files are only in", path2, ":", only_in_path2)
    if not only_in_path1 and not only_in_path2:
        return True
    else:
        return False

def dataSetIntegrityTest():
    execution_path = os.path.dirname(__file__)
    annotationPathTrain = os.path.join(execution_path , "DataSet/train/annotations/")
    imagePathTrain = os.path.join(execution_path , "DataSet/train/images/")
    annotationPathValidation = os.path.join(execution_path , "DataSet/validation/annotations/")
    imagePathValidation = os.path.join(execution_path , "DataSet/validation/images/")
    print(f"Trainings folder integrity: {compare_directories(annotationPathTrain,imagePathTrain)}")
    print(f"Validation folder integrity: {compare_directories(annotationPathValidation,imagePathValidation)}")

if __name__=="__main__":
    testModel()
    #startTraining()
    #dataSetIntegrityTest()
    print('DONE')