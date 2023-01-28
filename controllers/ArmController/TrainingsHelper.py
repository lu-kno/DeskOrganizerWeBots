from imageai.Detection import ObjectDetection  
from imageai.Detection.Custom import DetectionModelTrainer
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
import random
warnings.filterwarnings("ignore", category=UserWarning) 
imageWidth = 2560
imageHeight = 1422
SAVEFIGS=True
categories = ['dummy','','apple', 'orange', 'bottle','can','computer mouse','knife','fork','hammer','wooden spoon','beer bottle']
fileNamePostfix = 1

def makeSnapshot(camera,type='train'):
    print('Create training image and corresponding files in mode: '+type)
    recObjs = camera.getRecognitionObjects()
    createTrainingFiles(recObjs,camera,type)


def createTrainingFiles(recognizedObjectes,camera,type):
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

def createClassFiles(classes):
    execution_path = os.path.dirname(__file__)
    annotationPathTrain = os.path.join(execution_path , "DataSet/train/annotations/classes.txt")
    annotationPathValidation = os.path.join(execution_path , "DataSet/validation/annotations/classes.txt")
    with open(annotationPathTrain, "w") as file:
        i = 0
        for obj in classes:
            if(i==0):
                file.write("unkown\n")
            else:
                file.write(obj+"\n")
            i += 1
    with open(annotationPathValidation, "w") as file:
        i = 0
        for obj in classes:
            if(i==0):
                file.write("unkown\n")
            else:
                file.write(obj+"\n")
            i += 1

def startTraining():
    createClassFiles(categories[1:]) #Erzeuge "classes.txt" anhand von categorien Liste. Erstes element "dummy" wird ausgelassen
    # trainer = DetectionModelTrainer()
    # trainer.setModelTypeAsYOLOv3()
    # trainer.setDataDirectory(data_directory="DataSet")
    # objectNames = ['apple', 'orange', 'bottle','can','computer mouse','knife','fork','hammer','wooden spoon','beer bottle']
    # trainer.setTrainConfig(object_names_array=objectNames, batch_size=4, num_experiments=200, train_from_pretrained_model="Modelle/yolov3.pt")
    # trainer.trainModel()
    
def moveTableNodes(supervisor,table):
    margin = 0.1
    zCoord = 0.7897645717378102
    bottomLeft = [1.02418,0.784482,zCoord]
    topLeft = [2.01797,0.799213,zCoord]
    bottomRight = [1.02293,-0.993013,zCoord]
    topRight = [1.99722,-0.990657,zCoord]
    x_min = bottomLeft[0] + (topRight[0] - bottomLeft[0]) * margin
    x_max = topRight[0] - (topRight[0] - bottomLeft[0]) * margin
    y_min = bottomLeft[1] + (topRight[1] - bottomLeft[1]) * margin
    y_max = topRight[1] - (topRight[1] - bottomLeft[1]) * margin
    objects = [supervisor.getFromDef('apple'), supervisor.getFromDef('beerBottle'), supervisor.getFromDef('can')]
    for obj in objects:
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        z = bottomLeft[2]
        obj.getField('translation').setSFVec3f([x, y, z])
        xRotation = random.uniform(1, 360)
        yRotation = random.uniform(1, 360)
        zRotation = random.uniform(1, 360)
        angle = random.uniform(1, 360)
        obj.getField('rotation').setSFRotation([xRotation,yRotation,zRotation,angle])
        #print()
    # print(topLeft)
    # print(table.local2world([-1,-1,0])) # top left coords 
    # print('moveTableNodes() called')
    # apple = supervisor.getFromDef('apple')
    # #print(apple.getPosition())
    # fldTranslation = apple.getField('translation')

    # print(dir(apple))
    # #print(apple.getField('size').getSFVec3f())
    # #print(fldTranslation.getSFRotation())
    # #print(fldTranslation.getMFRotation())
    # print(fldTranslation.getSFVec3f())
    # #objects = [obj1, obj2, obj3]
    # fldTranslation.setSFVec3f([1.9618, -0.891729, 0.7897645717378102])
    # for obj in objects:
    #     x = random.uniform(bottomLeft[0], topRight[0])
    #     y = random.uniform(bottomLeft[1], topRight[1])
    #     z = bottomLeft[2]
    #     obj.fldTranslation.setSFVec3f([x, y, z])

if __name__=="__main__":
    startTraining()
    print('DONE')