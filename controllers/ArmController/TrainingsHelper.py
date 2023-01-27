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
warnings.filterwarnings("ignore", category=UserWarning) 
imageWidth = 2560
imageHeight = 1422
SAVEFIGS=True
categories = ['dummy','','apple', 'orange', 'bottle','can','computer mouse','knife','fork','hammer','wooden spoon','beer bottle']

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
    i = 1
    while True:
        filename = f"image_{i}.txt"
        filepath = os.path.join(annotationPath, filename)
        if not os.path.isfile(filepath):
            break
        i += 1
    fileName = f"image_{i}"
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
    createClassFiles(categories[1:]) #Erzeuge "classes.txt" anhand von categorien Liste. Erstes element "leer" wird ausgelassen
    # trainer = DetectionModelTrainer()
    # trainer.setModelTypeAsYOLOv3()
    # trainer.setDataDirectory(data_directory="DataSet")
    # objectNames = ['apple', 'orange', 'bottle','can','computer mouse','knife','fork','hammer','wooden spoon','beer bottle']
    # trainer.setTrainConfig(object_names_array=objectNames, batch_size=4, num_experiments=200, train_from_pretrained_model="Modelle/yolov3.pt")
    # trainer.trainModel()
    

if __name__=="__main__":
    startTraining()
    print('DONE')