from imageai.Detection import ObjectDetection  
from imageai.Detection.Custom import DetectionModelTrainer
from imageai.Detection.Custom import CustomObjectDetection
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
# imageWidth = 2560
# imageHeight = 1422
SAVEFIGS=True
categories = ['apple', 'orange','can','computer_mouse','hammer','beer_bottle','Cylinder','Cube']


def startTraining():
    execution_path = os.path.dirname(__file__)
    data_dir_path = os.path.join(execution_path , "DataSet")
    model_path = os.path.join(execution_path , "Modelle/yolov3_DataSet_mixedData_epoch-50.pt")
    createClassFiles(categories) 
    trainer = DetectionModelTrainer()
    trainer.setModelTypeAsYOLOv3()
    trainer.setDataDirectory(data_directory=data_dir_path)
    trainer.setTrainConfig(object_names_array=categories, batch_size=32, num_experiments=30, train_from_pretrained_model=model_path)
    trainer.trainModel()

def testModel():
    execution_path = os.path.dirname(__file__)
    detector = CustomObjectDetection()
    detector.setModelTypeAsYOLOv3()
    detector.setModelPath(os.path.join(execution_path , "Modelle/yolov3_DataSet_last.pt")) # path to custom trained model
    detector.setJsonPath(os.path.join(execution_path , "Modelle/DataSet_yolov3_detection_config.json")) # path to corresponding json
    detector.loadModel()
    detections = detector.detectObjectsFromImage(input_image=os.path.join(execution_path ,
    'snapshot.jpg'), output_image_path=os.path.join(execution_path ,'snapshot-detected.jpg'),
    nms_treshold = 0.1,
    objectness_treshold = 0.6)
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

def moveTableNodes(supervisor,table):
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
        obj = supervisor.getFromDef(cat)
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        z = bottomLeft[2]+0.1
        obj.getField('translation').setSFVec3f([x, y, z])
        xRotation = random.uniform(1, 360)
        yRotation = random.uniform(1, 360)
        zRotation = random.uniform(1, 360)
        angle = random.uniform(1, 360)
        obj.getField('rotation').setSFRotation([xRotation,yRotation,zRotation,angle])

def spinTableNodes(supervisor,table):
    print('Randomize objects rotation on table')
    for cat in categories:
        obj = supervisor.getFromDef(cat)
        xRotation = random.uniform(1, 360)
        yRotation = random.uniform(1, 360)
        zRotation = random.uniform(1, 360)
        angle = random.uniform(1, 360)
        obj.getField('rotation').setSFRotation([xRotation,yRotation,zRotation,angle])

def generateTrainingsData(amount, supervisor, camera, table):
    for i in range(amount):
        moveTableNodes(supervisor,table) #no physics after call?
        makeSnapshot(camera,'train')
            
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
    #testModel()
    startTraining()
    #dataSetIntegrityTest()
    print('DONE')