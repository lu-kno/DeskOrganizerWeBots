import json
import os
import random
from typing import Any
import matplotlib
from imageai.Detection.Custom import (CustomObjectDetection,
                                      DetectionModelTrainer)
matplotlib.use('TKAgg')
# minimum probability threshold for an object to be detected
MINIMUM_PERCENTAGE_PROBABILITY = 95
# defining the categories of objects that can be detected
categories = ['apple', 'orange','can','computer_mouse','hammer','beer_bottle','Cylinder','Cube']
# defining global variables for training data genration process
lastViewPointPos = 0
count = 0
currentNode = 0

def startTraining():
    """ Function to initiate the training process """
    execution_path = os.path.dirname(__file__)
    data_dir_path = os.path.join(execution_path , "DataSet")
    model_path = os.path.join(execution_path , "Modelle/yolov3_DataSet_mAP-0.02411_epoch-10.pt")
    createClassFiles(categories) 
    trainer = DetectionModelTrainer()
    trainer.setModelTypeAsYOLOv3()
    trainer.setDataDirectory(data_directory=data_dir_path)
    trainer.setTrainConfig(object_names_array=categories, batch_size=32, num_experiments=100, train_from_pretrained_model=model_path)
    trainer.trainModel()

def testModel():
    """ Function to test the model """
    execution_path = os.path.dirname(__file__)
    detector = CustomObjectDetection()
    detector.setModelTypeAsYOLOv3()
    modelPath = os.path.join(execution_path , "Modelle/first/yolov3_DataSet_last.pt")
    jsonPath = os.path.join(execution_path , "Modelle/first/DataSet_yolov3_detection_config.json")
    detector.setModelPath(modelPath) # path to custom trained model
    detector.setJsonPath(jsonPath) # path to corresponding json
    detector.loadModel()
    detections: list[dict[str,Any]] = detector.detectObjectsFromImage(input_image=os.path.join(execution_path, 'output' ,'snapshot.jpg'), output_image_path=os.path.join(execution_path , 'output' ,'snapshot-detected.jpg'),
    nms_treshold = 0.05,
    objectness_treshold = 0.5,
    minimum_percentage_probability = 90)
    for detection in detections:
        print(detection["name"], " : ", detection["percentage_probability"], " : ", detection["box_points"])

def createClassFiles(classes):
    """ 
    Function to generate the class files in the train and annotation directories.
    The class files specify which objects can be detected.

    Parameters:
    classes (list of str): A list of strings specifying the objects that can be detected.

    """
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
    """
    Takes a snapshot of the current scene captured by the provided camera, and saves it along with the corresponding 
    annotation files in the specified directory. The annotation files are created using the `createTrainingFiles()`
    method.

    Parameters:
    camera (Camera): An instance of the `Camera` class that captures the current scene.
    type (str): (optional) A string that specifies the type of snapshot to be created. Possible values are 'train' and 'test'. 
        Defaults to 'train'.

    Returns:
    None
    """
    print('Create training image and corresponding files in mode: '+type)
    recObjs = camera.getRecognitionObjects()
    createTrainingFiles(recObjs,camera,type)


def createTrainingFiles(recognizedObjectes,camera,type):
    """
    Creates annotation files in YOLO format and a JSON file with corresponding `rawData` for each detected object in the image.
    The data is obtained using the object detection provided in Webots.

    Parameters:
    recognizedObjects (list): A list of `RecognitionObject` instances that were detected in the current scene.
    camera (Camera): An instance of the `Camera` class that captured the current scene.
    type (str): A string that specifies the type of snapshot for which the annotation files are being created. 
        Possible values are 'train' and 'validation'.

    Returns:
    None
    """
    fileNamePostfix = 1
    imageWidth = camera.getWidth()
    imageHeight = camera.getHeight()

    # init directories
    execution_path = os.path.dirname(__file__)
    if(type=='validation'):
        dir = 'validation'
    else:
        dir = 'train'
    annotationPath = os.path.join(execution_path , "DataSet", dir, "annotations/")
    jsonPath = os.path.join(execution_path , "DataSet", dir, "raw_data/")
    imagePath = os.path.join(execution_path , "DataSet", dir, "images/")
    # create directories if needed
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
    # Iterate recognized objects and create annotation files
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
        # prepare yolo data
        yoloData.append(f"{categories.index(name)} {relativePosition[0]} {relativePosition[1]} {relativeSize[0]} {relativeSize[1]}\n")
        #prepare json data
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
    # save files
    camera.saveImage(imagePath+fileName+".jpg",100)
    with open(jsonPath+fileName+".json", 'w') as file:
        json.dump(jsonData, file, indent=4)   
    with open(annotationPath+fileName+".txt", 'w') as file:
        file.writelines(yoloData)
    print('File: '+fileName+' created')

def moveTableNodes(master,table):
    """
    Randomizes the position and orientation of objects on the table by selecting a random location and rotation for each object.

    Parameters:
    master (Supervisor): The Webots Supervisor object that controls the simulation.
    table (Node): The Webots Node object for the table that the objects will be placed on.

    Returns:
    None
    """
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


def single_objectImage_setup(supervisor,table,imagesPerViewpoint):
    """
    Controls the simulation for generating images of a single object from various viewpoints.

    Parameters:
    supervisor (Supervisor): The Webots Supervisor object that controls the simulation.
    table (Node): The Webots Node object for the table that the objects will be placed on.
    imagesPerViewpoint (int): The number of images to generate per viewpoint.

    Returns:
    None
    """
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
    # call spinTableNode to randomize the orientation of the object each call
    spinTableNode(supervisor,table,currentNode)
    count += 1
    
def moveViewPoint(supervisor,index):
    """
    Moves the viewpoint in the simulation to a specified position and orientation.

    Parameters:
    supervisor (Supervisor): The Webots Supervisor object that controls the simulation.
    index (int): The index of the viewpoint to move to. Must be between 0 and 3, inclusive.

    Returns:
    None
    """
    print(f'moveViewPoint with index: {index} called')
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
    """
    Swaps the current object with a new object from the `categories` list at the specified index, and positions the new object at the center of the table.

    Parameters:
    index (int): The index of the new object to be placed on the table.
    table (Node): The Webots Node object for the table that the objects will be placed on.
    supervisor (Supervisor): The Webots Supervisor object that controls the simulation.

    Returns:
    None
    """
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
    """Compare the content of two directories and check which files are only present in one of them, to 
    ensure that each image file has a corresponding annotation file. If the simulation is interrupted during the training data 
    generation process, this function can be used to verify the integrity of the dataset.

    Parameters:
    path1 (str): A string representing the path to the first directory to compare.
    path2 (str): A string representing the path to the second directory to compare.

    Returns:
    - bool: True if the directories have the same content, False otherwise.
    """
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
    """
    Tests the integrity of a dataset by comparing the contents of two directories.
    The function prints the results of the comparison to the console.

    """
    execution_path = os.path.dirname(__file__)
    annotationPathTrain = os.path.join(execution_path , "DataSet/train/annotations/")
    imagePathTrain = os.path.join(execution_path , "DataSet/train/images/")
    annotationPathValidation = os.path.join(execution_path , "DataSet/validation/annotations/")
    imagePathValidation = os.path.join(execution_path , "DataSet/validation/images/")
    print(f"Trainings folder integrity: {compare_directories(annotationPathTrain,imagePathTrain)}")
    print(f"Validation folder integrity: {compare_directories(annotationPathValidation,imagePathValidation)}")

# If statement to use the script via console rather than simulation
if __name__=="__main__":
    #testModel()
    #startTraining()
    #dataSetIntegrityTest()
    print('DONE')