<style>
body { counter-reset: h1counter h2counter h3counter h4counter h5counter h6counter; 

    //font-size: 12px;
}

h1 { counter-reset: h2counter; }
h2 { counter-reset: h3counter; }
h3 { counter-reset: h4counter; }
h4 { counter-reset: h5counter; }
h5 { counter-reset: h6counter; }
h6 {}

h2:before {
    counter-increment: h2counter;
    content: counter(h2counter) ".\0000a0\0000a0";
}

h3:before {
    counter-increment: h3counter;
    content: counter(h2counter) "." counter(h3counter) ".\0000a0\0000a0";
}

h4:before {
    counter-increment: h4counter;
    content: counter(h2counter) "." counter(h3counter) "." counter(h4counter) ".\0000a0\0000a0";
}

h5:before {
    counter-increment: h5counter;
    content: counter(h2counter) "." counter(h3counter) "." counter(h4counter) "." counter(h5counter) ".\0000a0\0000a0";
}

h6:before {
    counter-increment: h6counter;
    content: counter(h2counter) "." counter(h3counter) "." counter(h4counter) "." counter(h5counter) "." counter(h6counter) ".\0000a0\0000a0";
}

.center-div {
    text-align:center;
}

.center-image {
    display:block; 
    margin:auto;
}

.image-description {
    font-size: 11px;
}

.sub-header {
    font-weight: bold;
}


red {
    color: rgb(255, 0, 0);
}

/* p, li, ul, code {
    font-size: 12px;
    line-height:200%;
} */


</style>


<span>AMS</span>
<br>
<br>
<br>
<br>
<br>
<br>
<br>
<br>
<br>
<br>
<br>

# Robot Desk Organizer

<br>
<br>

**Lu Knoblich, Christian Schmitz**

<br>
<br>
<br>
<div style="page-break-after: always"></div>

# Abstract
- TODO Abstract
<div style="page-break-after: always"></div>

# Table of Content
1. [Robot Desk Organizer](#robot-desk-organizer)
2. [Abstract](#abstract)
3. [Table of Content](#table-of-content)
4. [Report: Autonomous workplace organizer](#report-autonomous-workplace-organizer)
   1. [Introduction](#introduction)
   2. [Project introduction](#project-introduction)
   3. [Solution Theory (given problems and proposed solutions)](#solution-theory-given-problems-and-proposed-solutions)
      1. [Object detection](#object-detection)
      2. [Coordinates transformation](#coordinates-transformation)
         1. [Transformation Matrix](#transformation-matrix)
      3. [Robot controller](#robot-controller)
         1. [Robot Kinematics](#robot-kinematics)
         2. [Organization Routine](#organization-routine)
      4. [Notes for this chapter (to be deleted later)](#notes-for-this-chapter-to-be-deleted-later)
   4. [Implementation](#implementation)
      1. [Object detection](#object-detection-1)
         1. [First approach](#first-approach)
         2. [Custom detection](#custom-detection)
      2. [Notes for this chapter (to be deleted later)](#notes-for-this-chapter-to-be-deleted-later-1)
   5. [Implementation](#implementation-1)
      1. [Object detection](#object-detection-2)
         1. [First approach](#first-approach-1)
         2. [Second approach / Solution](#second-approach--solution)
         3. [Trainings data](#trainings-data)


<!-- ## [Introduction](#introduction)
## [Project introduction](#project-introduction)
## [Solution Theory (given problems and proposed solutions)](#solution-theory-given-problems-and-proposed-solutions)
### [Object detection](#object-detection)
### [Coord transition](#coord-transition)
### [Robot arm](#robot-arm)
## [Implementation](#implementation)
### [Object detection](#object-detection-1)

### [Coord transition](#coord-transition-1)
     
### [Robot arm](#robot-arm-1)
     
## [Results](#Results)
    
## [Outlook](#outlook)

### [same content as in presentation silde](#same-content-as-in-presentation-silde)
## [References for Markdown](#references-for-markdown) -->




# Report: Autonomous workplace organizer

## Introduction

The purpose of this project is to address the problem of an cluttered work space.
The solution we developed is a robotic arm that is designed to clean up and organize the work area. 
In this report we will document and discuss the development process of the project. 

The report is comprised of three sections. The first part provides a general introduction to the Project, where the project idea as well as technology used will be addressed. The main section of this report is divided into two chapters: "Solution theory" and "Implementation".
The "Solution Theory" chapter addresses the problems that needed to be solved in order to realize the project and the corresponding theoretical solutions for these problems. 
The "Implementation" chapter provides detailed explanations of how the solutions were actually implemented and draws a comparison between the theoretical solution and the actual implementation. Finally, the last part of the report focuses on the project results and provides a conclusion, evaluating whether we have achieved our project goals and discussing further improvements for the project as well as learning outcomes. 

## Project introduction

The objective of the project is to create a robotic system capable of tidying and arranging a workspace. The design incorporates a camera that identifies objects within the area, which the robotic arm then grasps and relocates to a designated spot.

- prove of concept for varios real world application
- Solution transferable to other use cases

In the initial phases of the project, the decision was made to utilize a simulation rather than a physical robot. This choice was made due to the ease of testing and development in a simulated environment. The Webots simulation platform was selected for its compatibility with the project, as it is an open-source simulation platform utilized for research and education purposes. The platform is based on the ODE physics engine and the OpenGl graphics library, and offers a broad array of sensors and actuators that can be utilized to develop a robot. Furthermore, Webots integrates various existing robot-devices so that the developed controllers can be used in the real world applications. We chose to use the Irb4600 robot, which is a six-axis industrial robot that is widely used in industry. Additionally the Webots API is provided in various programming languages, including C++, Python, Java, and Matlab. Due to the machine learning and computer vision components of the project, we decided to use Python to implement the developed solution, as it is widely supported in computer vision and machine learning applications.


<div class="center-div">
  <img src="./project2.png"  width="75%" height="75%" class = "center-image" alt="Project setup in Webots" >
  <p class = "image-description">Figure 1: Project setup in Webots </p>
</div>

Figure one shows the project setup in Webots. A camera is used to detect objects in the workspace. The robot-arm is equipped with a gripper that can be utilized to grasp objects. The robot and its devices are controlled by a controller that is responsible for detecting objects, determining the robot's movement, and controlling the gripper. The entire system is self contained and doesn't require human interaction, other devices or an active web connection.

The system was developed by a team of two students and divided into three main components: object detection, coordinate transformation, and robotic arm control. 


## Solution Theory (given problems and proposed solutions)

This chapter addresses the solution concepts for the problems that needed to be solved in order to realize the project and is structured according to the previously mentioned main components of the project: object detection, coordinate transformation, and robotic arm control. 
 
### Object detection
The first component of the project is the object detection. Its purpose is to detect objects in the workspace and determine their relative coordinates and size in the image as well as their orientation in relation to the table. 

To simplify these problems, we decided to use a top-down view of the workspace. This means that the camera is positioned above the workspace, so that a linear correlation between the image and the table coordinates emerges. At the early stages of project development the training of a custom object detection model was not intended and it was planned to utilize an existing model. The YOLOv3 model, a convolutional neural network that is trained to detect objects in images, was selected due to its wide array of object classes and its high performance.

To detect the orientation of an object relative to the table, we decided to use OpenCV, a python library used in computer vision applications which provides a broad array of functions for image processing. The main idea was to determine the contours of the object by converting the image into the HSV color space and applying different filter. The contours are then used to calculate the main orientation of the object, using principle component analysis. 

### Coordinates transformation

Once the objects are detected, the next step is to determine their coordinates in the simulation. 

- TODO: first / theoretical approach to solve problem(s)

Once knowing the coordinates of the detected object in the image, 
the next step is to transform the objects position vector from the image's coordinate system to the world's coordinate system. This type of transformation is best achieved with the use of a transformation matrix.
<!-- the next step is to determine the coordinates of the object in the simulation environment so that its position relative to the robot arm can be used. -->

#### Transformation Matrix

A transformation matrix can be represented as a matrix frame, built from a combination of a rotation matrix, a translation vector, a scaling vector and a perspective projection matrix.

$$
\mathbf{T} = \left[
\begin{array}{ccc|c}
\ast&\ast&\ast&\ast\\
\ast&R   &\ast& T  \\
\ast&\ast&\ast&\ast\\
\hline
\ast&P   &\ast& S
\end{array}
\right]
$$
where:  
* $R$ is the rotation matrix with the dimensions $3\times 3$.  
* $T$ is the translation vector with the dimensions $3\times 1$.  
* $P$ is the perspective projection matrix with the dimensions $1\times 3$.  
* $S$ is the scale factor with the dimensions $1\times 1$ (for uniform or isotropic scaling).

The rotation matrix for a rotation around any given axis given by the unit vector $\mathbf{\vec{u}(x,y,z)}$ by an angle $\theta$ is given by the following formula:

$$
R =
\begin{bmatrix}
u_x^2(1-\cos\theta) + \cos\theta & u_xu_y(1-\cos\theta) - u_z\sin\theta & u_xu_z(1-\cos\theta) + u_y\sin\theta \\
u_xu_y(1-\cos\theta) + u_z\sin\theta & u_y^2(1-\cos\theta) + \cos\theta & u_yu_z(1-\cos\theta) - u_x\sin\theta \\
u_xu_z(1-\cos\theta) - u_y\sin\theta & u_yu_z(1-\cos\theta) + u_x\sin\theta & u_z^2(1-\cos\theta) + \cos\theta \\
\end{bmatrix}
$$



The translation vector is the position of the origin from image's coordinate system in the global simulation's coordinate system, i.e. the distance between both origins given as a three dimensional vector. 

$$
T =
\begin{bmatrix}
x\\
y\\
z\\
\end{bmatrix}
$$

The perspective projection matrix is not used in this project due to the camera orientation being perpendicular to the surface of intereset, but is included for completeness.

The scaling factor as given by the frame above can only be used for isotropic scaling, i.e. scaling in all three dimensions by the same factor. Since we are only interested in scaling in the x and y directions by different amounts, a single scaling factor can not be used by itself and needs to be expanded to a scaling matrix $\mathbf{S}$ with the following form:

$$
\mathbf{S}=
\begin{bmatrix}
S_x & 0 & 0   & 0\\
0 & S_y & 0   & 0\\
0 & 0   & S_z & 0\\
0 & 0   & 0   & 1
\end{bmatrix}
$$


where $S_x$, $S_y$ and $S_z$ are the scaling factors in the x, y and z directions respectively.

With the rotation and translation matrices a temporary Transformation matrix $T_{tmp}$ is built as the frame described previously using the unit 1 as a scaling factor. 

The resulting matrix is then multiplied with the sacling matrix  to obtain the final transformation matrix $T$ as follows. 

$$\mathbf{T} = \mathbf{T_{tmp}} \bullet \mathbf{S}$$
<!-- 
<red> The following is not complete, needs to be reworked and checked </red>

This is done by dividing the dimensions of the table in the simulation by the dimensions of the table in the image. Knowing this, it is possible to generate a transformation matrix

The first step is to determine the dimensions of the table in order to calculate the scale factor between the image and the simulation. Knowing the scale factor, it is necessary to determine the position and orientation of the table, in order to complete the transformation matrix that needs to be use to calculate the coordinates of the object in the simulation.

A transformation matrix is a 4x4 matrix that is used to transform a vector from one coordinate system to another. The transformation matrix is calculated by multiplying the translation matrix, the rotation matrix, and the scale matrix. The translation matrix is used to translate the vector from the origin of the coordinate system to the desired position. The rotation matrix is used to rotate the vector around the origin of the coordinate system. The scale matrix is used to scale the vector to the desired size.



In order to tranform the coordinates from the unit `pixels` to the unit `meters`, the scale matrix is calculated as follows:

$$
\begin{bmatrix}
 width & 0 & 0 & 0 \\
 0 & height & 0 & 0 \\
 0 & 0 & depth & 0 \\
 0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

where `width`, `height`, and `depth` are the dimensions of the table in the simulation. -->

### Robot controller

Additionally, a robot-controller needs to be developed to control the robotic arm and the gripper.

- TODO: first / theoretical approach to solve problem(s)
  
These components will then be integrated into a single routine to detect objects, maneuver the robotic arm to the objects, and relocate the objects to a specified location.

#### Robot Kinematics

    The Robot chosen for this task consists of a robotic arm with a gripper on the end of the arm. Without the gripper, the robot has 6 degrees of freedom.
    Calculating the position of the gripper (the end effector) while knowing the position of each individual motor can be done using a process called forward kinematics, which combines multiple applications of trigonometric formulas.

    Nonetheless, the reverse operation, which aims to calculate the required position of the joints in the kinematic chain given the (desired) position of the end effector presents a more challenging problem. Since the point to which the robot needs to move is defined as a three dimensional vector, it leaves 3 independent parameters, meaning there can be more than one solution for a given point.



    To go around this problem, it is possible to use inverse kinematics

    One option would be to use inverse kinematics to aproximate the required result.

    <red>explain ik in depth</red>

    Since the direction from which the robot is approaching the objects needs to be from above, some of the robots axis can be fixed to a predefined position. This reduces the number of degrees of freedom to 3, which makes it possible to use trigonometry to calculate the required position values for the remaining motors.

    The following figure shows the robot's coordinate system and the position of the objects in the simulation.

    <red>Insert figure from presentation</red>

    The position of the first motor can obtained with $ arctan(\frac{y}{z}) $, where $y$ and $z$ are the y and z coordinates of the object in the simulation.



#### Organization Routine

With the information regarding the objects' position and orientation in the simulation, the following step is to control the robots movement to produce the desired behavior. The robot controller is implemented in Python and uses the Webots API to control the robot. The robot controller is responsible for the following tasks:

Initialization of the robot instance and the devices attached to it: 
* camera
* gripper
* internal variables
* Motor sensors
* Motor actuators

Coordination of the robot's movement.
Coornidation of the different steps required for the organization process.
* Reding the camera's image and forwarding the data to the object detection module.
* Reading the object detection module's output and forwarding the data to the coordinate transformation module.
* Reading the coordinate transformation module's output and forwarding the data to the robotic arm control module.
* Perform the movement as required for the detected objects positions
  


```plantuml
@startuml


:**Simulation Start**;
repeat :Move Robot to HOME Position;
    repeat :Take Picture;
        :Perform Object Detection;
    repeat while (objects found?) is (no) not (yes)

    repeat :Coordinate Transformation;
        :Move object;
        :Remove from found objects;
    repeat while (more objects?) is (yes)
repeat while (Keep Watching?) is (yes)
:stop;
@enduml
```

```plantuml
@startuml
start
    :Move Gripper above Object Position;
    :Open Gripper;
    :Move Gripper down to Object Position;
    :Close Gripper;
    :Move Gripper up;
    :Move Gripper above Target Position;
    :Move Gripper down to Target Position;
    :Open Gripper;
    :Move Gripper up;
stop
@enduml
```



### Notes for this chapter (to be deleted later)
- Milestones or steps needed in project development
- We define which problems we needed to solve and our first approaches to solve these problems
## Implementation 
In this chapter we will describe the implementation of the solutions proposed in the previous chapter. Additionally, there will be a comparison between the theoretical solution and the actual implementation as well as a discussion of the difficulties that were encountered during the development process. The chapter is structured according to the previously mentioned main modules of the project: object detection, coordinate transformation, and robotic arm control.
### Object detection

#### First approach

The first approach to solve the problem of object detection was to use the YOLOv3 model. The model was trained on the COCO dataset, which contains 80 different object classes. During the early stages of development we setup a test scenario in Webots, where we placed various objects in the workspace and used the YOLOv3 model to detect the objects. 

Figure 2 shows the results of the object detection using the YOLOv3 model. The following objects on the workspace are included in the COCO dataset and should therefore be detectable by the model: computer mouse, apple, beer can and orange. The camera perspective in this test scenario was similar to the perspective in the final project setup.  

<div class="center-div">
  <img src="./cvResultExistingModel.jpg"  class = "center-image" alt="Object detection results existing YOLOv3 model" >
  <p class = "image-description">Figure 2: Object detection results YOLOv3 model </p>
</div>


The model was able to detect the beer can with an accuracy of 94 percent. However, the orange only had a likelihood of 71 percent whereas the apple and the computer mouse were not detected at all. 

Although the model was able to identify the beer can the overall performance was not satisfactory and another solution was needed. 

#### Custom detection

The second approach to solve the problem of object detection was to train a custom object detection model. In the project plan, it was not initially planned to train a custom model. However, to streamline the process, the decision was made to utilize the ImageAI library. ImageAI is a Python library that offers a convenient framework for training and utilizing object detection models. In order to reduce the effort needed to train the model, we decided to use transfer learning. 

<p class = "sub-header">Trainings data</p>

- automated data creation in yolo format
  - labeling 
    - code example
  - 

<p class = "sub-header">Training </p>

<p class = "sub-header">Result </p>

### Notes for this chapter (to be deleted later)
- Milestones or steps needed in project development
- We define which problems we needed to solve and our first approaches to solve these problems

## Implementation 
In this chapter we will describe the implementation of the solutions proposed in the previous chapter. Additionally, there will be a comparison between the theoretical solution and the actual implementation as well as a discussion of the difficulties that were encountered during the development process. The chapter is structured according to the previously mentioned main modules of the project: object detection, coordinate transformation, and robotic arm control.
### Object detection

#### First approach
The first approach to solve the problem of object detection was to use the YOLOv3 model. The model was trained on the COCO dataset, which contains 80 different object classes. During the early stages of development we setup a test scenario in Webots, where we placed various objects in the workspace and used the YOLOv3 model to detect the objects. 

Figure 2 shows the results of the object detection using the YOLOv3 model. The following objects on the workspace are included in the COCO dataset and should therefore be detectable by the model: computer mouse, apple, beer can and orange. The camera perspective in this test scenario was similar to the perspective in the final project setup.  

<div class="center-div">
  <img src="./cvResultExistingModel.jpg"  class = "center-image" alt="Object detection results existing YOLOv3 model" >
  <p class = "image-description">Figure 2: Object detection results YOLOv3 model </p>
</div>


The model was able to detect the beer can with an accuracy of 94 percent. However, the orange only had a likelihood of 71 percent whereas the apple and the computer mouse were not detected at all. Although the model was able to identify the beer can the overall performance was not satisfactory and another solution was needed.

#### Second approach / Solution

The second approach to solve the problem of object detection was to train a custom model. In the project plan, it was not initially planned to train an own model. However, to streamline the process, the decision was made to utilize the ImageAI library, a python library that offers a convenient framework for training and utilizing object detection models. 

In order to reduce the effort needed to train the model, we decided to use transfer learning, which is a machine learning method where a model, trained on a large dataset, is used as a starting point for a new model. The new model is then trained, containing the pre-trained weights of the origin model. [1] We chose to use the pre-trained YOLOv3 model, mentioned above, as the basis for transfer learning. 

#### Trainings data 

The first step to train a custom model is to gather and arrange the training data in the YOLO annotation format. In this format the data is divided into two main directories: "train" and "validation". Each of these directories contains two sub-directories: "images" and "annotations". It's recommended to use 80% of the data for training and 20% for validation. The data consists of both images of objects we want to detect and accompanying annotation files. Each image is linked with a corresponding annotation file that shares the same name as the image file and provides information about the objects in the image. The general structure of the object annotation is shown below. 

```prolog
<object-class><x-pos><y-pos><width><height>
```	
The file contains one line for each object in the image. The object class is an integer that represents the type of the object. The value corresponds to a list of objects in another file named "classes.txt" inside the "annotation" directory and is encoded by the index of the object in the list. The x-pos, y-pos, width, and height are the information for the bounding box of the object in the image. The values are normalized to the range [0, 1] and are relative to the width and height of the image.

<p class = "sub-header">Automatization</p>

Instead of creating and labeling the images ourselves we decided to automate the process. The plan was to utilize the object detection feature integrated in Webots to automatically generate the image and annotation files within their respective directories. We only utilized this detection method to create training data, as the detection is not based on image recognition but hard coded within webots. 

Two setups were prepared to generate training data. The first setup was a top-down view of the table with the objects placed on the table. The second setup was a view from the side of the table with individual objects placed on the table and rotated at 4 different view points.

The following steps were taken to automate the process:

1. Create a loop in the robot controller to call the snapshot and object randomization routine a specified number of times.
2. Define a function to take a snapshot of the camera image and create a corresponding annotation file. The function takes a reference to the camera object and a type as parameters and saves the image and annotation file in the respective directories. 
3. Design a function to randomize the position and orientation of the objects in the simulation. The function takes a reference to the robot object as a parameter and randomizes the position and orientation of the objects in the simulation.

- Top down

- 4 angled rotation object 

(The following steps were taken to automate the process:)



- automated data creation in yolo format
  - labeling 
    - code example
  - Table and 4 angle single 

<p class = "sub-header">Training </p>

After the data was created and labeled, the training process could be started. The training was done using the ImageAI library. 

- hardware used
- settings
  - batch size
  - epochs
  - ..
- trainings results
- 

<p class = "sub-header">Result </p>

 - weakness
   - Fragments of objects are detected as the object with a high probability (99%+)
- Still convicing performance if the nms is tweaked right

#### Conclusion

- Framework created to automate the process of creating training data 
  - Possibly transfarable to other projects
    - Depending on the quality of the object animation. (Proto files)
      - number of polygons
      - textures  

#### Notes for this chapter (to be deleted later)
- How the first approach turned out
  - bad accuracy 
  - not enough useable object classes 
  - no Proto files for existing object classes 
- custom detection
  - use of imageAI library 
- custom training
  - transfer learning
  - trainings data
      - randomized objects
      - labeling
      - automatization of data creation
  - training itself
    - Settings
- detection results

### Coord transition

- TODO: description of implementation Coord transition

### Robot arm

- TODO: description of implementation Robot arm
  
### Notes for this chapter (to be deleted later)
- implementation of solutions -> going into detail at interesting places 

- Structure corresponds to chapter in solution theory

- Point out differences between previously planned solutions and actual implementation

- Describe how it actually works

## Results

- TODO: presenting results

## Outlook 

- same content as in presentation silde



<div style="page-break-after: always"></div>

# Sources

[1] Sara Robinson et al., Design Patterns für Machine Learning. Entwurfsmuster  für Datenaufbereitung Modellbildung und MLOps. Sebastopol: O’Reilly, 	2022. S. 186.


<div style="page-break-after: always"></div>

## References for Markdown (to be deleted later)

![time spent on different calls](./timespentoncalls.png)

```prolog
piece(p01, [s00,s01,s02n,s00]).
% (...)

piece(marginLeft,[null,smargin,null,null]).
piece(margin2d,[smargin,smargin,smargin,smargin]).
piece(margin2d,[s00,smargin,smargin,smargin]).
```




This is achieved through the predicate `matchingShapes/2`, which was kept as a separate predicate to avoid the endless loops that would occur if `shapeMatch(A,B):## shapeMatch(B,A).` had been used.


```plantuml
@startyaml
pSearch(Piece, [Upper, Right, Down, Left]):
    0 deg rot.: piece(Piece, [Upper, Right, Down, Left])
    90 deg rot.: piece(Piece, [Left, Upper, Right, Down])
    180 deg rot.: piece(Piece, [Down, Left, Upper, Right])
    270 deg rot.: piece(Piece, [Right, Down Left, Upper])

@endyaml
```

```plantuml
@startuml

:**solutionDownwards**;
    if (previous row __is__ last bottom row) then (no: continue)
        :## matchRight starting on left margin 
          matching upper row shape
    #### add pieces from new row 
          to list of unavailable pieces
    #### solutionDownwards get following row (recursion)
        **return** this row and following rows;
    else (yes: stop)
        : no more rows can be built (puzzle complete)
        **return**  empty list;
    endif
:## list containing current and following rows;
@startuml
```

```plantuml
@startuml

:**matchRight**;
    if (current piece == right margin) then (no: continue)
        :## get required Left and Upper shapes
    #### search piece with matching shapes
    #### check if piece is still available
    #### matchRight on new piece (get following matches)(recursion)
        **return** this piece and following matches;
    else (yes: stop)
        :**return**  empty list;
    endif
:## list of pieces forming built row
## bottom shape of built row;
@startuml
```

|**Call \ PuzzleSize**	|10	|25	|50	|75	|100	|125	|150    |
|:---	|-:	|-:	|-:	|-:	|-:	|-:	|-:    |
|$memberchk/3	|10.40%	|24.90%	|53.00%	|71.60%	|88.00%	|89.40%	|91.30% |
|piece/2	|49.30%	|37.50%	|24.60%	|13.40%	|5.70%	|5.00%	|4.50%  |
|matchRight/6	|25.40%	|5.00%	|4.00%	|2.60%	|0.90%	|0.60%	|0.40%  |
|pieceSearch/3	|0.00%	|16.30%	|5.40%	|2.20%	|0.80%	|1.00%	|0.60%  |
|pSearch/2	|4.50%	|5.30%	|3.50%	|3.40%	|1.00%	|0.90%	|0.70%  |