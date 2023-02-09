<style>
body { counter-reset: h1counter h2counter h3counter h4counter h5counter h6counter; }

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


Table of contents
---
## [Jigsaw Puzzle Solver](#jigsaw-puzzle-solver)
### [Table of contents](#table-of-contents)
### [AMS Project Intro](#ams-project-intro)
### [References for Markdown](#references-for-markdown)




<div style="page-break-after: always"></div>


# Report: Autonomous workplace organizer


## Introduction

The purpose of this project is to address the problem of an cluttered work space. The solution we developed is a robotic arm that is designed to clean up and organize the work area. 
In this report we will document and discuss the development process of the project. The report is comprised of three sections. The first part provides a general introduction to the Project, where the project idea as well as technology used will be addressed. 
The main section of this report is divided into two chapters: "Solution theory" and "Implementation".
The "Solution Theory" chapter addresses the problems that needed to be solved in order to realize the project and the corresponding theoretical solutions to solve these problems. 
The "Implementation" chapter provides detailed explanations of how the solutions were actually implemented and draws a comparison between the theoretical solution and the actual implementation. Finally, the last part of the report focuses on the project results and provides a conclusion, evaluating whether we have achieved our project goals and discussing further improvements for the project as well as learning outcomes. 

## Project introduciton

The project idea is to develop a robotic arm that is able to clean up and organize a work area. The robot is equipped with a camera that is used to detect objects on this area. The detected objects are then picked up by the robotic arm and placed in another predefined place.

During the early stages of development we had to decide whether we wanted to use a real robot or a simulation. We decided to use a simulation, because it is easier to develop and test the project in a simulation environment. We chose to use the Webots simulation environment for this project. It is a free and open source simulation environment that is used for research and education. It is based on the ODE physics engine and the OpenGl graphics library. The simulation environment provides a wide range of sensors and actuators that can be used to develop a robot.

The project was developed in a team of two students. The project was divided into three main parts: object detection, coordinate transformation and robot arm control. The first part of the project was to develop a program that is able to detect objects on the work area. The second part was to develop a program that is able to transform the coordinates of the detected objects from the camera coordinate system to the coordinate system of the robot. The third part was to develop a program that is able to control the robot arm and move it to the detected objects.



#### Real life or simulation -> what simulation?

- Pros and cons real life vs simulation
- What project environment?

### Introduction Webots

## Solution Theory (given problems and proposed solutions)

- Milestones or steps needed in project development
- We define which problems we needed to solve and our first approaches to solve these problems
### Object detection

##### Idea how to solve problem,

- first approach

- Pros and cons

### Coord transition

##### ..

### Robot arm

##### ..

## Implementation 
- (implementation of solutions -> going into detail at interesting places )

- Structure corresponds to chapter in solution theory

- Point out differences between previously planned solutions and actual implementation

- Describe how it actually works

### Object detection

- Trainigns image creation process description


### Coord transition

#### ..

### Robot arm

#### ..

## Results

### ..

## Outlook 

### same content as in presentation silde




## References for Markdown

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

