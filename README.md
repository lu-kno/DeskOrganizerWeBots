# Instructions to run the simulation

1. [Download](https://cyberbotics.com/doc/guide/installation-procedure) Webots following the steps provided in the link. Make sure to install the version that matches your operating system.
   
2. [Download](https://www.python.org/downloads/) Python version 3.10.8 or higher. Make sure to add Python to your PATH.

3. Install dependencies:
   
  ```
  pip install -r requirements.txt
  ```

4. Clone the Git-Repository of the project:

  ```
  git clone https://github.com/lu-kno/DeskOrganizerWeBots.git
  ```
  
5. Start Webots and open the world file `worlds/DeskOrganizer.wbt` from the project folder. As soon as webots is started, the simulation starts automatically. The robot arm will need a few seconds to initialize and will then start moving. The Simulation can be controlled using the start, stop and reset button at the top of the screen.
# Instructions to use the interface
 On autoloop mode, you can press the key 'k' to randomize the position of the object on the table.

 On manual mode you can use the following keys to control the robot:
 WASD to move the sphere around in X-Y-Z directions
 Q and E to open and close the gripper
 1 and 3 to pick up an object or place it down

  Auto and Manual mode can be toggled by setting the `AUTO_LOOP` flag in ArmController.py to True or False respectively.
# Additional information: 
- The "vidoes" directory contains various videos showcasing the robot in action.
  
- The custom trained model is at: project-root\controllers\ArmController\Modelle\yolov3_DataSet_last.pt
  
- The origin model used for transfer learning is available [here]((https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/yolov3.pt/))

Info: The "videos" directory contains various videos showcasing the robot in action.

# DeskOrganizerWeBots

## Movement

the wrist of the robot now follows the target while positioning the hand/gripper downwards
gripper is closed with "Q" and opened with "E"
starting position is reset with "R"
position data is printed with "F"

"WASD" moves the sphere around in X-Y directions
SPACE and SHIFT+SPACE move the sphere UP or DOWN respectively

Movement directions make sense when "standing" in front of the robot
 


## To Do

 - [X] Separate Movement routines to grab objects from table
   - [X] move to XY position and Z+0.3
   - [X] open gripper
   - [X] lower to Z
   - [X] close gripper
   - [X] move back to Z+0.3
   - [X] go to destination XY Z+0.3
   - [X] open gripper to drop
 - [X] Merge pickup and delivery routines to one function pickNplace(ItemPosition, DestinationPosition)
 - 
 - [X] make gripper close and stop once force is detected
 - [X] routine to wait for movement to finish
 - [X] choose gripper force
 - [X] make tip of finger "open" when closing gripper to have a flat grabbing surface
 - [X] objects seem slippery, check ContactProperties (seemed to be unnecessary)
 - [X] only one finger seems to be closing properly, check why (solved itself, idk, maybe by restarting webots)


 - [x] integrate helper function to move robot to XY coordinates from table to world
 - [x] (Optional) Get optimal angle to grab object
  
 - [x] Integrate Image detection with arm controller
 - [x] Create scan and move routine
    1. Move Arm Away 
    2. Scan Table (Take picture and identify objects)
    3. Get list of objects with [category, position, angle]
    4. For each object:
       1. choose destination based on category
       2. pickNplace(position, destination)

 - [X] choose image recognition toolkit
 - [x] Get position of object in image as ratio to table size (from 0 to 1)
 - [x] Detect more than one object on the table
 - [x] Detect different shapes



## Report To Do
  - [x] readme with instructions on how to run the code and how to use the interface (including the keyboard shortcuts) 
    - [x] include webots version and python version used
    - [x] Mention video directory in repo to present results
    - [ ] test install instructions on a fresh machine
  
  - [x] find solution for model download (maybe use git lfs) 
      - [x] remove object detection model from git ignore and push it to repo
  
  - [x] make sure that the git repo is publicly accessible
  
  - [ ] merge project to master branch

  - [x] fix First page
    - [x] matrikelnumbers
    - [x] logo of th koeln instead of kepten and 
  - [x] fix First page
    - [x] matrikelnumbers
    - [x] logo of th koeln instead of kepten and 
  
  - [x] provide new requirements.txt
  
  - [x] remove references for latex
  - [x] fix references in pdf (example at chapter 3.1.2) 
  - [x] Add sources section
  - [x] Add table of illustrations
  - [x] fix indexing of table of contents in pdf
  
  - [ ] export final pdf (attempt in latex)- [DeskOrganizerWeBots](#deskorganizerwebots)
  - [Movement](#movement)
  - [To Do](#to-do)
  - [Report To Do](#report-to-do)
  - [used to avoid tracking of some world files](#used-to-avoid-tracking-of-some-world-files)
  - [sources](#sources)

  - [ ] create zip file with all necessary files for the hand in.
    - [ ] final report
    - [ ] software with source code
    - [ ] readme file with instructions
    - [x] video demonstraions


  - [ ] (optional) config.yaml file to change global flags and maybe set parameters
  - [x] complete source code comments and docstrings (Only first version comments in ArmController, still good enough for hand in)
  - [x] write abstract 
  - [x] record video of robot in action (already recorded by Lu)

  - [ ] implement functionality START_WITH_RANDOMIZED_TABLE
  - [ ] implement functionality ENDLESS_TABLE_RANDOMIZATION
Lu:

- [ ] add code snippets
- [ ] add figures
- [ ] proof read
- [ ] remove comments to self
- [ ] add individual points missing from the sections

## used to avoid tracking of some world files

`git update-index --assume-unchanged worlds/.DeskOrganizer.wbproj `
`git update-index --assume-unchanged worlds/.DeskOrganizer.jpg `


reset with `--no-assume-unchanged` flag


## sources

PCA analysis to detect orientation of objects
https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/