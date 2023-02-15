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
WASD to move the sphere around in X-Y directions
SPACE and SHIFT+SPACE move the sphere UP or DOWN respectively
Movement directions are from the orientation  "standing" in front of the robot
Q and E to open and close the gripper
starting position is reset with "R"
position data is printed with "F"
1 and 3 to pick up an object or place it down
the wrist of the robot now follows the target while positioning the hand/gripper downwards

  Auto and Manual mode can be toggled by setting the `AUTO_LOOP` flag in ArmController.py to True or False respectively.

  Likewise, other flags in ArmController.py can be set to enable or disable different functionalities.
# Additional information: 
- The "videos" directory contains various videos showcasing the robot in action.
  
- The custom trained model is at: project-root\controllers\ArmController\Modelle\yolov3_DataSet_last.pt
  
- The origin model used for transfer learning is available [here]((https://github.com/OlafenwaMoses/ImageAI/releases/download/3.0.0-pretrained/yolov3.pt/))




# Public Repository

https://github.com/lu-kno/DeskOrganizerWeBots