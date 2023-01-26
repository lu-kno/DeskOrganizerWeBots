# DeskOrganizerWeBots

# Movement

the wrist of the robot now follows the target while positioning the hand/gripper downwards
gripper is closed with "Q" and opened with "E"
starting position is reset with "R"
position data is printed with "F"

"WASD" moves the sphere around in X-Y directions
SPACE and SHIFT+SPACE move the sphere UP or DOWN respectively

Movement directions make sense when "standing" in front of the robot
 


# To Do

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


 - [ ] integrate helper function to move robot to XY coordinates from table to world
 - [ ] (Optional) Get optimal angle to grab object
  
 - [ ] Integrate Image detection with arm controller
 - [ ] Create scan and move routine
    1. Move Arm Away 
    2. Scan Table (Take picture and identify objects)
    3. Get list of objects with [category, position, angle]
    4. For each object:
       1. choose destination based on category
       2. pickNplace(position, destination)

 - [X] choose image recognition toolkit
 - [ ] Get position of object in image as ratio to table size (from 0 to 1)
 - [ ] Detect more than one object on the table
 - [ ] Detect different shapes



# used to avoid tracking of some world files

`git update-index --assume-unchanged worlds/.DeskOrganizer.wbproj `
`git update-index --assume-unchanged worlds/.DeskOrganizer.jpg `


reset with `--no-assume-unchanged` flag