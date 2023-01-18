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
 - [ ] Merge pickup and delivery routines to one function moveObject(ItemPosition, DestinationPosition)


 - [ ] integrate helper function to move robot to XY coordinates from table to world
 - [X] make gripper close and stop once force is detected
 - [X] routine to wait for movement to finish
 - [ ] choose gripper force

 - [X] choose image recognition toolkit
 - [ ] Get position of object in image as ratio to table size (from 0 to 1)
 - [ ] Detect more than one object on the table
 - [ ] Detect different shapes



