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

 - Movement routine to grab objects from table
   - move to XY position and Z+0.3
   - open gripper
   - lower to Z
   - close gripper
   - move back to Z+0.3
   - go to destination XY Z+0.3
   - open gripper to drop
 - helper function to move robot to XY coordinates on the table
 - choose image recognition toolkit
 - 