# Arm controller

Arm Movement:

- Go To position
- keep wrist looking down

Gripper Movement:
- Open
  - Move motors to zero
  - Wait until no force is detected
- Close
  - Force Control
    - Close each Finger a slowly and check for feedback forces
    - if a finger has a feedback force > 1: Stop moving Finger
    - if finger reached its limit: stop moving finger
    - if all fingers stopped: Success! Gripper is closed.
  - Contact surface leveling
    - 3 Joints per finger
    - first and last joints are driven in oposite direction to compensate rotation
      - Contact surface remains vertical, i.e. parallel to expected grabing surface

Movement Routine:

- Move Away to HOME
- Take Picture
  - Detect and Identify Objects
  - Obtain Objects Orientation
  - Return list of objects
- Coordinate Translation from Table/image coordinates to world coordinates
- Organize Table
  - For each object:
    - Pick Up Object
      - Move gripper to position 30 cm above object
      - open Gripper and lower hand to objects position (wait to reach position)*
      - close gripper to grab object
      - raise hand to previous position
    - Place Object
      - Move 30 cm above target location
      - lower hand
      - Open gripper until no force is detected
      - raise hand to previous position

