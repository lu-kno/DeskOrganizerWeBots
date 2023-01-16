"""CamRobotController controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#cam_robot = supervisor.getFromDef("robot") #getting Node Obj of camRobot
#print(cam_robot.getField("name").getSFString())
#cam_robot = robot.getDevice("robot")
#time_step = int(robot_cam.getBasicTimeStep())
    
#cam_robot = supervisor.getRobot(cam_robot)
#camera = cam_robot.getDevice("camera")
    
camera = Camera('camera')
camera.enable(timestep)
firstRun  = True



# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    if(firstRun):
        cameraData = camera.getImage()
        # get the gray component of the pixel (5,10)
        gray = Camera.imageGetGray(cameraData, camera.getWidth(), 5, 10)
        red = Camera.imageGetRed(cameraData, camera.getWidth(), 5, 10)
        green = Camera.imageGetGreen(cameraData, camera.getWidth(), 5, 10)
        blue = Camera.imageGetBlue(cameraData, camera.getWidth(), 5, 10)
        print('CamRobotController Output:')
        print(f'  RGB fuer Pixel 5,10 :  [{red},{green},{blue}]')   
    firstRun = False
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
