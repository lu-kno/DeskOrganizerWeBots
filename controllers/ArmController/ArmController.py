import sys
import tempfile
import numpy as np
import math
import ImageDetector
import time
from controller import Supervisor, Robot, Camera

try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')
    
if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')
IKPY_MAX_ITERATIONS = 4


    
def looper(func):
    def inner(self,*args,**kwargs):
        while self.supervisor.step(self.timestep) != -1:
            if func(self, *args,**kwargs)==-1:
                return
    return inner    

def looperTimeout(func):
    def inner(self,*args,**kwargs):
        timeout = 10000
        while self.supervisor.step(self.timestep) != -1:
            if func(self, *args,**kwargs)==-1:
                return

            timeout-=self.timestep
            if timeout<0:
                print(f'TIMED OUT: {func.__name__}')
                return
    return inner

class MyGripper:
    SPEED = 0.05 # SPEED in DEGREES
    GRIP_FORCE = 1.5
    
    def __init__(self,master):
        self.f1 = [master.supervisor.getDevice(f'finger_1_joint_{i}') for i in [1,2,3]]
        self.f2 = [master.supervisor.getDevice(f'finger_2_joint_{i}') for i in [1,2,3]]
        self.f3 = [master.supervisor.getDevice(f'finger_middle_joint_{i}') for i in [1,2,3]]
        self.fingers = [self.f1,self.f2,self.f3]
        # self.enableForceFeedback()
        
        self.master = master
        self.supervisor = master.supervisor
        self.timestep = master.timestep
        
    @looperTimeout
    def closeOld(self):
        # inc = 0.5
        closed = np.array([0 for f in self.fingers])
        maxDiff = 0
        for i, f in enumerate(self.fingers):
            maxDiff = max(abs(f[0].getPositionSensor().getValue() - f[0].getMaxPosition()),maxDiff)
            # f[0].setPosition(min(f[0].getPosition()+inc,f[0].getMaxPosition))
            if f[0].getForceFeedback()<self.GRIP_FORCE:
                f[0].setPosition(f[0].getMaxPosition())
            else:
                f[0].setPosition(f[0].getPositionSensor().getValue())
                closed[i] = 1
                
        if np.all(closed==1):
            return -1
            
        if maxDiff<0.05:
            print('Gripper Closed Completely')
            return -1
        
    @looperTimeout
    def close(self):
        inc = self.SPEED * np.pi/180 * self.timestep
        forces = np.array([f[0].getForceFeedback() for f in self.fingers])
        maxPositions = np.array([f[0].getMaxPosition() for f in self.fingers])
        minPositionsTip = np.array([f[2].getMinPosition() for f in self.fingers])
        
        positions = np.array([f[0].getPositionSensor().getValue() for f in self.fingers])
        # currentPositionsTip = np.array([f[2].getPositionSensor().getValue() for f in self.fingers])
        closedFingers = []
                
        print(f'speed -> {inc/self.timestep}')
        # print(f'meanForce -> {np.mean(forces)}')
        # print(f'maxForce -> {np.max(forces)}')
        print(f'Forces:  {"   ".join([f"{f:>7.2f}" for f in forces])}')
        
        for finger, force, maxPos, minPosTip, pos in zip(self.fingers,forces,maxPositions, minPositionsTip,positions): #
            if abs(pos-maxPos)<0.05 or force>self.GRIP_FORCE:
                closedFingers.append(True)
                continue
            else:
                closedFingers.append(False)
            finger[0].setPosition(min(pos+inc, maxPos))
            finger[2].setPosition(max(-pos-inc, minPosTip))
            
        if np.all(closedFingers):
            return -1    
            
    def open(self):
        for f in self.fingers:
            # f[0].setPosition(min(f[0].getPosition()+inc,f[0].getMaxPosition))
            f[0].setPosition(f[0].getMinPosition())
            f[2].setPosition(f[2].getMaxPosition())
            
    def enable(self,*args):
        self.enableForceFeedback(*args)        
    
    def enableForceFeedback(self,*args):
        for f in self.fingers:
            for j in f:
                j.enableForceFeedback()
                j.getPositionSensor().enable(self.timestep)
            
    def printForces(self):
        print(f'forces:')
        for i,f in enumerate(self.fingers):
            print(f'  Finger {i+1} FF:  {f[0].getForceFeedback()}')  
            

class RobotArm():
    HAND_LENGTH = 0.38 ## To Do: This value needs to be checked. Might need to be .03 larger
    SAFE_HEIGHT = 0.3

    
    def __init__(self):
        self.supervisor = Supervisor()
        self.timestep = int(4 * self.supervisor.getBasicTimeStep())
        
        self.keyboard = self.supervisor.getKeyboard()
        self.keyboard.enable(10)
        
        self.gripper = MyGripper(self)
        self.gripper.enable(self.timestep)

        self.camera = Camera('camera')
        self.camera.enable(100)
        self.camera.recognitionEnable(self.timestep)

        self.filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            self.filename = file.name
            file.write(self.supervisor.getUrdf().encode('utf-8'))
        self.armChain = Chain.from_urdf_file(self.filename, active_links_mask=[False, True, True, True, True, True, True, False, False, False, False])

        self.motors = []
        self.motorsMax = []
        self.motorsMin = []
        for link in self.armChain.links:
            if 'motor' in link.name:
                motor = self.supervisor.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(self.timestep)
                self.motors.append(motor)
                self.motorsMax.append(motor.getMaxPosition())
                self.motorsMin.append(motor.getMinPosition())


        self.target = self.supervisor.getFromDef('TARGET')
        self.targetTranslation = self.target.getField('translation')
        
        self.arm = self.supervisor.getSelf()


    def start(self):
        # self.drawCircle()
        self.loop()
    
    @looper
    def loop(self):
        self.followSphereFromAbove()
        self.handleKeystroke()
        image2worldTest(self.supervisor)
        
        
    @looper 
    def drawCircle(self):
        # print('Loop 1: Draw a circle on the paper sheet.')
        # print('Draw a circle on the paper sheet...')
        
        t = self.supervisor.getTime()
    
        # Use the circle equation relatively to the arm base as an input of the IK algorithm.
        x = 0.25 * math.cos(t) + 1.1
        y = 0.25 * math.sin(t) - 0.95
        z = 0.05
    
        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors] + [0,0,0,0]
        print(f'len(initial_position) -> {len(initial_position)}')
        ikResults = self.armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
        # Actuate the 3 first arm motors with the IK results.
        for i in range(3):
            self.motors[i].setPosition(ikResults[i + 1])
        # Keep the hand orientation down.
        self.motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
        # Keep the hand orientation perpendicular.
        self.motors[5].setPosition(ikResults[1])
    
        # Conditions to start/stop drawing and leave this loop.
        if self.supervisor.getTime() > 2 * math.pi + 1.5:
            return -1
        elif self.supervisor.getTime() > 1.5:
            # Note: start to draw at 1.5 second to be sure the arm is well located.
            # supervisor.getFromDef('GRIPPER').close(True)
            print("TO DO: Close Gripper")
        return
    
    
    def goToSphere(self):
        # Get the absolute postion of the target and the arm base.
        targetPosition = self.target.getPosition()
        armPosition = self.arm.getPosition()
        # Compute the position of the target relatively to the arm.
        # x and y axis are inverted because the arm is not aligned with the Webots global axes.
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2]
    
        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors] + [0,0,0,0]
        ikResults = self.armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
        # Recalculate the inverse kinematics of the arm if necessary.
        position = self.armChain.forward_kinematics(ikResults)
        squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
        if math.sqrt(squared_distance) > 0.03:
            ikResults = self.armChain.inverse_kinematics([x, y, z])
    
        # Actuate the arm motors with the IK results.
        for i,m in enumerate(self.motors):
            m.setPosition(ikResults[i + 1])

        return     
                
    def followSphereFromAbove(self, safeHeight=None):
        if safeHeight is None:
            safeHeight = self.SAFE_HEIGHT
            
        # Get the absolute postion of the target and the arm base.
        targetPosition = self.target.getPosition()
        armPosition = self.arm.getPosition()
        # Compute the position of the target relatively to the arm.
        # x and y axis are inverted because the arm is not aligned with the Webots global axes.
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2]
    
        self.moveTo([x,y,z+safeHeight])
        

    def handleKeystroke(self):
        key = self.keyboard.getKey()
        
        x,y,z = self.target.getPosition()
        ballSpeed = 0.03
        
        # if (key==ord('T')):
            # goToSphere()           
            
        # print positions
        if (key==ord('F')):
            print(f'targetPosition - > {self.target.getPosition()}')
            print(f'armPosition - > {self.arm.getPosition()}')
            print(f'motorPositions:\n\t{[m.getPositionSensor().getValue() for m in self.motors]}')
        
        #gripper.printForces()
        
        if (key==ord('Q')):
            self.gripper.close()
            print('closing grip')
        if (key==ord('E')):
            self.gripper.open()
            print('opening grip')  
            
        if (key==ord('1')):
            print('Picking Up Object')
            self.pickUpObject([x,y,z])
        if (key==ord('3')):
            print('Releasing object')   
            self.deliverObject([x,y,z])     
    
    
        #reset starting position
        if (key==ord('R')):
            print(self.target)
            print(type(self.target))
            self.targetTranslation.setSFVec3f([1.44, 0, 1.77])
            
        if (key==ord('W')):
            self.targetTranslation.setSFVec3f([x-ballSpeed,y,z])
        if (key==ord('A')):
            self.targetTranslation.setSFVec3f([x,y-ballSpeed,z])
        if (key==ord('S')):
            self.targetTranslation.setSFVec3f([x+ballSpeed,y,z])
        if (key==ord('D')):
            self.targetTranslation.setSFVec3f([x,y+ballSpeed,z])
            
        if (key==ord(' ')):
            self.targetTranslation.setSFVec3f([x,y,z+ballSpeed])
        if (key==self.keyboard.SHIFT+ord(' ')):
            self.targetTranslation.setSFVec3f([x,y,z-ballSpeed])

        #trigger Camera and Img interpretation
        if (key==ord('P')):
            print("pressed: P")
            self.camera.saveImage("snapshot.jpg",100)
            #ImageDetector.openCvTest()
            ImageDetector.openCvTest()
        if (key==ord('L')):
            print("pressed: L")
            self.camera.saveImage("snapshot.jpg",100)
            #ImageDetector.callWeBotsRecognitionRoutine(self.camera)
            ImageDetector.imageAiTest()
                  
    def moveTo(self, pos):
        try:
            x0,y0,z0 = pos
            z0 = z0 + self.HAND_LENGTH
            
            a = 1.095594 # length of first arm
            b = math.sqrt(0.174998**2 + (0.340095+0.929888)**2) # effective length of second arm (corrected)
            w2_correction = math.atan(0.174998/(0.340095+0.929888))
            
            l0t = np.array([0, 0, 0.159498])
            l1t = np.array([0.178445, 0, 0.334888]) + l0t# y is uncertain if needed
                
            x = x0 #- l1_offset[0]*2
            y = y0 #- l1_offset[1]*2
            z = z0 - l1t[2]
            
            h = math.sqrt(x*x+y*y)-l1t[0]
            c = math.sqrt(h*h+z*z)
            
            w0 = math.atan(y0/x0) # base rotation (align to point to h)
        
            w1 = math.pi/2 - (math.atan(z/h) + math.acos((a*a-b*b+c*c)/(2*a*c))) # shoulder direction
            w2 = math.pi/2 - math.acos((a*a+b*b-c*c)/(2*a*b)) + w2_correction# elbow direction
            
            w3 = 0 # forearm rotation (should always point to 0 (down))
            w4 = math.pi/2-w1-w2#math.atan(h/z) + math.acos((b*b+c*c-a*a)/(2*b*c)) - math.pi/2# wrist direction
            w5 = w0 # wrist rotation
            
            if x<0:
                w0 += math.pi
            motor_angles = (np.array([w0,w1,w2,w3,w4,w5]) + math.pi) % (2*math.pi) - math.pi
            
            # print(f'moving to: {motor_angles}')
            
            self.setPosition(motor_angles)
            self.awaitPosition(motor_angles)
                
                
        except Exception as e:
            print(e)
            
    def clipAngles(self, angles):
        return np.clip(np.array(angles), self.motorsMin, self.motorsMax)
    
    def setPosition(self, angles):
        angles = self.clipAngles(angles)
        for m, a in zip(self.motors, angles):
                m.setPosition(a)
        
    # def awaitPosition(self, angles):
    #     diff = 10
    #     timeout = 30000 # in ms
        
    #     angles = self.clipAngles(angles)
    #     while diff>0.05 and timeout>0:
    #         self.supervisor.step(self.timestep)
    #         timeout-=self.timestep
            
    #         motor_values = np.array([m.getPositionSensor().getValue() for m in self.motors])
    #         diff = np.max(abs(motor_values - angles))

    @looperTimeout            
    def awaitPosition(self, angles):
        angles = self.clipAngles(angles)
        motor_values = np.array([m.getPositionSensor().getValue() for m in self.motors])
        diff = np.max(abs(motor_values - angles))
        if diff < 0.05:
            return -1
        
    
    def pickUpObject(self, pos,safeHeight=None):
        if safeHeight is None:
            safeHeight = self.SAFE_HEIGHT
        
        pos = np.array(pos)
        posSafe = pos + np.array([0,0,safeHeight])
        
        self.moveTo(posSafe)
        self.gripper.open()
        self.moveTo(pos)
        # self.sleep(500)
        self.gripper.close()
        # self.sleep(500)
        self.moveTo(posSafe)
        
    def deliverObject(self, pos, method='place', safeHeight=None):   
        if safeHeight is None:
            safeHeight = self.SAFE_HEIGHT
            
        pos = np.array(pos)
        posSafe = pos + np.array([0,0,safeHeight])
        
        self.moveTo(posSafe)
        
        if method == 'drop':
            self.gripper.open()
            return
        
        self.moveTo(pos)
        self.gripper.open()
        self.moveTo(posSafe)
        
    def pickNplace(self,position, destination, place_method=None):
        self.pickUpObject(position)
        self.deliverObject(destination, method=place_method)
        
            
    def sleep(self, ms):
        while ms>0:
            self.supervisor.step(self.timestep)
            ms -= self.timestep
        
        
        
        

        
def image2worldTest(supervisor):
    mover = supervisor.getFromDef('Mover').getField('translation')
    imageRef = supervisor.getFromDef('MoverReference')
    MainTable = supervisor.getFromDef('MainTable')
    
    
    follower = supervisor.getFromDef('Follower').getField('translation')
    
    #print(f'mover -> {mover}')
    #print(f'mover.getSFVec3f() -> {mover.getSFVec3f()}')
    
    res = image2world(mover.getSFVec3f(),MainTable.getPosition(), rotation=MainTable.getField('rotation').getSFVec3f(),tableSize=MainTable.getField('size').getSFVec3f())
    
    xn,yn,zn = res
    follower.setSFVec3f([xn,yn,zn])
    # follower.setSFVec3f(np.array([0,0,0]))
    
        
def image2world(pos, tableOrigin, tableSize=None, rotation=None):
    ''' this function tranforms the coordinates from the table to world coordinates.
    if no tablesize is given, pos is assumed to be in absolute values. otherwise its a value relative to the table size, from -1 to +1'''
    
    
    
    Sx, Sy, Sz = -1,1,-1
    tx,ty,tz = tableOrigin
    tx+=tableSize[0]/2
    ty+=tableSize[1]/2
        
    if tableSize is not None:
        tz=tz+tableSize[2]
        # Sx, Sy, Sz = tableSize[0]/2, tableSize[1]/2, 1
    
    if rotation is None:
        rotation[0,0,1,0]
    a = rotation[2]*rotation[3]
    
    # tMat = [
    #     [Sx*math.cos(a), -Sy*math.sin(a), 0,    tx],
    #     [Sx*math.sin(a),  Sy*math.cos(a), 0,    ty],
    #     [0,               0,              Sz,   tz],
    #     [0,               0,              0,    1]
    # ]
    tMat = [
        [0, -Sy, 0,    tx],
        [Sx,  0, 0,    ty],
        [0,               0,              Sz,   tz],
        [0,               0,              0,    1]
    ]
    
    if len(pos)==3:
        pos = np.array([*pos,1])
    
    res = np.matmul(tMat,pos)[:3]
    #print(f'pos: {pos}')
    #print(f'result: {res}')
    return res
    
        
        
robot = RobotArm()
robot.start()




