
from __future__ import annotations
import typing
from typing import Optional, Iterable, Any, Literal
from collections.abc import Sequence

import math
import numbers
import os
import sys
import tempfile
import time
from warnings import warn

import numpy as np
import yaml
import controller
from controller import Camera, Motor, Robot, Supervisor, Node, PositionSensor, Keyboard
from controller.device import Device
from controller.wb import wb
from utils import logger, looper, looperTimeout


# import TrainingsHelper
# import ImageDetector
from ImageDetection import ImageDetector, TrainingsHelper

Vec3 = tuple[float, float, float]

DISABLE_FFB = False
DISABLE_FINGER_TIP = False
AUTO_LOOP = False
TABLE_TRANSFORM_TEST = True

def _enableFB(self, sampling_period: int):
    '''Partial fix for Webots bug in R2023a to allow force feedback sampling period to be set'''
    wb.wb_motor_enable_force_feedback(self._tag, sampling_period)
    
Motor.enableForceFeedback = _enableFB

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

class MyGripper(logger):
    SPEED = 5 # SPEED in DEGREES
    GRIP_FORCE = 2
    
    def __init__(self,master: RobotArm, logging: str = 'D', logName: str = 'Gripper') -> None:
        super().__init__(logging=logging, logName=logName)
        
        self.master = master
        self.supervisor = master.supervisor
        self.timestep = master.timestep
        
        
        self.f1: list[Motor] = []
        self.f2: list[Motor] = []
        self.f3: list[Motor] = []
        
        self.f1psensor: list[PositionSensor] = []
        self.f2psensor: list[PositionSensor] = []
        self.f3psensor: list[PositionSensor] = []
        
        for i in [1,2,3]:
            m1 = self.supervisor.getDevice(f'finger_1_joint_{i}')
            m2 = self.supervisor.getDevice(f'finger_2_joint_{i}')
            m3 = self.supervisor.getDevice(f'finger_middle_joint_{i}')
            if isinstance(m1, Motor) and isinstance(m2, Motor) and isinstance(m3, Motor):
                self.f1.append(m1)
                self.f2.append(m2)
                self.f3.append(m3)
            else:
                raise TypeError(f'Gripper motors not found')
            
            s1 = m1.getPositionSensor()
            s2 = m2.getPositionSensor()
            s3 = m3.getPositionSensor()
            if isinstance(s1, PositionSensor) and isinstance(s2, PositionSensor) and isinstance(s3, PositionSensor):
                self.f1psensor.append(s1)
                self.f2psensor.append(s2)
                self.f3psensor.append(s3)
            else:
                raise TypeError(f'Gripper position sensors not found')
        #[master.supervisor.getDevice(f'finger_1_joint_{i}') for i in [1,2,3]]
        #[master.supervisor.getDevice(f'finger_2_joint_{i}') for i in [1,2,3]]
        #[master.supervisor.getDevice(f'finger_middle_joint_{i}') for i in [1,2,3]]
        
        self.fingers: list[list[Motor]] = [self.f1,self.f2,self.f3]
        self.positionSensors: list[list[PositionSensor]] = [self.f1psensor,self.f2psensor,self.f3psensor]
        # self.enableForceFeedback()
                
        
    @looperTimeout
    def close(self) -> int|None:
        '''
        Close the grippers fingers until their force is above GRIP_FORCE or finger completely closed. 
        Await movement. 
        Timeout after 10 seconds.
        '''
        inc = self.SPEED * np.pi/180 #* self.timestep
        forces = np.array([f[0].getForceFeedback() for f in self.fingers])
        maxPositions = np.array([f[0].getMaxPosition() for f in self.fingers])
        minPositionsTip = np.array([f[2].getMinPosition() for f in self.fingers])
        
        positions = np.array([ps[0].getValue() for ps in self.positionSensors])
        # currentPositionsTip = np.array([ps[2].getValue() for ps in self.positionSensors])
        closedFingers = []
                
        self.logVV(f'speed -> {inc/self.timestep}')
        self.logVV(f'Forces:  {"   ".join([f"{f:>7.2f}" for f in forces])}')
        
        for finger, force, maxPos, minPosTip, pos in zip(self.fingers,forces,maxPositions, minPositionsTip,positions): #
            if (not DISABLE_FFB and force>self.GRIP_FORCE) or abs(pos-maxPos)<0.05  :
                closedFingers.append(True)
                continue
            else:
                closedFingers.append(False)
            finger[0].setPosition(min(pos+inc, maxPos))
            if not DISABLE_FINGER_TIP:
                finger[2].setPosition(max(-pos-inc, minPosTip))
            
        if np.all(closedFingers):
            return -1    
            
    @looperTimeout
    def open(self) -> int|None:
        '''
        Open the grippers fingers. 
        Await movement until the force is below GRIP_FORCE. 
        Timeout after 10 seconds
        '''
        maxForce = 0
        for f in self.fingers:
            # f[0].setPosition(min(f[0].getPosition()+inc,f[0].getMaxPosition))
            f[0].setPosition(f[0].getMinPosition())
            if not DISABLE_FINGER_TIP:
                f[2].setPosition(f[2].getMaxPosition())
            maxForce = max(maxForce,f[0].getForceFeedback(),f[1].getForceFeedback())
        if maxForce<self.GRIP_FORCE:
            return -1
            
    def enable(self,*args) -> None:
        '''Short for enableForceFeedback'''
        self.enableForceFeedback(*args)        
    
    def enableForceFeedback(self,*args) -> None:
        '''
        Enable ForceFeedback for all fingers and joints
            Input:
                args:   timestep (optional) Default is self.timestep
        '''
        timestep = args[0] if args else self.timestep
            
        for i, f in enumerate(self.fingers):
            for ii,j in enumerate(f):
                self.logD(f'enabling ForceFB: Finger {i}, {ii}')
                try:
                    j.enableForceFeedback(timestep)
                except TypeError as te:
                    # warn(te, category=None, stacklevel=1)
                    self.logE('SamplingPeriod of ForceFeedback could not be set. Please update your Webots installation.', category=None, stacklevel=1)
                    raise te
                    self.logE(te, category=None, stacklevel=1)
                    #j.enableForceFeedback()
                    
                self.logD(j.getForceFeedbackSamplingPeriod())
                self.positionSensors[i][ii].enable(timestep)
                # j.getPositionSensor().enable(timestep)
            
    def printForces(self) -> None:
        '''Print the current force feedback of all fingers and joints'''
        printout = 'forces:\n'
        printout += '\n'.join([f'  Finger {i+1} FF:  {f[0].getForceFeedback()}' for i,f in enumerate(self.fingers)])
        self.log(printout)
        # for i,f in enumerate(self.fingers):
        #     self.log(f'  Finger {i+1} FF:  {f[0].getForceFeedback()}')  
            

class RobotArm(logger):
    HAND_LENGTH = 0.365#75 ## To Do: This value needs to be checked. Might need to be .03 larger
    SAFE_HEIGHT = 0.3
    HOME_POSITION = [0.7, 0, 1.1]
    
    def __init__(self,logging: str = 'D', logName: str = 'RobotArm') -> None:
        super().__init__(logging=logging, logName=logName)
        
        self.supervisor = Supervisor()
        self.timestep = int(4 * self.supervisor.getBasicTimeStep())
        
        self.master = self
        
        self.keyboard = self.supervisor.getKeyboard()
        self.keyboard.enable(10)
        
        self.gripper = MyGripper(self, logging=self.logging)
        self.gripper.enable(self.timestep)

        self.camera = Camera('camera')
        self.camera.enable(100)
        self.camera.recognitionEnable(self.timestep)
        
        self.dataCam = Camera('dataCam')
        self.dataCam.enable(int(1000/24))
        self.dataCam.recognitionEnable(self.timestep)
        self.viewPoint = self.supervisor.getFromDef('Viewpoint')
        self.collectData = False
        self.lastDCamPos = np.zeros(3)
        self.lastDCamOri = np.zeros(4)
        
        
        self.mainTable = Table(self.supervisor.getFromDef('MainTable'))
        self.loopCount = 0
        self.dataCount = 0

        # Get ArmChain Data
        self.filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            self.filename = file.name
            file.write(self.supervisor.getUrdf().encode('utf-8'))
        self.armChain = Chain.from_urdf_file(self.filename, active_links_mask=[False, True, True, True, True, True, True, False, False, False, False])

        # Get Motors and their range
        self.motors: list[Motor]= []
        self.motorsMax: list[float] = []
        self.motorsMin: list[float] = []
        self.positionSensors: list[PositionSensor] = []
        
        for link in self.armChain.links:
            if 'motor' in link.name:
                motor: Motor|Device = self.supervisor.getDevice(link.name)
                assert isinstance(motor, Motor)
                motor.setVelocity(1.0)
                
                position_sensor: controller.PositionSensor|None = motor.getPositionSensor()
                assert position_sensor is not None
                position_sensor.enable(self.timestep)
                
                self.motors.append(motor)
                self.motorsMax.append(motor.getMaxPosition())
                self.motorsMin.append(motor.getMinPosition())
                self.positionSensors.append(position_sensor)

        # Get Target
        self.target = self.supervisor.getFromDef('TARGET')
        self.targetTranslation = self.target.getField('translation')
        
        # Get Self
        self.arm = self.supervisor.getSelf()
        
        # Get Image Scanner
        self.imageScanner = ImageDetector.ImageScanner(self, model='imageai',logging=logging)
        self.foundObjects = []
        self.stopOrganization = False
        
        # Object Info
        with open(os.path.join(os.path.dirname(__file__),'data','Objects.yaml'),'r') as f:
            self.objectInfo = yaml.load(f, Loader=yaml.loader.SafeLoader)
        self.logVV('Known Object Info: \n',self.objectInfo)
        self.PhotoshootIndex=0
        self.lastScan=0
        
        self.testMover = self.supervisor.getFromDef('Mover')
        self.testMoverReference = self.supervisor.getFromDef('MoverReference')
        self.testFollower = self.supervisor.getFromDef('Follower')
        if not TABLE_TRANSFORM_TEST:
            self.testMover.getField('translation').setSFVec3f([0,0,0])
            self.testMoverReference.getField('translation').setSFVec3f([0,0,0])
            self.testFollower.getField('translation').setSFVec3f([0,0,0])

            

    def sleep(self, _time: float = 1) -> None:
        time = _time
        while self.supervisor.step(self.timestep) != -1:
            # self.master.stepOperations()
            time = time - self.timestep
            if time<0:
                return

    def stepOperations(self) -> None:
        '''
        Executes the operations that are scheduled for each simulation step.
        Keystrokes are handled.
        Datacam follows the viewpoint.
        If self.collectData is True, a snapshot is taken if the viewpoint has moved.
        '''
        self.handleKeystroke()
        vpPos = self.viewPoint.getField('position').getSFVec3f()
        vpOri = self.viewPoint.getField('orientation').getSFRotation()
        
        self.arm.getField('dataCamTrans').setSFVec3f(vpPos)
        self.arm.getField('dataCamRot').setSFRotation(vpOri)
        
        diffP = self.lastDCamPos - np.array(vpPos)
        diffO = self.lastDCamOri - np.array(vpOri)
        
        self.lastDCamPos = np.array(vpPos)
        self.lastDCamOri = np.array(vpOri)
        
        if TABLE_TRANSFORM_TEST:
            self.testFollower.getField('translation').setSFVec3f(list(self.mainTable.local2world(self.testMover.getField('translation').getSFVec3f())))
        if ((max(diffP)>0.1) or (max(diffO)>0.17)) and self.collectData:
            self.logV('Taking Snapshot')
            TrainingsHelper.makeSnapshot(self.dataCam,type='train')
        
        return
    
    def start(self) -> None:
        '''Robot's entry point and setup routine.'''
        try:
            # self.drawCircle()
            self.moveTo(self.HOME_POSITION)
            if AUTO_LOOP:
                self.autoloop()
            else:
                self.loop()
        except Exception as e:
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
            self.logE(e)
            raise e
    
    @looper
    def loop(self) -> None:
        '''Main loop of the robots controller. Manual mode'''
        # self.goToSphere()
        self.followSphereFromAbove()
        # self.handleKeystroke()
        # image2worldTest(self)
    
    @looper
    def autoloop(self) -> None:
        '''Main loop of the robots controller. Autonomous mode'''
        
        # self.handleKeystroke()
        self.moveTo(self.HOME_POSITION)
        objects = self.imageScanner.scanImage()
        self.organizeObjects(objects)
        
    def singleItemPhotoshoot(self, next: bool = True) -> None:
        if next: 
            self.PhotoshootIndex= (self.PhotoshootIndex+1)%8
        TrainingsHelper.swapObj(self.PhotoshootIndex, self.mainTable, self.supervisor)
        
    def organizeObjects(self, objects: Iterable[dict]) -> None:
        '''
        Organizes the objects in the table
            Inputs:
                objects: List of objects to be organized
        '''
        for obj in objects:
            if self.stopOrganization:
                self.stopOrganization=False
                return
            destination = self.mainTable.local2world((-0.1,0.9,0))
            voffset = 0
            
            if obj['name'] in self.objectInfo.keys():
                voffset = self.objectInfo[obj['name']]['voffset'] 
                destination = self.objectInfo[obj['name']]['destination']
                
            destination = tuple(np.array(destination)+np.array((0,0,voffset)))
            # destination[2] = destination[2] + voffset
            worldpos = self.mainTable.local2world(obj['position']+[voffset])
            self.logV(f"{obj['name']} Pos in Table: {obj['position']}")
            self.logV(f"{obj['name']} Pos in World: {worldpos}")
            
            self.pickNplace(worldpos, destination , rotation=-obj['orientation'])#-np.pi/2
    
    @looper
    def randomPosSamplingLoop(self,sampleSize,type):
        if self.loopCount % 10 == 0:
            if self.loopCount % 20 == 0:
                TrainingsHelper.moveTableNodes(self.supervisor,self.mainTable)
            else:
                TrainingsHelper.makeSnapshot(self.camera,type)
                self.dataCount +=1
        self.loopCount += 1
        if self.dataCount>sampleSize:
            return -1

    @looper
    def singleObjectImageLoop(self,imagesPerPerspective,type):
        if self.loopCount % 10 == 0:
            if self.loopCount % 20 == 0:
                TrainingsHelper.single_objectImage_setup(self.supervisor,self.mainTable,imagesPerPerspective)
            else:
                TrainingsHelper.makeSnapshot(self.dataCam,type)
                self.dataCount +=1
        self.loopCount += 1
        if self.dataCount>imagesPerPerspective*32: # amountPerspectives*amountObjects = 32 
            return -1

        
        
    @looper 
    def drawCircle(self) -> int|None:
        ''' Draws a circle. Unused. Useless. Rests from tutorial'''
        # print('Loop 1: Draw a circle on the paper sheet.')
        # print('Draw a circle on the paper sheet...')
        
        t = self.supervisor.getTime()
    
        # Use the circle equation relatively to the arm base as an input of the IK algorithm.
        x = 0.25 * math.cos(t) + 1.1
        y = 0.25 * math.sin(t) - 0.95
        z = 0.05
    
        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0] + [sensor.getValue() for sensor in self.positionSensors] + [0,0,0,0]
        self.logV(f'len(initial_position) -> {len(initial_position)}')
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
            self.logV("TO DO: Close Gripper")
        return
    
    
    def goToSphere(self) -> None:
        '''
        Uses IK to move gripper to targets position. Orientation/direction is not considered.
        '''
        # Get the absolute postion of the target and the arm base.
        targetPosition = self.target.getPosition()
        armPosition = self.arm.getPosition()
        # Compute the position of the target relatively to the arm.
        # x and y axis are inverted because the arm is not aligned with the Webots global axes.
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2]
    
        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0] + [sensor.getValue() for sensor in self.positionSensors] + [0,0,0,0]
        ikResults = self.armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
        # Recalculate the inverse kinematics of the arm if necessary.
        position = np.array(self.armChain.forward_kinematics(ikResults))
        squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
        if math.sqrt(squared_distance) > 0.03:
            ikResults = self.armChain.inverse_kinematics([x, y, z])
    
        # Actuate the arm motors with the IK results.
        for i,m in enumerate(self.motors):
            m.setPosition(ikResults[i + 1])

        return     
                
    def followSphereFromAbove(self, safeHeight: float|None = None) -> None:
        '''
        Moves the arm above the Spheres location.
        Uses trigonometry to calculate the angle of the arm's motors to place the wrist with the hand pointing downwards above the target.
        Input:
            safeHeight: (optional) Height above the target to move to. If None, self.SAFE_HEIGHT is used.
        '''
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
        

    def handleKeystroke(self) -> None:
        '''Routine function to handle keystrokes and hold keybindings'''
        key = self.keyboard.getKey()
        
        x,y,z = self.target.getPosition()
        ballSpeed = 0.03
                    
        # print positions
        if (key==ord('F')):
            self.log(f'targetPosition - > {self.target.getPosition()}')
            self.log(f'armPosition - > {self.arm.getPosition()}')
            self.log(f'motorPositions:\n\t{[sensor.getValue() for sensor in self.positionSensors]}')
                
        if (key==ord('Q')):
            self.gripper.close()
            self.log('closing grip')
        if (key==ord('E')):
            self.gripper.open()
            self.log('opening grip')  
            
        if (key==ord('1')):
            self.log('Picking Up Object')
            self.pickUpObject((x,y,z))
        if (key==ord('3')):
            self.log('Releasing object')   
            self.deliverObject((x,y,z))     
    
    
        #reset starting position
        if (key==ord('R')):
            self.log(self.target)
            self.log(type(self.target))
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
        if (key==int(Keyboard.SHIFT)+ord(' ')):
            self.targetTranslation.setSFVec3f([x,y,z-ballSpeed])

        #trigger Camera and Img interpretation
        if (key==ord('I')):
            self.log("pressed: I")
            self.collectData=True
        if (key==ord('O')):
            self.log("pressed: O")
            self.collectData=False
        if (key==ord('N')):
            self.log("pressed: N")
            self.singleItemPhotoshoot(next=True)
            # self.randomPosSamplingLoop(150,'train')
            
            
        if (key==ord('P')):
            self.log("pressed: P")
            #self.randomPosSamplingLoop(200,'train')
        if (key==int(Keyboard.SHIFT)+ord('P')):
            self.log("pressed:shift +  P")
            #self.randomPosSamplingLoop(50,'validation')    
        if (key==ord('L')):
            self.log("pressed: L")
            self.camera.saveImage("snapshot.jpg",100)
            TrainingsHelper.testModel()
            #ImageDetector.callWeBotsRecognitionRoutine(self.camera)
            #ImageDetector.imageAiTest()
            #TrainingsHelper.moveTableNodes(self.supervisor,self.mainTable)
        if (key==ord('K')):
            self.log("pressed: K")
            TrainingsHelper.moveTableNodes(self,self.mainTable)
            self.sleep(2)
            self.stopOrganization = True
            #self.singleObjectImageLoop(8,'train')
        if (key==int(Keyboard.SHIFT)+ord('K')):  
            self.log("pressed: shift K")
            #self.singleObjectImageLoop(2,'validation')
        if (key==ord('7')):  
            TrainingsHelper.moveViewPoint(self.supervisor,0)
        if (key==ord('8')):  
            TrainingsHelper.moveViewPoint(self.supervisor,1)
        if (key==ord('9')):  
            TrainingsHelper.moveViewPoint(self.supervisor,2)
        if (key==ord('0')):  
            TrainingsHelper.moveViewPoint(self.supervisor,3)

    def moveTo(self, pos, rotation: float|None = None):
        '''
        Input:
            pos (list/iterable of length 3):
                target position
            rotation:
                rotation angle of the gripper (after compensation of robots rotation)
        
        Moves the Gripper (fingertips) to given positions with hand pointing down.
        Movements area awaited until desired position is reached.
        '''
        if not isinstance(rotation, numbers.Number):
            rotation=0
        
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
            
            w5 = w0 + rotation# wrist rotation
            # w5 = ((w0+np.pi/2)%np.pi)-np.pi + rotation
            
            if x<0:
                w0 += math.pi
            motor_angles = (np.array([w0,w1,w2,w3,w4,w5]) + math.pi) % (2*math.pi) - math.pi
            
            # print(f'moving to: {motor_angles}')
            
            self.setPosition(motor_angles)
            self.awaitPosition(motor_angles)
                
        except Exception as e:
            self.logW(e)
            
    def clipAngles(self, angles: Iterable[float]) -> np.ndarray:
        '''
        Input: 
            angles (list/iterable) with length equal to len(self.motors).
            
        This clips the values to be inside the possible angles of each motor 
        '''
        return np.clip(np.array(angles), self.motorsMin, self.motorsMax)
    
    def setPosition(self, angles: Iterable[float]) -> None:
        '''
        Input: 
            angles (list/iterable) with length equal to len(self.motors).
        
        This clips and sets the angle of each motor directly.
        '''
        angles = self.clipAngles(angles)
        for m, a in zip(self.motors, angles):
                m.setPosition(a)

    @looperTimeout            
    def awaitPosition(self, angles: Iterable[float]) -> int|None:
        '''
        Input: 
            angles (list/iterable) with length equal to len(self.motors).
            
        Clips angles to motors range and calculates difference to actual motor angles
        Function loops until the max difference is below 0.05 rad or timeout is reached.
        '''
        angles = self.clipAngles(angles)
        motor_values = np.array([sensor.getValue() for sensor in self.positionSensors])
        diff = np.max(abs(motor_values - angles))
        if diff < 0.05:
            return -1
        
    def pickUpObject(self, _pos: Vec3, safeHeight: float|None = None, rotation: float|None = None) -> None:
        '''
        Inputs: 
            pos (List/iterable of length 3):
                Position of the objet to be picked up
            safeHeight (Optional):
                Value to override Robot's SAFE_HEIGHT
        
        Moves above the object to be picked up (SAFE_HEIGHT above object), 
        goes down, grabs the object and goes back up.
        Movements are awaited to reach desired position before starting next movement.
        '''
        if safeHeight is None:
            safeHeight = self.SAFE_HEIGHT
        
        pos = np.array(_pos)
        posSafe = pos + np.array([0,0,safeHeight])
        
        self.moveTo(posSafe, rotation=rotation)
        self.gripper.open()
        self.moveTo(pos, rotation=rotation)
        self.gripper.close()
        self.moveTo(posSafe)
        
    def deliverObject(self, _pos: Vec3, method: Literal['drop']|Any = 'place', safeHeight: float|None = None) -> None:   
        '''
        Inputs: 
            pos (List/iterable of length 3):
                Position where the object need to be delivered (destination)
            method:
                'drop' lets the object fall. anything else places the object calmly.
            safeHeight (Optional):
                Value to override Robot's SAFE_HEIGHT
        
        Moves above the objects destination (SAFE_HEIGHT above 'pos'), 
        if method=='drop':
            gripper opens
        else:
            hand goes down, releases the object and goes back up.
        Movements are awaited to reach desired position before starting next movement.
        '''
        
        if safeHeight is None:
            safeHeight = self.SAFE_HEIGHT
            
        pos = np.array(_pos)
        posSafe = pos + np.array([0,0,safeHeight])
        
        self.moveTo(posSafe)
        
        if method != 'drop':
            self.moveTo(pos)
        self.gripper.open()
        self.moveTo(posSafe)
        
    def pickNplace(self, position: Vec3, destination: Vec3, place_method: Literal['drop']|Any = None, rotation: float|None = None) -> None:
        '''
        Inputs:
            position (List/Iterable of length 3): 
                Current position of the object to be moved.
            destination (List/Iterable of length 3):
                Location where the Object is to be placed.
            place_method:
                if 'drop': gripper letts the object fall from SAFE_HEIGHT above destination.
                else: object is placed calmly
            rotation (number)(optional):
                rotation of the object to be picked up
                
        Moves an object from one position to another with the gripper.
        '''
        self.pickUpObject(position, rotation=rotation)
        self.deliverObject(destination, method=place_method)
        

        
        
        

        
        
def image2worldTest(arm: RobotArm) -> None:
    mover = arm.supervisor.getFromDef('Mover').getField('translation')
    imageRef = arm.supervisor.getFromDef('MoverReference')
    
    follower = arm.supervisor.getFromDef('Follower').getField('translation')
    
    res = arm.mainTable.local2world(tuple(mover.getSFVec3f()))
    
    xn,yn,zn = res
    follower.setSFVec3f([xn,yn,zn])
    
        
    
class Table(logger):
        
    def __init__(self, node: Node, logging: str = 'D', logName: str = 'Table') -> None:
        super().__init__(logging=logging, logName=logName)
        self.node = node
        # self.size = node.getField('size').getSFVec3f()
        # self.rotation = node.getField('rotation').getSFRotation()
        # self.position = node.getPosition()
        
        # self.orientation = node.getOrientation()
        
    
    # @position.setter
    # def position(self,a):
    #     self.logE(f'Can not set position of table to {a}. Property is read only. use Table.node.setPosition() instead.')
        
    @property
    def position(self):
        return self.node.getPosition()
    @property
    def rotation(self):
        return self.node.getField('rotation').getSFRotation()
    @property
    def size(self):
        return self.node.getField('size').getSFVec3f()
    @property
    def orientation(self):
        return self.node.getOrientation()
    
        
    def local2worldOld(self, _pos: Vec3) -> Vec3:
        ''' this function tranforms the coordinates from the table to world coordinates.
        if no tablesize is given, pos is assumed to be in absolute values. otherwise its a value relative to the table size, from 0 to +1'''
        
        # for i in range(3):
        #     pos[i]=pos[i]*self.size[i]

        Sx, Sy, Sz = 1,1,1
        tx,ty,tz = self.position
        tx+=self.size[0]/2
        ty+=self.size[1]/2
            
        if self.size is not None:
            tz=tz+self.size[2]
            # Sx, Sy, Sz = self.size[0]/2, self.size[1]/2, 1
        
        if self.rotation is None:
            self.rotation[0,0,1,0]
        a = self.rotation[2]*self.rotation[3]
        
        # tMat = [
        #     [Sx*math.cos(a), -Sy*math.sin(a), 0,    tx],
        #     [Sx*math.sin(a),  Sy*math.cos(a), 0,    ty],
        #     [0,               0,              Sz,   tz],
        #     [0,               0,              0,    1]
        # ]
        tMat = [
            [0,  -self.size[0],   0,    tx],
            [-self.size[1],  0,    0,    ty],
            [0,   0,    1,   tz],
            [0,   0,    0,    1]
        ]
        
        if len(_pos)==3:
            pos = np.array([*_pos,1])
        else:
            pos = np.array(_pos)
        if not (np.shape(pos) == (4,)):
            self.logW(f'tMat.shape: {np.shape(tMat)}\n pos.shape: {np.shape(pos)}')
        
        res = tuple(np.matmul(tMat,pos)[:3])
        #print(f'pos: {pos}')
        #print(f'result: {res}')
        return res
            
    def local2world(self, _pos: Vec3) -> Vec3:
        ''' this function tranforms the coordinates from the table to world coordinates.
        if no tablesize is given, pos is assumed to be in absolute values. otherwise its a value relative to the table size, from 0 to +1'''
        
        # for i in range(3):
        #     pos[i]=pos[i]*self.size[i]

        TableT_unscaled=[[0,-1,0,0.5],
                        [-1,0,0,0.5],
                        [0,0,-1,1],
                        [0,0,0,1]]
        TableScaling=[[self.size[0],0,0,0],
                       [0,self.size[1],0,0],
                       [0,0,self.size[2],0],
                       [0,0,0,1]]
            
        TableT=np.matmul(TableT_unscaled,TableScaling)
        
        WorldT=np.array([[math.cos(self.rotation[3]),-math.sin(self.rotation[3]),0,self.position[0]],
                         [math.sin(self.rotation[3]),math.cos(self.rotation[3]),0,self.position[1]],
                         [0,0,1,self.position[2]],
                         [0,0,0,1]])
                
        
        TMat = np.matmul(np.matmul(WorldT, TableScaling), TableT_unscaled)
        
        r = np.matmul(TMat, np.array([*_pos,1]))[:3]
        self.logD(_pos, r)
        return r
          
        
        
robot = RobotArm(logging='Very_verbose')
robot.start()



if False:
    # 
    import os
    import sys

    os.environ['LD_LIBRARY_PATH'] = '/usr/local/webots/lib/controller'
    os.environ['PYTHONPATH'] = '/usr/local/webots/lib/controller/python'
    os.environ['WEBOTS_PROJECT'] = '/home/lu/Documents/0_MasterDocs/AMS/CubeGrabberExample/Webots/DeskOrganizer/DeskOrganizer.wbt'
    os.environ['WEBOTS_HOME'] = '/usr/local/webots/'
    module_path = '/usr/local/webots/lib/controller/python/'
    if module_path not in sys.path:
        sys.path.append(module_path)
        
    import controller


