# Enter here exit cleanup code.


# Copyright 1996-2022 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstration of inverse kinematics using the "ikpy" Python module."""

import sys
import tempfile
import numpy as np
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor, Robot, Camera



class MyGripper:
    def __init__(self):
        self.f1 = [supervisor.getDevice(f'finger_1_joint_{i}') for i in [1,2,3]]
        self.f2 = [supervisor.getDevice(f'finger_2_joint_{i}') for i in [1,2,3]]
        self.f3 = [supervisor.getDevice(f'finger_middle_joint_{i}') for i in [1,2,3]]
        self.fingers = [self.f1,self.f2,self.f3]
        self.enableForceFeedback()
        
    def close(self):
        # inc = 0.5
        for f in self.fingers:
            # f[0].setPosition(min(f[0].getPosition()+inc,f[0].getMaxPosition))
            f[0].setPosition(f[0].getMaxPosition())
            
    def open(self):
        for f in self.fingers:
            # f[0].setPosition(min(f[0].getPosition()+inc,f[0].getMaxPosition))
            f[0].setPosition(f[0].getMinPosition())
            
    def enableForceFeedback(self):
        for f in self.fingers:
            for j in f:
                j.enableForceFeedback()
            
    def printForces(self):
        print(f'forces:')
        for i,f in enumerate(self.fingers):
            print(f'  Finger {i+1} FF:  {f[0].getForceFeedback()}')  
  

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')


IKPY_MAX_ITERATIONS = 4

print('Initialize the Webots Supervisor.')
supervisor = Supervisor()
keyboard = supervisor.getKeyboard()
keyboard.enable(50)
gripper = MyGripper()

timeStep = int(4 * supervisor.getBasicTimeStep())

print('Create the arm chain from the URDF')
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True, False, False, False, False])

print('Initialize the arm motors and encoders.')
motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)

print('Get the arm and target nodes.')
target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()




def drawCircle():
    print('Loop 1: Draw a circle on the paper sheet.')
    print('Draw a circle on the paper sheet...')
    
    while supervisor.step(timeStep) != -1:
        t = supervisor.getTime()
    
        # Use the circle equation relatively to the arm base as an input of the IK algorithm.
        x = 0.25 * math.cos(t) + 1.1
        y = 0.25 * math.sin(t) - 0.95
        z = 0.05
    
        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
        print(f'len(initial_position) -> {len(initial_position)}')
        ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
        # Actuate the 3 first arm motors with the IK results.
        for i in range(3):
            motors[i].setPosition(ikResults[i + 1])
        # Keep the hand orientation down.
        motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
        # Keep the hand orientation perpendicular.
        motors[5].setPosition(ikResults[1])
    
        # Conditions to start/stop drawing and leave this loop.
        if supervisor.getTime() > 2 * math.pi + 1.5:
            break
        elif supervisor.getTime() > 1.5:
            # Note: start to draw at 1.5 second to be sure the arm is well located.
            # supervisor.getFromDef('GRIPPER').close(True)
            print("TO DO: Close Gripper")
    
def goToSphere():
    print("Loop 2: Move the arm hand to the target.")
    print('Move the yellow and black sphere to move the arm...')
    
    while supervisor.step(timeStep) != -1:
        # Get the absolute postion of the target and the arm base.
        targetPosition = target.getPosition()
        armPosition = arm.getPosition()
        # Compute the position of the target relatively to the arm.
        # x and y axis are inverted because the arm is not aligned with the Webots global axes.
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2]
    
        # Call "ikpy" to compute the inverse kinematics of the arm.
        initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
        ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
        # Recalculate the inverse kinematics of the arm if necessary.
        position = armChain.forward_kinematics(ikResults)
        squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
        if math.sqrt(squared_distance) > 0.03:
            ikResults = armChain.inverse_kinematics([x, y, z])
    
        # Actuate the arm motors with the IK results.
        for i in range(len(motors)):
            motors[i].setPosition(ikResults[i + 1])
            
        return
        #gripper.printForces()
            
        key = keyboard.getKey()
        if (key==ord('A')):
            gripper.close()
            print('closing grip')
        if (key==ord('S')):
            gripper.open()
            print('opening grip')       
            
def followSphereFromAbove():
    print("Loop 2: Move the arm hand to the target.")
    print('Move the yellow and black sphere to move the arm...')
    
    targetTrans = target.getField("translation")
    ballSpeed = 0.05
    
    while supervisor.step(timeStep) != -1:
        # Get the absolute postion of the target and the arm base.
        targetPosition = target.getPosition()
        armPosition = arm.getPosition()
        # Compute the position of the target relatively to the arm.
        # x and y axis are inverted because the arm is not aligned with the Webots global axes.
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2]
    
    
        
        try:
            motor_angles = moveTo([x,y,z])
        except Exception as e:
            print(e)
            continue
        # Actuate the arm motors with the IK results.
        for i in range(len(motors)):
            motors[i].setPosition(motor_angles[i])
                
        key = keyboard.getKey()
        
        # if (key==ord('T')):
            # goToSphere()           
                
                
        # print positions
        if (key==ord('F')):
            print(f'targetPosition - > {targetPosition}')
            print(f'armPosition - > {armPosition}')
            print(f'motorPositions:\n\t{[m.getPositionSensor().getValue() for m in motors]}')
        
        #gripper.printForces()
         
        if (key==ord('Q')):
            gripper.close()
            print('closing grip')
        if (key==ord('E')):
            gripper.open()
            print('opening grip')        
    
    
        #reset starting position
        if (key==ord('R')):
            print(target)
            print(type(target))
            targetTrans.setSFVec3f([1.44, 0, 1.77])
            
        if (key==ord('W')):
            targetTrans.setSFVec3f([x-ballSpeed,y,z])
        if (key==ord('A')):
            targetTrans.setSFVec3f([x,y-ballSpeed,z])
        if (key==ord('S')):
            targetTrans.setSFVec3f([x+ballSpeed,y,z])
        if (key==ord('D')):
            targetTrans.setSFVec3f([x,y+ballSpeed,z])
            
        if (key==ord(' ')):
            targetTrans.setSFVec3f([x,y,z+ballSpeed])
        if (key==keyboard.SHIFT+ord(' ')):
            targetTrans.setSFVec3f([x,y,z-ballSpeed])
            
            
            
def moveTo(pos):
    x0,y0,z0 = pos
    
    
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
    
    print(f'moving to: {motor_angles}')
    
    return motor_angles


botcam = Camera('camera')
print(f'botcam -> {botcam}')
botcam.enable(timeStep)

followSphereFromAbove()

