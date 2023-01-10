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
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor
from controller import Robot


if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')


IKPY_MAX_ITERATIONS = 4

print('Initialize the Webots Supervisor.')
supervisor = Supervisor()
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






def setGripper(distance):
    pass    


print('Loop 1: Draw a circle on the paper sheet.')
print('Draw a circle on the paper sheet...')

DrawCircle = 0

while supervisor.step(timeStep) != -1 and DrawCircle:
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

# Loop 2: Move the arm hand to the target.
print('Move the yellow and black sphere to move the arm...')
counter=0
inc = 0.05
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
        
        
    fingers = ['finger_1_joint_1', 'finger_2_joint_1', 'finger_middle_joint_1', 'palm_finger_1_joint', 'palm_finger_2_joint']
    
    fingersJ1 = [supervisor.getDevice(f'finger_{i}_joint_1') for i in [1,2,'middle']]
    fingersJ2 = [supervisor.getDevice(f'finger_{i}_joint_2') for i in [1,2,'middle']]
    fingersJ3 = [supervisor.getDevice(f'finger_{i}_joint_3') for i in [1,2,'middle']]
        
    for j in [fingersJ1,fingersJ2, fingersJ3]:
        for f in j:
            f.enableForceFeedback()
        
    print(f'forces:')
        
    for i,f in enumerate(fingersJ1):
        fpos = abs((counter % f.getMaxPosition()*2) - f.getMaxPosition())
        f.setPosition(max(fpos,f.getMinPosition()))
        print(f'  Finger {i} FF:  {f.getForceFeedback()}')        
        
    counter += inc
    



