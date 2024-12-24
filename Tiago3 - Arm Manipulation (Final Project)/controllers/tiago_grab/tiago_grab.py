from controller import Robot, Supervisor
import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel, Selector
from allign import Allign
from scan import Scan
from moveToGrab import MoveToGrab
from grab import Grab
from stop import Stop
from backup import BackUp
from planning import Planning
from navagation import Navagation
from drop import Drop
from storeitem import StoreItem

robot = Robot()

timestep = 8

robot_joints = [
       'torso_lift_joint',
       'arm_1_joint', # Shoulder Left Right
       'arm_2_joint', # Shoulder up down
       'arm_3_joint', # Elbow left right rotation
       'arm_4_joint',
       'arm_5_joint',
       'arm_6_joint', # Wrist joint
       'arm_7_joint', # Hand rotation
       'head_1_joint',
       'head_2_joint',
       'gripper_left_finger_joint',
       'gripper_right_finger_joint']

# Setting up motors to be quick accesed
robot_motors = {}
for joint in robot_joints:
    robot_motors[joint] = robot.getDevice(joint)

# Force sensing enabled for grippers
robot_motors['gripper_left_finger_joint'].enableForceFeedback(timestep)
robot_motors['gripper_right_finger_joint'].enableForceFeedback(timestep)

encoders = {}

# Setting up encoders for quick access
for i, joint in enumerate(robot_joints):
    if joint == 'gripper_left_finger_joint':
        encoders['gripper_left_finger_joint'] = robot.getDevice('gripper_left_sensor_finger_joint')
        encoders['gripper_left_finger_joint'].enable(timestep)
        
    elif joint == 'gripper_right_finger_joint':
        encoders['gripper_right_finger_joint'] = robot.getDevice('gripper_right_sensor_finger_joint')
        encoders['gripper_right_finger_joint'].enable(timestep)

    else:
        encoders[joint] = robot.getDevice(joint +'_sensor')
        encoders[joint].enable(timestep)

# Preset arm positions so I can just reference them when needing to move arm
JC = { 'openGripper' : {'gripper_left_finger_joint' : 0.045,
                        'gripper_right_finger_joint': 0.045},
       'closeGripper': {'gripper_left_finger_joint' : 0.0,
                        'gripper_right_finger_joint': 0.0},
       'prepareGrab': {'torso_lift_joint' : 0.33,
                       'arm_1_joint' : 1.57079, 
                       'arm_2_joint' : 0, 
                       'arm_3_joint' : 0,
                       'arm_4_joint' : 0,
                       'arm_5_joint' : 0,
                       'arm_6_joint' : 0, 
                       'arm_7_joint' : 1.57079, 
                       'gripper_left_finger_joint' : 0.045,
                       'gripper_right_finger_joint': 0.045,
                       'head_1_joint':0,
                       'head_2_joint':0},
       'storeItem': {'torso_lift_joint' : 0.33,
                       'arm_1_joint' : 0.5, # Shoulder Left Right
                       'arm_2_joint' : 1, # Shoulder up down
                       'arm_3_joint' : 0, # Elbow left right rotation
                       'arm_4_joint' : -0.32,
                       'arm_5_joint' : 0,
                       'arm_6_joint' : 0, # Wrist joint
                       'arm_7_joint' : 1.57079, # Hand rotation
                       'head_1_joint':0,
                       'head_2_joint':0},
        'dropItem': {'torso_lift_joint' : 0.33,
                       'arm_1_joint' : 0.08, 
                       'arm_2_joint' : 0, 
                       'arm_3_joint' : 0,
                       'arm_4_joint' : 0,
                       'arm_5_joint' : 0,
                       'arm_6_joint' : 0, 
                       'arm_7_joint' : 1.57079, 
                       'head_1_joint':0,
                       'head_2_joint':0}
                       }
                       
# Function to change arm position by just passing in a name
def changeArmPos(name):
    for joint in JC[name]:
        robot_motors[joint].setPosition(JC[name][joint])

# Blackboard definition to pass info between behaviors
class Blackboard:
    def __init__(self) -> None:
        self.data = {}
    
    def read(self, key):
        return self.data.get(key)

    def write(self, key, value):
        self.data[key] = value

# Initalize blackboard with this robot and our waypoints
blackboard = Blackboard()
blackboard.write('robot', robot)
blackboard.write('motors', robot_motors)
blackboard.write('JC', JC)
blackboard.write('encoders', encoders)



# The behavior tree
tree = Sequence("Main", children = [
    Parallel("Allign And Grab", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
        Scan('Scanning', blackboard),
        MoveToGrab("Grabbing", blackboard),
        Allign("Testing", blackboard)
        ]),
    Grab("Grabbing", blackboard),
    BackUp("backing up", blackboard),
    StoreItem("Storing Item", blackboard),
    Planning("Planning route to side of table", blackboard, (0.4, -1.7)),
    Navagation("navagating on route", blackboard),
    Drop("Dropping Item", blackboard),
    StoreItem("Storing Nothing, just for easier nav", blackboard),
    Planning("Run it back to allign", blackboard, (-1.61, 0.3)),
    Navagation("navagating on route", blackboard),
    Planning("Aliging to origin", blackboard, (-0.3, 0.2)),
    Navagation("navagating on route", blackboard),
], memory=True)

tree.setup_with_descendants()

while robot.step(timestep) != -1:
    tree.tick_once()


