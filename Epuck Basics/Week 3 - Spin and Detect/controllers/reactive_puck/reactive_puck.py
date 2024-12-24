from controller import Robot, DistanceSensor, Motor
from enum import Enum

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 3

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Defined states
class State(Enum):
    MOVE = 1
    TURN = 2
    STOP = 3
    TURN_TO_RIGHT = 4
    MOVE_ALONG_WALL = 5

state = State.MOVE
print('i move')

def move():
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)

def turn():
    leftMotor.setVelocity(MAX_SPEED/3)
    rightMotor.setVelocity(-MAX_SPEED/3)
    
def stop():
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

already_turned = False
# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    # detect obstacles
    object_in_front = psValues[7] >= 120 and psValues[0] >= 120
    
    match state:
        case State.MOVE:
            move()
            if object_in_front:
                if already_turned:
                    state = State.TURN_TO_RIGHT
                    print('i turn 90')
                    continue
                print('i turn 180')
                state = State.TURN
                already_turned = True
            continue
        
        case State.TURN:
            object_in_back = (psValues[4] >= 120 and psValues[3] >= 120) 
            similar_dis = abs(psValues[4] - psValues[3]) <= 4
            
            if(object_in_back and similar_dis):
                print('i move again')
                state = State.MOVE
            turn()
            continue
            
        case State.TURN_TO_RIGHT:
            if(psValues[5] >= 158.73):
                print('i move along wall')
                state = State.MOVE_ALONG_WALL
            turn()
            continue
        
        case State.MOVE_ALONG_WALL:
            if(psValues[5] <= 70):
                print('i die, goodbye')
                state = State.STOP
                continue
            move()
            continue
            
        case State.STOP:
            stop()
            break
                
    
    
    

    