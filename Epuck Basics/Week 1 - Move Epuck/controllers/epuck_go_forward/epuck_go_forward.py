from controller import Robot, Motor

TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

timestep = int(robot.getBasicTimeStep())
# get a handler to the motors and set target position to infinity (speed control)
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')

motor_left.setPosition(float('Inf'))
motor_right.setPosition(float('Inf'))

ls = robot.getDevice('ls0')
ls.enable(timestep)

# set up the motor speeds at 10% of the MAX_SPEED.

while robot.step(TIME_STEP) != -1:
   print(ls.getValue())
   pass