"""follow_line controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

MAX_SPEED = 6.2
# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Setup, mapping sensors, motors, etc.
leftMotor = robot.getDevice('wheel_left_joint')
rightMotor = robot.getDevice('wheel_right_joint')

lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

map = np.zeros((300,460))
marker = robot.getFromDef('marker').getField('translation')

WP = [(0.55, -0.4), #start
      (0.55, -2.51), #down the right
      (0.18, -2.94), #turning
      (-0.65, -3.09),
      (-1.62, -2.76), #up the left
      (-1.69, -2.13),
      (-1.69, -0.4),
      (-1.31, 0.03),
      (-1.03, 0.28),#turn around
      (-1.51, 0.41),#turn around
      (-1.58, 0.05),
      (-1.69, -0.25),
      (-1.69, -0.9),
      (-1.69, -2),
      (-1.72, -2.82),
      (-1.25, -3.04),
      (-0.7, -3),
      (0.27, -2.73),
      (0.64, -2.24),
      (0.64, -0.28),
      (0.2, 0),] 
index = 0
marker.setSFVec3f([*WP[index], index])



odom_angle = 1.5708

xw = 0
yw = 0
timestep_count = 0

delta_t = 32/1000
end = False
# generates angles from 120 to -120 to associate with lidar data
angles = np.linspace(2.094, -2.094, 667)

# transform from world to map coordinates
def world2map(xw, yw):
    return max(min(int((xw + 2.15)/3.6 * 300), 299), 0) , max(min(int((1.7 - yw)/5.55 * 460), 459), 0)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    while end:
        # loop so you can see my c-space :D
        kernel= np.ones((55,55))
        result = (map > 0.9) * 1
        cmap = signal.convolve2d(result,kernel,mode='same')
        cspace = cmap>0.99
        
        
        plt.imshow(cspace)
        plt.show()
        
    # get robot position and orientation
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]

    odom_angle =np.arctan2(compass.getValues()[0],compass.getValues()[1])
    
    # display where we are in red
    px, py = world2map(xw,yw)
    display.setColor(0xFF0000)
    display.drawPixel(px,py)
    
    
    # Euclidean distance to WP error
    rho = np.sqrt((xw - WP[index][0])**2 + (yw - WP[index][1])**2)
    alpha = np.arctan2(WP[index][1] - yw,WP[index][0] - xw) - odom_angle
    
    # limit angle between +- pi
    if alpha > np.pi:
        alpha = alpha - 2*np.pi
   
    # Check if close to next waypoint and if all waypoints are done
    if rho < 0.3:
        index = (index + 1) 
        if index == len(WP):
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            end = True
            continue
        marker.setSFVec3f([*WP[index], 0.0669])
       
   
    
    # get data from lidar
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 100
    
  
    
    
    # Transformation Matrix = Rotation + translation!!!!
    w_T_r = np.array([[np.cos(odom_angle), -np.sin(odom_angle),  xw + 0.2 * np.cos(odom_angle)],
                      [np.sin(odom_angle),  np.cos(odom_angle),  yw + 0.2 * np.sin(odom_angle)],
                      [                 0,                   0,   1]])
    
   
    # Multiply distance with corresponding angle
    X_i = np.array([ranges * np.cos(angles), 
                    ranges * np.sin(angles), 
                    np.ones(667)])
    D = w_T_r @ X_i
    
    # for each pixel, increase confidence at location
    for i in range(667):
        if i < 160 or i > 667 - 160:
            continue
        
        px, py = world2map(D[0, i],D[1, i])

        map[px, py] = min(map[px, py] + 0.02, 1)
        
       
        v=int(map[px,py]*255)
        color=(v*256**2+v*256+v)
        display.setColor(color)
        display.drawPixel(px, py)
      

    # set speed based on error from WP
    p1, p2 = 10 , 15
    leftSpeed = - alpha*p1 + rho*p2
    rightSpeed = alpha*p1 + rho*p2
    
    leftSpeed = max(min(leftSpeed, 6.28), -6.28)
    rightSpeed = max(min(rightSpeed, 6.28), -6.28)
    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    pass

