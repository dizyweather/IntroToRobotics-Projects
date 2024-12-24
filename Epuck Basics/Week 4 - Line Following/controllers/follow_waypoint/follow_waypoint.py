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
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()

display = robot.getDevice('display')

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)

leftMotor.setPosition(float('Inf'))
rightMotor.setPosition(float('Inf'))

map = np.zeros((300,300))
marker = robot.getFromDef('marker').getField('translation')

WP = [(0, 0.68), (0.44, 0.68), (0.66, 0.51), (0.35, 0.24), (0.63, 0), (0.63, -0.16), (0, -0.16)]
index = 0
marker.setSFVec3f([*WP[index], index])
plt.ion()
gs = []

for i in range(3):
    gs.append(robot.getDevice('gs' + str(i)))
    gs[-1].enable(timestep)

odom_angle = 1.5708

xw = 0
yw = 0
timestep_count = 0
radius = 0.0201
d_between_wheels = 0.052
delta_t = 32/1000

# generates angles from 180 to -180 to associate with lidar data
angles = np.linspace(3.1415, -3.1415, 360)

def world2map(xw, yw):
    return max(min(int((xw + 0.195) * 300), 299), 0) , max(min(int((0.75 - yw) * 300), 299), 0)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    
    xw = gps.getValues()[0]
    yw = gps.getValues()[1]

    odom_angle =np.arctan2(compass.getValues()[0],compass.getValues()[1])
    
    px, py = world2map(xw,yw)
    display.setColor(0xFF0000)
    display.drawPixel(px,py)
    
    
    # Euclidean distance to WP error
    rho = np.sqrt((xw - WP[index][0])**2 + (yw - WP[index][1])**2)
    alpha = np.arctan2(WP[index][1] - yw,WP[index][0] - xw) - odom_angle
    
    if alpha > np.pi:
        alpha = alpha - 2*np.pi
   
        
    if rho < 0.1:
        index = (index + 1) % len(WP) 
        marker.setSFVec3f([*WP[index], index])
    print(alpha / 3.14 * 180)
    
    g = []
    for g_sensor in gs:
        g.append(g_sensor.getValue())
        
    if (g[0] > 500 and g[1]<350 and g[2]>500): # drive straight
        phildot, phirdot = MAX_SPEED, MAX_SPEED
    elif(g[2]<550): # turn right
        phildot, phirdot = 0.25 * MAX_SPEED, -0.1*MAX_SPEED
    elif(g[0] < 550): # turn left
        phirdot, phildot = 0.25 * MAX_SPEED, -0.1*MAX_SPEED
    
    
    # delta_x = radius * (phildot + phirdot) / 2 * delta_t
    # delta_w = radius * (phirdot - phildot) / d_between_wheels * delta_t
    
    
    # odom_angle += delta_w
    
    
    # xw += np.cos(odom_angle) * delta_x
    # yw += np.sin(odom_angle) * delta_x
    
    # get data from lidar
    ranges = np.array(lidar.getRangeImage())
    ranges[ranges == np.inf] = 100
    
  
    # makes an array of angles
    # angles=np.linspace(3.1415, -3.1415,360)
    
    # associates lidar data with corresponding angle
    # fig, ax = plt.subplots(subplot_kw = {'projection': 'polar'})
    # ax.plot(angles, ranges, '.')
    
    # plt.show()
    # print("lidar angle: " + str(angles[0]))
    # print("robot orientation: " + str(odom_angle))
    
    # x,y =[], []
    # for i in range(360):
        # if(np.isfinite(ranges[i])):
            # x_i=np.cos(angles[i])*ranges[i]
            # y_i=np.sin(angles[i])*ranges[i]
            # x.append(x_i)
            # y.append(y_i)
    
    # plt.plot(x,y,'.')
    # plt.show()
    
    # Rotation Matrix
    w_R_r = np.array([[np.cos(odom_angle), -np.sin(odom_angle)],
                      [np.sin(odom_angle), np.cos(odom_angle)]])
    
    # Transformation Matrix = Rotation + translation!!!!
    w_T_r = np.array([[np.cos(odom_angle), -np.sin(odom_angle),  xw],
                      [np.sin(odom_angle),  np.cos(odom_angle),  yw],
                      [                 0,                   0,   1]])
    
    # World coordinate in vec form
    X_w = np.array([[xw], 
                    [yw]])
                 
    
    x_r, y_r = [], []
    x_w, y_w = [], []
    # for i, angle in enumerate(angles):
        # x_i = ranges[i] * np.cos(angle)
        # y_i = ranges[i] * np.sin(angle)
        # x_r.append(x_i)
        # y_r.append(y_i)
        
        # D = w_T_r @ np.array([[x_i], [y_i], [1]]) 
        # x_w.append(np.cos(odom_angle)*x_i        + np.cos(odom_angle + 1.57)*y_i + xw)
        # y_w.append(np.cos(odom_angle - 1.57)*x_i + np.cos(odom_angle)*y_i + yw       )
        # x_w.append(D[0])
        # y_w.append(D[1])
    
    X_i = np.array([ranges * np.cos(angles), 
                    ranges * np.sin(angles), 
                    np.ones(360)])
    D = w_T_r @ X_i
    
    for i in range(360):
        
        
        px, py = world2map(D[0, i],D[1, i])
        #Bresenham's Line Algorithm
        
        
        # rx, ry = world2map(xw,yw)
        # dx = px - rx
        # dy = py - ry
        
        # y = ry
        # if(dx == 0):
            # continue
        # derror = abs(dy/dx)
        # error = 0
        
        # for x in range(rx, px):
            # map[px, py] = max(map[px, py] - 0.02, 0)
            # error += derror
            # if error >= 0.5:
                # ry += 1
                # error -= 1
        
        
        
        map[px, py] = min(map[px, py] + 0.02, 1)
        
        if map[px, py] > 0.9:
            v=int(map[px,py]*255)
            color=(v*256**2+v*256+v)
            display.setColor(color)
            display.drawPixel(px, py)
        # v=int(map[px,py]*255)
        # color=(v*256**2+v*256+v)
        # display.setColor(color)
        # display.drawPixel(px, py)
    
    # if timestep_count >= 100:
    
        # kernel= np.ones((20,20))
        # result = (map > 0.9) * 1
        # cmap = signal.convolve2d(result,kernel,mode='same')
        # cspace = cmap>0.99
        
        # plt.pause(0.1)
        # plt.imshow(cspace)
        # plt.show()
        # timestep_count = -1

    
    
    
    
    # plt.plot(D[0, :], D[1, :], '.')
    # plt.pause(0.01)
    # plt.show()
    timestep_count = timestep_count + 1
     
    p1, p2 = 6.28 , 15
    leftSpeed = - alpha*p1 + rho*p2
    rightSpeed = alpha*p1 + rho*p2
    
    leftSpeed = max(min(leftSpeed, 6.28), -6.28)
    rightSpeed = max(min(rightSpeed, 6.28), -6.28)
    
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    pass

# Enter here exit cleanup code.
