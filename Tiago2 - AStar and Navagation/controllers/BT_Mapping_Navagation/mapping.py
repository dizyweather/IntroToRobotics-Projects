import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees

# transform from world to map coordinates
def world2map(xw, yw):
    return max(min(int((xw + 2.15)/3.6 * 300), 299), 0) , max(min(int((1.7 - yw)/5.55 * 460), 459), 0)

# transform from map to world coordinates
def map2world(px, py):
    xw = px / (300.0/3.6) - 2.15
    yw = py / (460.0/5.55) + 1.7
    
    return [xw, yw]

class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        self.hasrun = False
        self.robot = blackboard.read('robot')

    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())

        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(timestep)
        self.lidar.enablePointCloud()

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(timestep)

        self.display = self.robot.getDevice('display')
        
        self.logger.debug("%s [Mapping::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Mapping::initialise()]" % self.name)
        self.map = np.zeros((300,460))
        self.angles = np.linspace(2.094, -2.094, 667)
        
    
    def update(self):
        self.hasrun = True
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        odom_angle =np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])
        
        px, py = world2map(xw,yw)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(px,py)

    
        # get data from lidar
        ranges = np.array(self.lidar.getRangeImage())
        ranges[ranges == np.inf] = 100
        
    
        # Transformation Matrix = Rotation + translation!!!!
        w_T_r = np.array([[np.cos(odom_angle), -np.sin(odom_angle),  xw + 0.2 * np.cos(odom_angle)],
                        [np.sin(odom_angle),  np.cos(odom_angle),  yw + 0.2 * np.sin(odom_angle)],
                        [                 0,                   0,   1]])
        
    
        # Multiply distance with corresponding angle
        X_i = np.array([ranges * np.cos(self.angles), 
                        ranges * np.sin(self.angles), 
                        np.ones(667)])
        D = w_T_r @ X_i
        
        # for each pixel, increase confidence at location
        for i in range(667):
            if i < 160 or i > 667 - 160:
                continue
            
            px, py = world2map(D[0, i],D[1, i])


            self.map[px, py] = min(self.map[px, py] + 0.01, 1)
            
            # display said pixels
            v=int(self.map[px,py]*255)
            color=(v*256**2+v*256+v)
            self.display.setColor(color)
            self.display.drawPixel(px, py)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        if self.hasrun:
            kernel= np.ones((55,55))
            result = (self.map > 0.9) * 1
            cmap = signal.convolve2d(result,kernel,mode='same')
            cspace = cmap>0.99

            np.save('cspace', cspace)

    

