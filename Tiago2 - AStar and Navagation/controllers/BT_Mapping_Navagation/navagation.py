import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees

# transform from world to map coordinates
def world2map(xw, yw):
    return max(min(int((xw + 2.15)/3.6 * 300), 299), 0) , max(min(int((1.7 - yw)/5.55 * 460), 459), 0)

class Navagation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Navagation, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard

    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(timestep)

        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor.setPosition(float('Inf'))
        self.rightMotor.setPosition(float('Inf'))

        self.marker = self.robot.getFromDef('marker').getField('translation')

        self.logger.debug("%s [Navagation::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Navagation::initialise()]" % self.name)
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        self.index = 0

        self.WP = self.blackboard.read('waypoints')
        
    
    def update(self):
        self.logger.debug("%s [Navagation::update()]" % self.name)

        # Get current position and orienatation
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]

        odom_angle =np.arctan2(self.compass.getValues()[0],self.compass.getValues()[1])

        # Euclidean distance to WP error
        rho = np.sqrt((xw - self.WP[self.index][0])**2 + (yw - self.WP[self.index][1])**2)
        alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - odom_angle

        # limit angle between +- pi
        if alpha > np.pi:
            alpha = alpha - 2*np.pi
        elif alpha < -np.pi:
            alpha = alpha + 2*np.pi

        

        # set speed based on error from WP
        p1, p2 = 4 , 2
        leftSpeed = - alpha*p1 + rho*p2
        rightSpeed = alpha*p1 + rho*p2
        
        leftSpeed = max(min(leftSpeed, 6.28), -6.28)
        rightSpeed = max(min(rightSpeed, 6.28), -6.28)
        
        self.leftMotor.setVelocity(leftSpeed)
        self.rightMotor.setVelocity(rightSpeed)

        # Check if close to next waypoint and if all waypoints are done
        if rho < 0.4:
            self.index = (self.index + 1) 
            if self.index == len(self.WP):
                self.leftMotor.setVelocity(0)
                self.rightMotor.setVelocity(0)
                return py_trees.common.Status.SUCCESS
            
            self.marker.setSFVec3f([*self.WP[self.index], 0.0669])
            
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
