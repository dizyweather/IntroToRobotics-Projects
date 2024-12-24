import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees

# transform from world to map coordinates
def world2map(xw, yw):
    return max(min(int((xw + 2.15)/3.6 * 300), 299), 0) , max(min(int((1.7 - yw)/5.55 * 460), 459), 0)

class Stop(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Stop, self).__init__(name)
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


        self.logger.debug("%s [Navagation::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Navagation::initialise()]" % self.name)
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        self.index = 0

        
    
    def update(self):
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)
            
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
