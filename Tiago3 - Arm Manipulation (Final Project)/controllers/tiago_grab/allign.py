import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees

class Allign(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Allign, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard

    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())

        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor.setPosition(float('Inf'))
        self.rightMotor.setPosition(float('Inf'))



        self.logger.debug("%s [Allign::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Allign::initialise()]" % self.name)
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

    
    def update(self):
        self.logger.debug("%s [Allign::update()]" % self.name)
        y = self.blackboard.read('y')
        
        if y < -0.03:
            leftAdjustment = 0.2
            rightAdjustment = -0.2
        else:
            leftAdjustment = -0.2
            rightAdjustment = 0.2
        
        
        self.leftMotor.setVelocity(self.leftMotor.getVelocity() + leftAdjustment)
        self.rightMotor.setVelocity(self.rightMotor.getVelocity() + rightAdjustment)
        
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
