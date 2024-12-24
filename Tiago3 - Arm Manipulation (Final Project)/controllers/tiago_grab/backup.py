import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees

class BackUp(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(BackUp, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.motors = blackboard.read('motors')
        self.JC= self.blackboard.read('JC')
    
    def changeArmPos(self, name):
        for joint in self.JC[name]:
            self.motors[joint].setPosition(self.JC[name][joint])
    def setup(self):
        
        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor.setPosition(float('Inf'))
        self.rightMotor.setPosition(float('Inf'))

        self.gps = self.robot.getDevice('gps')



        self.logger.debug("%s [BackUp::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [BackUp::initialise()]" % self.name)
        self.leftMotor.setVelocity(-0.5)
        self.rightMotor.setVelocity(-0.5)

    
    def update(self):
        self.logger.debug("%s [BackUp::update()]" % self.name)
        x = self.gps.getValues()[0]
        if x < 0.35:
            self.rightMotor.setVelocity(0)
            self.leftMotor.setVelocity(0)
            return py_trees.common.Status.SUCCESS
        
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
