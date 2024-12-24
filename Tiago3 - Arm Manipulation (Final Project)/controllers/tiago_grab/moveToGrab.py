import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees

py_trees.logging.level = py_trees.logging.Level.DEBUG

class MoveToGrab(py_trees.behaviour.Behaviour):
    def changeArmPos(self, name):
        for joint in self.JC[name]:
            self.motors[joint].setPosition(self.JC[name][joint])
            
    def __init__(self, name, blackboard):
        super(MoveToGrab, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.JC = blackboard.read('JC')
        self.motors = blackboard.read('motors')
        self.encoders = blackboard.read('encoders')

    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())
       
        self.leftMotor = self.robot.getDevice('wheel_left_joint')
        self.rightMotor = self.robot.getDevice('wheel_right_joint')
        self.leftMotor.setPosition(float('Inf'))
        self.rightMotor.setPosition(float('Inf'))

        self.logger.debug("%s [MoveToGrab::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [MoveToGrab::initialise()]" % self.name)
        self.leftMotor.setVelocity(0)
        self.rightMotor.setVelocity(0)

        self.changeArmPos('prepareGrab')
        
        
    def update(self):
        self.logger.debug("%s [MoveToGrab::update()]" % self.name)

        torso_pos = self.encoders['torso_lift_joint'].getValue()
        arm_1_pos = self.encoders['arm_1_joint'].getValue()
        arm_2_pos = self.encoders['arm_2_joint'].getValue()

        if arm_1_pos > 1.5 and arm_2_pos < 0.1 and torso_pos > 0.29:
            self.leftMotor.setVelocity(0.5)
            self.rightMotor.setVelocity(0.5)
        else:
            self.leftMotor.setVelocity(0)
            self.rightMotor.setVelocity(0)
        
        x = self.blackboard.read('x')
        if x < 0.86:
            self.leftMotor.setVelocity(0)
            self.rightMotor.setVelocity(0)
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
