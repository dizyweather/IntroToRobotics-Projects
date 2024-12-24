import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees



class Grab(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Grab, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.JC = blackboard.read('JC')
        self.motors = blackboard.read('motors')

    def changeArmPos(self, name):
        for joint in self.JC[name]:
            self.motors[joint].setPosition(self.JC[name][joint])
            
    
    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())
        
        self.leftGripper = self.blackboard.read('motors')['gripper_left_finger_joint']
        self.rightGripper = self.blackboard.read('motors')['gripper_right_finger_joint']

        self.logger.debug("%s [Grab::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Grab::initialise()]" % self.name)
        self.changeArmPos('closeGripper')
        
    
    def update(self):
        self.logger.debug("%s [Grab::update()]" % self.name)

        rForce = self.rightGripper.getForceFeedback()
        lForce = self.leftGripper.getForceFeedback()
        
        if rForce < -10 and lForce < -10:
            # self.rightGripper.setVelocity(0)
            # self.leftGripper.setVelocity(0)
            return py_trees.common.Status.SUCCESS
        
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
