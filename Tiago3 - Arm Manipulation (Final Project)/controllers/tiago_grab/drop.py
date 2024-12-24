import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees



class Drop(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Drop, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.JC = blackboard.read('JC')
        self.motors = blackboard.read('motors')
        self.encoders = blackboard.read('encoders')

    def changeArmPos(self, name):
        for joint in self.JC[name]:
            self.motors[joint].setPosition(self.JC[name][joint])
            
    
    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())
        
        self.leftGripper = self.blackboard.read('motors')['gripper_left_finger_joint']
        self.rightGripper = self.blackboard.read('motors')['gripper_right_finger_joint']

        self.logger.debug("%s [Drop::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Drop::initialise()]" % self.name)
        self.changeArmPos('dropItem')
        
    
    def update(self):
        self.logger.debug("%s [Drop::update()]" % self.name)

        current_pos_1 = self.encoders['arm_1_joint'].getValue()
        current_pos_2 = self.encoders['arm_2_joint'].getValue()

        if current_pos_1 < 0.09 and current_pos_2 < 0.05:
            self.changeArmPos('openGripper')
            
        
        grip_1_pos = self.encoders['gripper_left_finger_joint'].getValue()
        grip_2_pos = self.encoders['gripper_right_finger_joint'].getValue()

        if grip_1_pos > 0.044 and grip_2_pos > 0.044:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
