import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees



class StoreItem(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(StoreItem, self).__init__(name)
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
        self.logger.debug("%s [StoreItem::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [StoreItem::initialise()]" % self.name)
        self.changeArmPos('storeItem')
        
    
    def update(self):
        self.logger.debug("%s [StoreItem::update()]" % self.name)

        current_pos_1 = self.encoders['arm_1_joint'].getValue()
        current_pos_2 = self.encoders['arm_2_joint'].getValue()

        if current_pos_1 > 0.45 and current_pos_2 > 0.95:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
