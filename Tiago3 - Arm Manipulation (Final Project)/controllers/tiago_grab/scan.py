import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees

class Scan(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Scan, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard

    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(timestep)
        
        self.camera = self.robot.getDevice('camera')
        self.camera.recognitionEnable(timestep)
        self.camera.enable(timestep)
        
        self.logger.debug("%s [Scan::setup()]" % self.name)
        
        self.blackboard.write('x', 0)
        self.blackboard.write('y', 0)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Scan::initialise()]" % self.name)
        
    
    def update(self):
        self.logger.debug("%s [Scan::update()]" % self.name)

        objects = self.camera.getRecognitionObjects()

        if len(objects) == 0:
            return py_trees.common.Status.FAILURE
        
        # Get current position and orienatation
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]

        closest = objects[0]
        closest_distance = np.sqrt((xw - closest.getPosition()[0])**2 + (yw - closest.getPosition()[1])**2)
        for object in objects:
            dis = np.sqrt((xw - object.getPosition()[0])**2 + (yw - object.getPosition()[1])**2)
            if dis < closest_distance:
                closest = object
                closest_distance = dis
        
        y = closest.getPosition()[1]
        x = closest.getPosition()[0]
        
        self.blackboard.write('y', y)
        self.blackboard.write('x', x)
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    
