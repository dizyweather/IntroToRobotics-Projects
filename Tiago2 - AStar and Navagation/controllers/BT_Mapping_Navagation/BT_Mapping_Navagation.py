from os.path import exists

import numpy as np
import py_trees
from py_trees.composites import Sequence, Parallel, Selector

from navagation import Navagation
from mapping import Mapping
from planning import Planning

from controller import Robot, Supervisor

# Blackboard definition to pass info between behaviors
class Blackboard:
    def __init__(self) -> None:
        self.data = {}
    
    def read(self, key):
        return self.data.get(key)

    def write(self, key, value):
        self.data[key] = value

# Checks if we already has a map and return the proper pu_tree status
class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        file_exists = exists('cspace.npy')
        if file_exists:
            print('Map already exists!')
            return py_trees.common.Status.SUCCESS
        else:
            print('Map doesn\'t exist!')
            return py_trees.common.Status.FAILURE

# create the Robot instance.
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Inital waypoints for mapping if we aren't provided a map
WP = [(0.55, -0.4), #start
      (0.55, -2.51), #down the right
      (0.18, -2.94), #turning
      (-0.65, -3.09),
      (-1.62, -2.76), #up the left
      (-1.69, -2.13),
      (-1.69, -0.4),
      (-1.31, 0.03),
      (-1.03, 0.28),#turn around
      (-1.51, 0.41),#turn around
      (-1.58, 0.05),
      (-1.69, -0.25),
      (-1.71, -0.9),
      (-1.70, -2),
      (-1.72, -2.82),
      (-1.25, -3.04),
      (-0.7, -3.1),
      (0.27, -2.73),
      (0.64, -0.28),
      (0.2, 0),] 

# Initalize blackboard with this robot and our waypoints
blackboard = Blackboard()
blackboard.write('robot', robot)
blackboard.write('waypoints', WP)

# The behavior tree
tree = Sequence("Main", children=[
            # First generates a map if one is not provided
            Selector("Does Map Exist?", children = [
                DoesMapExist("Test for map"),
                Parallel("Mapping", policy=py_trees.common.ParallelPolicy.SuccessOnOne(), children=[
                    Mapping("map on environment", blackboard),
                    Navagation("move around table", blackboard)
                ])
            ], memory=True),
            # Using said cspace, construct a path and navagate to the bottom left corner
            Planning("Compute path to lower lefthand corner", blackboard, (-1.43, -3.12)),
            Navagation("moving toward lower lefthand corner", blackboard),
            # Using said cspace, construct a path and navagate to the top right corner
            Planning("Compute path to upper righthand corner", blackboard, (0.88, 0.09)),
            Navagation("moving towards upper righthand corner", blackboard)
        ], memory = True)

tree.setup_with_descendants()

while robot.step(timestep) != -1:
    tree.tick_once()

