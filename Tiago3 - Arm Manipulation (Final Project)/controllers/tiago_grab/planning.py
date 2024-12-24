from collections import defaultdict
import numpy as np
from matplotlib import pyplot as plt
from py_trees.common import Status
from scipy import signal
import py_trees
from heapq import heapify, heappush, heappop


# transform from world to map coordinates
def world2map(xw, yw):
    return max(min(int((xw + 2.15)/3.6 * 300), 299), 0) , max(min(int((1.7 - yw)/5.55 * 460), 459), 0)
    
# transform from map to world coordinates
def map2world(px, py):
    xw = px / (300.0/3.6) - 2.15
    yw = 1.7 - py / (460.0/5.55)
    
    return [xw, yw]

# My implementation of A* search
def getShortestPath(map, start, goal):
    def getNeighbors(x, y):
        neighbors = []
        for i in range(3):
            for j in range(3):
                if j == 1 and i == 1:
                    continue
                if abs(i - 1) == 1 and abs(j - 1) == 1:
                    neighbors.append((np.sqrt(2),(x + i - 1, y + j - 1)))
                else:
                    neighbors.append((1,(x + i - 1, y + j - 1)))
        return neighbors
    
    distances = defaultdict(lambda:float("inf"))
    distances[start]=0

    path = []
    min_queue = []
    heapify(min_queue)
    heappush(min_queue, (0, start))

    dict = {}
    dict[start] = 1

    while len(min_queue) != 0:
        current_dis, current = heappop(min_queue)
        if(current == goal):
            
            retrace = current
            while retrace != 1:
                path.insert(0, retrace)
                retrace = dict[retrace]
            break

        neighbors = getNeighbors(current[0], current[1])

        for (dis,(x, y)) in neighbors:
            if x < 0 or x >= len(map):
                continue
            if y < 0 or y >= len(map[0]):
                continue
            
            if map[x][y] != 1:
                if (x, y) in distances:
                    if distances[(x, y)] > dis + distances[current]:
                        distances[(x, y)] = dis + distances[current]
                        dict[(x, y)] = current
                    continue
                dict[(x, y)] = current
                distances[(x, y)] = dis + distances[current] 
                heappush(min_queue, (distances[(x, y)] + np.sqrt((goal[0]-x)**2+(goal[1]-y)**2), (x, y)))
    
    return path
        
    

class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
        
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        px, py = world2map(goal[0], goal[1])
        self.goal = (px, py)

    def setup(self):
        timestep = int(self.robot.getBasicTimeStep())

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(timestep)

        self.display = self.robot.getDevice('display')
        
        self.logger.debug("%s [Mapping::setup()]" % self.name)
    
    def initialise(self) -> None:
        self.logger.debug("%s [Mapping::initialise()]" % self.name)
        
    
    def update(self):
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]

        current_position = world2map(xw, yw)
         
        cspace = np.load('cspace.npy')
        self.display.setColor(0xFFFFFF)

        # Draws the cspace provided onto the display
        for i in range(len(cspace)):
            for j in range(len(cspace[0])):
                if cspace[i][j] == 1:
                    self.display.drawPixel(i, j)
            
        # Generates pass in pixel coordinates
        path = getShortestPath(np.load('cspace.npy'), current_position, self.goal)
        world_path = []
        self.display.setColor(0xFF00FF)
        
        # Simutaniously drawing our pixel path in purple and converting them to world coordinates
        for p in path:
            self.display.drawPixel(p[0], p[1])
            world_path.append(map2world(p[0], p[1]))
        self.display.setColor(0xFFFFFF)
        
        # Writes the world coordinate path to the blackboard for navagation to use
        self.blackboard.write('waypoints', world_path)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        

    

