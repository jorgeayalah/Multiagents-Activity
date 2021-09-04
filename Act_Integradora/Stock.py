#Stock Model
import agentpy as ap
import numpy as np
import time
import math
from matplotlib.pyplot import grid
from agents.Robot import Robot #agent
from agents.Box import Box

class StockModel(ap.Model):
    def setup(self):
        # starts chronometer
        self.currentTime = time.time() #saves a epoch time
        self.currentTime = time.localtime(self.currentTime)
        # create box agents
        nBoxes = 10 
        nRobots = 5
        self.grid = ap.Grid(self, [self.p.size]*2,  track_empty=True)

        self.boxes = ap.AgentList(self, nBoxes, Box) #instantiate
        self.robots = ap.AgentList(self, nRobots, Robot) #instantiate
        self.goal = [0, 0]
        # create grid (stock)
        
        self.grid.add_agents(self.boxes, random=True, empty=True)
        self.grid.add_agents(self.robots, random=True, empty=True)
        
        # communicate positions to both robots and boxes
        self.robots.setPos(self.grid)
        self.boxes.setPos(self.grid)

        self.diagonal = int(math.sqrt(math.pow(self.p.size, 2)))
        self.robots.setDiagonal(self.diagonal)
        self.BRrouter = {}

        for robot in self.robots:
            robot.setGoing(True)
            self.assignTarget(robot)

    def distanceBetween(self, point, point1):
        return math.sqrt(math.pow(point[0]-point1[0], 2) + math.pow(point[1] - point1[1], 2))
    
    def assignTarget(self, robot: Robot):
        if not robot.isGoing():
            robot.setTarget(self.goal)
            return
        minDist = self.diagonal
        boxTarget = None
        boxposTarget = None
        for box in self.boxes:
            if box.id in self.BRrouter or box.condition == 1:
                continue
            boxpos = self.grid.positions[box]
            distance = self.distanceBetween(boxpos, robot.getPos())
            if distance < minDist:
                boxTarget = box
                boxposTarget = boxpos
                minDist = distance
        if boxTarget != None:
            robot.setTarget(boxposTarget)
            self.BRrouter[boxTarget.id] = robot

    def step(self):
        for robot in self.robots:
            robot.step()
            if (robot.isGoing()):
                distance = self.distanceBetween(robot.getPos(), robot.getTarget())
                if abs(distance) <= 1:
                    box = self.grid.agents[robot.getTarget()].to_list()
                    for element in box:
                        if element.type == "box":
                            box = element
                    robot.increaseBoxQty()
                    self.assignTarget(robot)
            else:
                rbGoalDist = abs(self.distanceBetween(robot.getPos(), self.goal))
                if (rbGoalDist <= 1):
                    self.goal = [self.goal[0]+1, self.goal[1]+1]
                    robot.setGoing(True)
                    self.assignTarget(robot)
        

    def end(self):
        # documents the time that the model used
        currentTime2 = time.time()
        currentTime2 = time.localtime(currentTime2)
        finalTime = currentTime2-self.currentTime
        self.report('Tiempo total: ', finalTime)

