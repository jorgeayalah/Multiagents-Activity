import agentpy as ap
import numpy as np

# class Directions():
#     up = 1, down = 3, left = 3, right = 4

class Robot(ap.Agent):
    def setup(self):
        self.going = False # boolean for currently 'going' for a box
        self.target = (0, 0)
        self.boxQty = 0
        self.condition = 1
    def setPos(self, grid):
        self.grid = grid
        self.pos = grid.positions[self]

    def getPos(self):
        return self.pos

    def setTarget(self, target: tuple):
        self.target = target
        self.going = True
        print("Robot got a Box at" + str(self.target))

    def getTarget(self):
        return self.target
    
    def setDiagonal(self, diagonal):
        self.diagonal = diagonal
    
    def getDiagonal(self):
        return self.diagonal

    def setBRrouter(self, BRrouter: dict):
        self.BRrouter = BRrouter

    def increaseBoxQty(self):
        self.boxQty += 1

    def resetBoxQty(self):
        self.boxQty = 0

    def getBoxQty(self):
        return self.boxQty

    def setGoing(self, stat):
        self.going = stat

    def isGoing(self):
        return self.going

    def getVector(self):
        xDelta = self.target[1] - self.pos[1]
        yDelta = self.target[0] - self.pos[0]
        if xDelta == 0 and yDelta == 0:
            return (0, 0)
        vector = np.array([yDelta, xDelta])
        vector = vector / np.linalg.norm(vector)
        vector = vector.tolist()
        vector[0] = int(round(vector[0]))
        vector[1] = int(round(vector[1]))
        return vector
    
    # def takeDirection(self):
    def step(self):
        if self.going:
            movementVec = self.getVector()
            self.grid.move_by(self, movementVec)
            self.pos = self.grid.positions[self]
