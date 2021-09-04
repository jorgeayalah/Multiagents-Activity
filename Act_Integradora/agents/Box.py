import agentpy as ap

class Box(ap.Agent):
    def setup(self):
        self.condition = 2
        self.pos = (0, 0)
    def setPos(self, grid):
        self.grid = grid
        self.pos = self.grid.positions[self]
    def getPos(self):
        return self.pos
