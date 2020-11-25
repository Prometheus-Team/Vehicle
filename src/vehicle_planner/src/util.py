class Directions:
    @staticmethod
    def getCoordinate(direction):
        if direction == "SOUTH":
            return (1, 0)
        if direction == "NORTH":
            return (-1, 0)
        if direction == "WEST":
            return (0, -1)
        if direction == "EAST":
            return (0, 1)

    def __init__(self, priorityList):
        self.directions = {}
        self.invertedDirections = {}
        for index, value in enumerate(priorityList):
            self.directions[value] = index + 1
            self.invertedDirections[index + 1] = value

    def getDirections(self):
        return (self.directions, self.invertedDirections)


class Point:
    @staticmethod
    def addTuples(tuple1, tuple2):
        return (tuple1[0] + tuple2[0], tuple1[1] + tuple2[1])

    def __init__(self, tuple):
        self.x = tuple[0]
        self.y = tuple[1]
        self.point = tuple

    def __add__(self, tuple):
        self.x = self.x + tuple[0]
        self.y = self.y + tuple[1]
        self.point = (self.x, self.y)