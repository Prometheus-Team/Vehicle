from util import Directions

# Finding Step Priority
# todo: change class name
class NextDestination:
    EAST = "EAST"
    WEST = "WEST"
    NORTH = "NORTH"
    SOUTH = "SOUTH"

    def __init__(self, bounds):
        self.bounds = bounds

    def generateStepPriority(self):
        # Horizontal Distance
        # eastDistance = abs(self.currentPosition["x"] - self.bounds["eastBound"])
        # westDistance = abs(self.currentPosition["x"] - self.bounds["westBound"])
        nearestHorizontalDistance = 0
        horizontalPriority = [self.EAST, self.WEST]
        if self.bounds["eastBound"] < self.bounds["westBound"]:
            # TODO: Add a buffer - how much should we care about a few decimal differences
            nearestHorizontalDistance = self.bounds["eastBound"]
            horizontalPriority = [self.EAST, self.WEST]
            # print("Nearest EAST", nearestHorizontalDistance)
        elif self.bounds["westBound"] < self.bounds["eastBound"]:
            nearestHorizontalDistance = self.bounds["westBound"]
            horizontalPriority = [self.WEST, self.EAST]
            # print("Nearest WEST", nearestHorizontalDistance)
        else:
            # Have Random priority
            nearestHorizontalDistance = self.bounds["eastBound"]
            # print("Nothin")

        # print("H: ", nearestHorizontalDistance)

        # Vertical Distance
        # northDistance = abs(self.currentPosition["y"] - self.bounds["northBound"])
        # southDistance = abs(self.currentPosition["y"] - self.bounds["southBound"])
        nearestVerticalDistance = 0
        verticalPriority = [self.NORTH, self.SOUTH]
        if self.bounds["northBound"] < self.bounds["southBound"]:
            nearestVerticalDistance = self.bounds["northBound"]
            verticalPriority = [self.NORTH, self.SOUTH]
            # print("Nearest NORTH", nearestVerticalDistance)
        elif self.bounds["southBound"] < self.bounds["northBound"]:
            nearestVerticalDistance = self.bounds["southBound"]
            verticalPriority = [self.SOUTH, self.NORTH]
            # print("Nearest SOUTH", nearestVerticalDistance)
        else:
            # Have Random priority
            nearestVerticalDistance = self.bounds["northBound"]
            # print("NothinGG")

        # print("V: ", nearestVerticalDistance)

        if nearestHorizontalDistance < nearestVerticalDistance:
            # print("ONE")
            # stepPrecedence
            stepPriority = [
                horizontalPriority[0],
                verticalPriority[0],
                verticalPriority[1],
                horizontalPriority[1],
            ]
        elif nearestHorizontalDistance > nearestVerticalDistance:
            # print("TWO")
            # stepPrecedence
            stepPriority = [
                verticalPriority[0],
                horizontalPriority[0],
                horizontalPriority[1],
                verticalPriority[1],
            ]
        else:
            # print("THREE")
            # At the center so precedence does not matter
            stepPriority = [
                verticalPriority[0],
                horizontalPriority[0],
                horizontalPriority[1],
                verticalPriority[1],
            ]

        directionPriority = Directions(stepPriority).getDirections()
        return directionPriority


# robotPositionState = NextDestination(
#     bounds={"northBound": 20, "southBound": 20, "westBound": 10, "eastBound": 30}
# )

# print("LOOKIE")
# print(robotPositionState.generateStepPriority())


# if eastBound > westBound