#!/usr/bin/env python

import numpy as np
from util import Directions, Point
from nextDestination import NextDestination
import rospy
from vehicle_lib.srv import Explore, InitBound
from vehicle_lib.msg import IntArr

class Exploration:
    OPEN = 0
    WALL = 1
    VISITED = 2
    BUFFER = 3
    SOUTH = "SOUTH"
    NORTH = "NORTH"
    WEST = "WEST"
    EAST = "EAST"

    BUFFER_SIZE = 3

    # todo: is this not important?
    visitedStack = []

    branchingStack = []
    # todo: the visited stack needs to be updated when the robot moves across a line
    # todo: add branching stack

    # todo: discuss with Milky
    # ? Should we check distances from currentPosition to the closest branching point? Instead of going back to the last seen branching point?
    # ?

    def __init__(self):
        self.initBoundSrv = rospy.Service('/pi/exploration/initBound', InitBound, self.initConfigService)
        self.exploreSrv = rospy.Service('/pi/exploration/nextDestination', Explore, self.getNextStepService)


    def initConfigService(self, req):
        self.initConfig(
            mapShape=(req.width, req.height),
            bounds={
                "northBound": req.up,
                "southBound": req.down,
                "westBound": req.left,
                "eastBound": req.right,
            },
        )

        return True

    def getNextStepService(self, req):
        print(req)
        m = []
        for i in req.grid:
            m.append(list(i.pts))

        m = np.array(m)
        return self.getNextStep(updatedMap=m, currentPosition=(req.x, req.y))
        
    def initConfig(self, mapShape, bounds):
        # self.board = areaMap
        # loop through the board
        # when you get a 1, set the below to self.BUFFER
        # row1 - row2
        # column1 - column2

        self.visitedMap = np.zeros(mapShape)
        # 2D num-py array
        # [[,]]
        # value of a cell --> OPEN, WALL, VISITED, UNVISITED, ROBOT POSITION

        # todo: discuss with Sami
        # ? Open, Wall, and Unvisited should be already in Sami's input
        # ? We're keeping track of Visited in this class.
        # ? It would be great if we can get the Robot's Position from Sami.

        # padding/buffer to check whether something is an obstacle

        # self.stepPriority = stepPriority
        # ["EAST", "NORTH", "SOUTH", "WEST"]

        robotPositionState = NextDestination(
            bounds={
                "northBound": bounds["northBound"],
                "southBound": bounds["southBound"],
                "westBound": bounds["westBound"],
                "eastBound": bounds["eastBound"],
            }
        )
        self.stepPriority = robotPositionState.generateStepPriority()

    def canGoTo(self, position):
        print(position)
        # print(self.board[0][0])
        canGoTo = (
            self.board[position[0], position[1]] == self.OPEN
            and self.board[position[0], position[1]] != self.BUFFER
            and self.visitedMap[position[0], position[1]] != self.VISITED
        )

        return canGoTo

    def getOpenDirections(self, currentPosition):
        currentRow = currentPosition[0]
        currentColumn = currentPosition[1]

        openDirections = []
        sortedOpenDirections = []

        # todo: discuss with Sami/Milky??
        # todo: the +/- 1 values should actually be the vision range of the robot.
        # todo: OR we should decide on the size of a "TILE"

        # decide on adding padding to openings on both sides...
        # dimension of robot - 30x30
        # min distance traveled 30cm

        # ?: OR maybe set the next go to point to a place where we think we might find an open space that is facing our preferred direction

        try:
            if currentRow + 1 >= 0 and self.canGoTo((currentRow + 1, currentColumn)):
                openDirections.append(self.SOUTH)
        except IndexError:
            pass

        try:
            if currentRow - 1 >= 0 and self.canGoTo((currentRow - 1, currentColumn)):
                openDirections.append(self.NORTH)
        except IndexError:
            pass

        try:
            if currentColumn + 1 >= 0 and self.canGoTo((currentRow, currentColumn + 1)):
                openDirections.append(self.EAST)
        except IndexError:
            pass

        try:
            if currentColumn - 1 >= 0 and self.canGoTo((currentRow, currentColumn - 1)):
                openDirections.append(self.WEST)
        except IndexError:
            pass

        for i in openDirections:
            sortedOpenDirections.append(self.stepPriority[0][i])

        sortedOpenDirections.sort()

        for i in range(len(sortedOpenDirections)):
            sortedOpenDirections[i] = self.stepPriority[1][sortedOpenDirections[i]]

        return sortedOpenDirections

    def updateVisitedMap(self, currentPosition, direction):
        self.visitedMap[currentPosition[0], currentPosition[1]] = self.VISITED

        VISITING_RANGE = 1
        # VISITING_RANGE = 4
        if direction == "NORTH" or direction == "SOUTH":
            # if facing NORTH or SOUTH, mark adjacent (EAST and WEST) positions inside the VISION_RANGE as VISITED

            continueLeftCheck = True
            continueRightCheck = True
            for i in range(VISITING_RANGE + 1):

                # todo: add the position to branching stack on the last loop
                if continueLeftCheck is False and continueRightCheck is False:
                    break

                leftSideAdjacentCellColumn = currentPosition[1] - i - 1
                rightSideAdjacentCellColumn = currentPosition[1] + i + 1

                if leftSideAdjacentCellColumn < 0:
                    leftSideAdjacentCellColumn = 0

                try:
                    leftAdjacentPoint = (currentPosition[0], leftSideAdjacentCellColumn)
                    rightAdjacentPoint = (
                        currentPosition[0],
                        rightSideAdjacentCellColumn,
                    )

                    if continueLeftCheck and self.canGoTo(leftAdjacentPoint):
                        self.visitedMap[leftAdjacentPoint] = self.VISITED

                        if i == VISITING_RANGE and self.canGoTo(
                            (leftAdjacentPoint[0], leftAdjacentPoint[1] - 1)
                        ):
                            self.branchingStack.append(leftAdjacentPoint)

                    else:
                        continueLeftCheck = False

                    if continueRightCheck and self.canGoTo(rightAdjacentPoint):
                        self.visitedMap[rightAdjacentPoint] = self.VISITED

                        if i == VISITING_RANGE and self.canGoTo(
                            (rightAdjacentPoint[0], rightAdjacentPoint[1] + 1)
                        ):
                            self.branchingStack.append(rightAdjacentPoint)

                    else:
                        continueRightCheck = False

                except IndexError:
                    break

        elif direction == "EAST" or direction == "WEST":
            continueTopCheck = True
            continueBottomCheck = True

            for i in range(VISITING_RANGE + 1):
                topSideAdjacentCellColumn = currentPosition[0] - i - 1
                bottomSideAdjacentCellColumn = currentPosition[0] + i + 1

                if topSideAdjacentCellColumn < 0:
                    topSideAdjacentCellColumn = 0

                if continueTopCheck is False and continueBottomCheck is False:
                    break

                try:
                    topAdjacentPoint = (topSideAdjacentCellColumn, currentPosition[1])
                    bottomAdjacentPoint = (
                        bottomSideAdjacentCellColumn,
                        currentPosition[1],
                    )

                    if continueTopCheck and self.canGoTo((topAdjacentPoint)):
                        self.visitedMap[topAdjacentPoint] = self.VISITED

                        if i == VISITING_RANGE and self.canGoTo(
                            (
                                topAdjacentPoint[0] - 1,
                                topAdjacentPoint[1],
                            )
                        ):
                            self.branchingStack.append(topAdjacentPoint)

                    else:
                        continueTopCheck = False

                    if continueBottomCheck and self.canGoTo((bottomAdjacentPoint)):
                        self.visitedMap[bottomAdjacentPoint] = self.VISITED

                        if i == VISITING_RANGE and self.canGoTo(
                            (
                                bottomAdjacentPoint[0] + 1,
                                bottomAdjacentPoint[1],
                            )
                        ):
                            self.branchingStack.append(bottomAdjacentPoint)

                    else:
                        continueBottomCheck = False

                except IndexError:
                    break

    # Tuple with 2 elements
    # (X, Y) - where X is the row index
    #       - where Y is the column index
    def getNextStep(self, updatedMap, currentPosition):
        self.board = updatedMap
        unexploredPositions = np.where(self.visitedMap == self.OPEN)
        if len(unexploredPositions[0]) == 0 and len(unexploredPositions[0]) == 0:
            return IntArr([])
        openDirections = self.getOpenDirections(currentPosition)

        if len(openDirections) > 1:
            if len(self.branchingStack) == 0 or (
                len(self.branchingStack) > 0
                and self.branchingStack[-1] != (currentPosition)
            ):
                self.branchingStack.append(currentPosition)

        directionFacing = ""
        if len(openDirections) > 0:
            directionFacing = openDirections[0]
        self.updateVisitedMap(currentPosition, directionFacing)

        # print("*************************************")
        # print(openDirections)
        # print("*************************************")
        # print(self.visitedMap)

        for direction in openDirections:
            prevStepPriority = self.stepPriority[0][direction]

            while True:
                newPoint = Point.addTuples(
                    currentPosition, Directions.getCoordinate(direction)
                )

                openDirections = self.getOpenDirections(newPoint)
                if (
                    len(openDirections) > 0
                    and self.stepPriority[0][openDirections[0]] == prevStepPriority
                ):
                    self.updateVisitedMap(newPoint, direction)
                    currentPosition = newPoint
                else:
                    print(newPoint)
                    return IntArr(newPoint)

        else:
            # backtracking
            # print("got back to branching point")
            if len(self.branchingStack) > 0:
                returnPoint = self.branchingStack.pop()
                print(returnPoint)
                return IntArr(returnPoint)
            else:
                # print("no more moves")
                return IntArr([])

        # (x,y)

def main():
    rospy.init_node("Exploration")
    Exploration()
    rospy.spin()

if __name__=='__main__':
    main()