import rospy, math, time
from vehicle_lib.srv import Explore, InitBound, InitMap, GetMap, ScanArea, GetShortestPath
from vehicle_lib.msg import Location, Speed
import enum

class Boundary:
    def __init__(self, left, right, up, down):
        self.left = left
        self.right = right
        self.up = up
        self.down = down

    def convertMeterToGrid(self, ratio):
        return (
            math.ceil(self.left/ratio),
            math.ceil(self.right/ratio),
            math.ceil(self.up/ratio),
            math.ceil(self.down/ratio)
        )

class Status(enum.Enum):
    STARTED = 0
    COMPLETED = 1
    FAILED = 2
    RETURNING = 3
    TO_START = 4

class Master:
    def __init__(self):
        self.mappingStatus = Status.TO_START
        self.auto = True
        self.overrideBatteryWarning = False

        self.speedToBatteryLevelRatio = 100/1.2
        self.criticalBatteryLevel = 30
        self.minBatteryForExploration = 80
        self.minBatteryForReturn = 45
        self.worldToMapRatio = 0.02    # 1 cell = 0.02 meters in the world 

        self.speed = Speed(0,0)
        self.curLoc = (time.time(), Location(0,0,0))
        self.initLoc = Location(0, 0)

        # Services
        self.initBoundSrv = rospy.ServiceProxy('/pi/exploration/initBound', InitBound)
        self.getNextDestinationSrv = rospy.ServiceProxy('/pi/exploration/nextDestination', Explore)

        self.initMapSrv = rospy.ServiceProxy('/pi/mapper/initMap', InitMap)
        self.getMapSrv = rospy.ServiceProxy('/pi/mapper/getMap', GetMap)

        self.scanAreaSrv = rospy.ServiceProxy('/pi/api/scanCmd', ScanArea)
        self.getShortestPathSrv = rospy.ServiceProxy('/pi/travel/getShortestPath', GetShortestPath)

        # Topics
        self.subCurLoc = rospy.Subscriber('/pi/localization/curLoc', Location, self.curLocListener)


    def config(self):
        # Compute speed to pwm ratio
        # Initiate map with given size
        # Initiate planner with given size
        pass


    def checkBatteryLevelForExploration(self):
        if self.calcBatteryLevel() < self.minBatteryForExploration:
            return False

        return True

    def checkBatteryLevelForReturn(self):
        if self.calcBatteryLevel() < self.minBatteryForReturn:
            return False

        return True
        
    def calcBatteryLevel(self):
        return ((self.speed[0] + self.speed[1])/2) * self.speedToBatteryLevelRatio

    def startMapping(self, boundary, detail, checkBattery=True):
        if checkBattery:    # Make the vehicle go with maximum power and record speed
            if not self.checkBatteryLevelForExploration():
                return False

        a = boundary.convertMeterToGrid(self.worldToMapRatio)
        w = a[0] + a[1]
        h = a[2] + a[3]

        # initialize mapper
        mapInitialized = self.initMapSrv(w, h)
        
        # initialize exploration map
        # detail value can be 1 - 5 showing the tolerance of the undiscovered space before we finish exploration
        boundInitialized = self.initBoundSrv(w, h, a[0], a[1], a[2], a[3], detail)

        
    '''
    # todo:
        loop:
            # scan area
            # make map      # done by mapper as soon as a reading is published by the arduino
            # send map to exploration and get next point
            # compute shortest path to found point
            # execute path and do localization simultaneously
    '''
    def mapArea(self):
        while True:
            if self.mappingStatus == Status.RETURNING:
                return True
                
            scanStarted = self.scanAreaSrv(True)

            if not scanStarted:
                print("An error has occured trying to start the scanner")
                return False
            
            # 2D grid map of the area is returned as 2D array
            map = self.getMapSrv(True)

            loc = self.curLoc[1]

            # A tuple/array is returned for the next best position to discover [x, y]
            nextDest = self.getNextDestinationSrv(map, loc.x, loc.y)

            if len(nextDest) == 0:
                nextDest = [self.initLoc.x, self.initLoc.y]
                self.mappingStatus = Status.RETURNING       # ????? Add another type of status to show if the map completed its mission

            # Array of locations is returned as a path to follow
            path = self.getShortestPathSrv(map, loc, Location(nextDest[0], nextDest[1]))

            # Execute path
            # Maybe rotate vehicle slowly in its own axis to get more images
            # Then repeat loop again


    def abortMapping(self):
        # find shortest path between current location and intital location
        pass


    # Callbacks to topics
    # ======================================================================
    def curLocListener(self, msg):
        self.curLoc = (time.time(), Location(msg.x, msg.y, msg.heading))