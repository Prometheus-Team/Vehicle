import rospy, math, time
import hashlib, cv2

from vehicle_lib.srv import Explore, InitBound, InitMap, GetMap, ScanArea, GetShortestPath, Goto
from vehicle_lib.msg import Location, Speed, Path, Distance
import enum
from std_msgs.msg import Bool, Int32, Float32, Int16
import socket, threading
import numpy as np

# import cPickle as pickle
import pickle

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

    def getInitLoc(self):
        w = self.left + self.right
        h = self.up + self.down

        return (w/2.0) - self.right, (h/2.0) - self.up

    def getWidthHeight(self):
        return self.left + self.right, self.up + self.down

    def getWidthHeightGrid(self, ratio):
        a = self.convertMeterToGrid(ratio)
        return a[0] + a[1], a[2] + a[3]

    def getArea(self):
        w, h = self.getWidthHeight()
        return w * h

class Status(enum.Enum):
    TO_START = 0
    STARTED = 1
    PAUSE = 2
    RETURNING = 3
    COMPLETED = 4
    FAILED = 5

class BatteryStatus(enum.Enum):
    NO_EXPLORATION = 0
    START_RETURN = 1
    SHUTDOWN_SYSTEM = 2
    NORMAL = 3

class SensorData:
    def __init__(self, speed, heading, frontDistance, curLoc, batteryLevel):
        self.speed = speed
        self.heading = heading
        self.frontDistance = frontDistance
        self.curLoc = curLoc
        self.batteryLevel = batteryLevel

class Master:
    def __init__(self, ip):
        self.IP = ip
        self.sensorStreamConnected = False
        self.cmdRecvConnected = False

        self.boundInitialized = False
        self.mapInitialized = False

        self.mappingStatus = self.preMappingStatus = Status.TO_START
        self.auto = True
        self.overrideBatteryWarning = False

        self.maxAreaToExplore = 300        # allowed maximum length to explore in one side
        self.maxSpeed = 3
        self.minSpeed = 1
        self.speedToBatteryLevelRatio = 100/1.2     # 1.2m/s shows 100% battery level
        self.criticalBatteryLevel = 30      # minimum battery level to be announced before battery dies
        self.minBatteryForExploration = 80      # min battery level to start exploration
        self.minBatteryForReturn = 45       # min battery level the vehicle must have to stop exploring and start returning to base
        self.worldToMapRatio = 0.02    # 1 cell = 0.02 meters in the world 

        self.mapUpdateEvery = 5     # in seconds
        self.lastMapSentTime = time.time()

        self.map = None
        self.batteryLevel = (0,0)      # (batteryLevel, batteryStatus)
        self.speed = Speed(0,0, None, 0, 0)
        self.heading = 0
        self.frontDistance = 0
        self.curLoc = [time.time(), Location(0,0,0, None)]
        self.initLoc = [time.time(), Location(0, 0, 0, None)]

        # Services
        self.initBoundSrv = rospy.ServiceProxy('/pi/exploration/initBound', InitBound)
        self.getNextDestinationSrv = rospy.ServiceProxy('/pi/exploration/nextDestination', Explore)

        self.initMapSrv = rospy.ServiceProxy('/pi/mapper/initMap', InitMap)
        self.getMapSrv = rospy.ServiceProxy('/pi/mapper/getMap', GetMap)

        self.scanAreaSrv = rospy.ServiceProxy('/pi/api/scanCmd', ScanArea)
        self.getShortestPathSrv = rospy.ServiceProxy('/pi/travel/getShortestPath', GetShortestPath)

        self.gotoLocSrv = rospy.ServiceProxy('/pi/steering/goto', Goto)

        # Topics
        self.subCurLoc = rospy.Subscriber('/pi/localization/currentLoc', Location, self.curLocListener)

        self.subHeading = rospy.Subscriber('/pi/api/heading', Float32, self.headingListener)
        self.subSpeed = rospy.Subscriber('/pi/api/speed', Speed, self.speedListener)
        self.subFrontDis = rospy.Subscriber('/pi/api/frontDistance', Distance, self.frontDistanceListener)


        self.pubAuto = rospy.Publisher('/pi/master/auto', Bool, queue_size=10)
        self.pubManual = rospy.Publisher('/pi/api/manual', Int16, queue_size=10)

        self.configSock()
        

    def configSock(self):
        try:
            self.sensorStreamSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # client that streams sensor data
            self.sensorStreamSock.bind(('', 9998))
            
            self.cmdRecvSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # server that accepts cmds
            self.cmdRecvSock.bind(('', 8090))
            self.cmdRecvConnected = True

            sense = threading.Thread(target=self.sendSensorData)
            sense.setDaemon(True)
            sense.start()

            cmd = threading.Thread(target=self.cmdRecvCallback)
            cmd.setDaemon(True)
            cmd.start()
            

        except Exception as e:
            print(e)
            print("Unable to connect to device") 

    def imageToSha1(self, URL="http://192.168.0.147:8000/stream.mjpg"):
        inputStream = cv2.VideoCapture(URL)

        while True:
            _ret, image = inputStream.read()
            if _ret == False:
                break
            self.imageSha = hashlib.sha1(image).hexdigest()

    def startStreamingSensorData(self):
        sense = threading.Thread(target=self.sendSensorData)
        sense.start()
        sense.join()

        # # todo: initialize this thread somewhere
        # cmd = threading.Thread(target=self.imageToSha1)
        # cmd.start()
        # cmd.join()

    def sendSensorData(self):
        if not self.sensorStreamConnected:
            try:
                self.sensorStreamSock.listen(5)
                self.sensorStreamConnected = True

            except:
                print("Unable to connect")
                return False
        while True:
            print("Sending sensor data")
            try:
                conn, addr = self.sensorStreamSock.accept()
                while True:
                    sensorData = self.getSensorData()
                    # if time.time() - self.lastMapSentTime > self.mapUpdateEvery:
                    #     grid = self.getMapSrv(True).map
                    #     self.map = np.array([list(i.pts) for i in grid])
                    #     print("Shape:",self.map.shape)
                    #     sensorData['map'] = self.map

                    # else:
                    #     sensorData['map'] = None

                    print(sensorData)

                    data = pickle.dumps(sensorData)
                    conn.sendall(data)

                    time.sleep(0.1)

            except:
                continue

        self.sensorStreamSock.close()

    def cmdRecvCallback(self):
        print("Cmd waiting")
        self.cmdRecvSock.listen(5)
        while True:
            try:
                conn, addr = self.cmdRecvSock.accept()
                data = conn.recv(62556)
                print(data)
                cmd = pickle.loads(data)
                print(cmd)
                ret = self.routeCmd(cmd)

                if ret and len(ret)> 1:
                    suc, code = ret
                    if suc==True or suc==False:
                        stat = {'success':suc, 'code':code}
                        conn.sendall(pickle.dumps(stat))

                else:
                    stat = {'success':True, 'code':200}
                    conn.sendall(pickle.dumps(stat))

            except:
                continue

        self.cmdRecvSock.close()

    def routeCmd(self, cmd):
        cmdList = {
            'startExplore':['left','right','up','down'],
            'abortExplore':[True, False],
            'move':['forward','backward','left','right','stop'],
            'manualControlChange':[True, False],
            'systemCheck':[True, False]
        }
        # print(cmd.keys())
        c = str(cmd.get('command'))
        if c and c in cmdList.keys():
            arg = cmd.get(u'args')
            if c == 'startExplore':
                print("Start explore")
                if sum(np.array([isinstance(a, int) or isinstance(a, float) for a in arg.values()]) == 0) > 0:
                    return False, 400

                if (arg['left'] + arg['right']) * (arg['up'] + arg['down']) > self.maxAreaToExplore:
                    return False, 400
                
                bound = Boundary(arg['left'], arg['right'], arg['up'], arg['down'])
                ret = self.startMapping(bound, 0, False)
                if ret:
                    return True, 200
                else:
                    return False, 500

            elif c == 'abortExplore':
                self.abortMapping()

            elif c == 'move':
                print("Here in driving")
                cmdCode = {'stop':0,'forward':1,'backward':2,'left':3,'right':4}
                if not self.auto:
                    cm = cmdCode.get(arg)
                    if cm is not None:
                        ret = self.cmdManual(cm)
                        if ret:
                            return True, 200
                        else:
                            return False, 500
                    else:
                        return False, 400

            elif c == 'manualControlChange':
                print("Change auto")
                self.changeDriver(arg)

            elif c == 'systemCheck':
                return self.checkSystem(), 200
            

    def getSensorData(self):
        URL = "http://192.168.0.147:8000/stream.mjpg"
        inputStream = cv2.VideoCapture(URL)
        _ret, image = inputStream.read()
        imageHash = hashlib.sha1(image).hexdigest()

        return {
            'speed':[self.speed.left, self.speed.right],
            'heading':self.heading,
            'location':[self.curLoc[1].x, self.curLoc[1].y, self.heading],
            'frontDistance':self.frontDistance,
            'batteryLevel':self.batteryLevel,
            'imageSha': imageHash
        }

    def config(self):
        # Compute speed to pwm ratio
        # Initiate map with given size
        # Initiate planner with given size
        pass

    # accept manual control cmd from GUI and execute
    def cmdManual(self, direction):
        try:
            if direction >= 0 and direction <= 4:
                self.pubManual.publish(direction)

            return True

        except:
            return False

    # change driving from autonomous to manual or vice verse 
    def changeDriver(self, manual):
        if manual in (True, False):
            self.auto = not manual
            self.pubAuto.publish(self.auto)

            if self.auto and self.mapInitialized and self.boundInitialized:
                self.mapArea()

    def checkSystem(self):
        interval = 5
        # # 1 - Forward motion
        # self.cmdManual(1)
        # time.sleep(interval)
        # self.cmdManual(0)

        # self.updateBatteryLevel()

        return True

    # used to accept system battery low warning 
    def changeMappingStatus(self, status):
        self.mappingStatus = status
        self.mappingStatusWithBattery()

    # ignore system battery warning
    def overrideBatteryWarn(self):
        self.overrideBatteryWarning = True
        self.mappingStatus = self.preMappingStatus
        self.mappingStatusWithBattery()

    # check system battery level and pause mapping if necessary
    def mappingStatusWithBattery(self):
        self.updateBatteryLevel()
        
        if self.mappingStatus != Status.PAUSE and not self.overrideBatteryWarning and self.batteryLevel[1] != BatteryStatus.NORMAL:
            self.mappingStatus = Status.PAUSE

    def updateBatteryLevel(self):
        batLevel = self.calcBatteryLevel()

        if self.mappingStatus == Status.TO_START and batLevel <= self.minBatteryForExploration:
            stat = BatteryStatus.NO_EXPLORATION

        elif self.mappingStatus == Status.STARTED and batLevel <= self.minBatteryForReturn:
            stat = BatteryStatus.START_RETURN

        elif batLevel <= self.criticalBatteryLevel:
            stat = BatteryStatus.SHUTDOWN_SYSTEM

        else:
            stat = BatteryStatus.NORMAL

        self.batteryLevel = (batLevel, stat)
        
    def calcBatteryLevel(self):
        # return math.ceil(((self.speed.left + self.speed.right)/2) * self.speedToBatteryLevelRatio)
        sp = (self.speed.left + self.speed.right)/2.0
        expSp = (self.speed.expLeft + self.speed.expRight)/2.0
        return sp/expSp


    def startMapping(self, boundary, detail, checkBattery=True):
        if checkBattery:    # Make the vehicle go with maximum power and record speed
            self.updateBatteryLevel()
            if self.batteryLevel[1] != BatteryStatus.NORMAL:
                return False

        bound = boundary.convertMeterToGrid(self.worldToMapRatio)
        w, h = boundary.getWidthHeightGrid(self.worldToMapRatio)

        x,y = boundary.getInitLoc()
        self.initLoc[1].x = x
        self.initLoc[1].y = y

        # initialize mapper
        self.mapInitialized = self.initMapSrv(w, h).done
        
        # initialize exploration map
        # detail value can be 1 - 5 showing the tolerance of the undiscovered space before we finish exploration
        # self.boundInitialized = self.initBoundSrv(w, h, bound[0], bound[1], bound[2], bound[3], detail).done

        if self.auto:
            mapThr = threading.Thread(target=self.mapArea)
            mapThr.start()
            mapThr.join()

        return True
            # self.mapArea()
        print("Map init: ", self.mapInitialized)
        
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
        self.mappingStatus = Status.STARTED
        self.preMappingStatus = Status.STARTED

        while True:
            # self.mappingStatusWithBattery()

            if not self.auto:
                return False

            if self.mappingStatus == Status.PAUSE:      # mapping paused until override
                continue           

            scanStarted = self.scanAreaSrv(True).started

            if not scanStarted:
                print("An error has occured trying to start the scanner")
                return False
            
            # 2D grid map of the area is returned as 2D array
            map = self.getMapSrv(True).map

            # A tuple/array is returned for the next best position to discover [x, y]
            nextDest = self.getBestDestination(map)

            # Array of locations(in meters) is returned as a path to follow
            path = self.getPath(self.curLoc[1], nextDest, True)

            # Execute path
            reachedDest = self.executePath(path)

            # Maybe rotate vehicle slowly in its own axis to get more images
            # Then repeat loop again

            if reachedDest and self.mappingStatus == Status.RETURNING:
                self.mappingStatus = Status.COMPLETED
                return True
    
    def getBestDestination(self, grid):
        x,y = self.convertPointToGrid((self.curLoc[1].x, self.curLoc[1].y))
        nextDest = self.getNextDestinationSrv(grid, x, y).nextPoint

        if nextDest:
            return nextDest

        self.mappingStatus = Status.RETURNING       # ????? Add another type of status to show if the map completed its mission
        
        return self.initLoc[1]

    def getPath(self, start, end, endIsGrid=False):
        st = self.convertPointToGrid((start.x, start.y))
        if endIsGrid:       # if end point is given in grid indicies
            en = end
        else:
            en = self.convertPointToGrid((end.x, end.y))

        gridPath = self.getShortestPathSrv(map, st, en).path
        path = self.convertGridPathToPoints(gridPath)

        return path

    def executePath(self, path):
        for loc in path[1:]:
            done = self.gotoLocSrv(None, loc).done

            if not done:
                return False

        return True

    def abortMapping(self):
        # find shortest path between current location and intital location

        self.mappingStatus = Status.RETURNING       # ????? Add another type of status to show if the map completed its mission

        # Array of locations(in meters) is returned as a path to follow
        path = self.getPath(self.curLoc[1], self.initLoc[1])

        # Execute path
        reachedDest = self.executePath(path)

        if reachedDest and self.mappingStatus == Status.RETURNING:
            self.mappingStatus = Status.COMPLETED
            return True

        return False


    # Callbacks to topics
    # ======================================================================
    def speedListener(self, msg):
        self.speed = Speed(msg.left, msg.right, None, msg.expLeft, msg.expRight)

    def headingListener(self, msg):
        self.heading = msg.data

    def frontDistanceListener(self, msg):
        self.frontDistance = msg.distance

    def curLocListener(self, msg):
        self.curLoc = (time.time(), Location(msg.x, msg.y, msg.heading))


    # Misc
    # ======================================================================
    def convertPointToGrid(self, point):
        return math.ceil(point[0]/self.worldToMapRatio), math.ceil(point[1]/self.worldToMapRatio)

    def convertGridToPoint(self, grid):
        return grid[0]*self.worldToMapRatio, grid[1]/self.worldToMapRatio


    def convertGridPathToPoints(self, path):
        return [Location(loc.x*self.worldToMapRatio, loc.y*self.worldToMapRatio) for loc in path]


def main():
    rospy.init_node("MasterNode")
    # a = Master('192.168.0.1')
    a = Master('localhost')
    print("Master node initiated...")
    rospy.spin()

if __name__=='__main__':
    main()
