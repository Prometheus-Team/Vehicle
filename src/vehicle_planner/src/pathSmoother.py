# Adopted from Udacity AI for Robotics course PID lesson
# -----------

from copy import deepcopy

class SmoothPath:
    def __init__(self, weightData = 0.5, weightSmooth = 0.1, tolerance = 0.000001):
        self.weightData = weightData
        self.weightSmooth = weightSmooth
        self.tolerance = tolerance

    def copy(self, path):
        nPath = []
        for i in path:
            nPath.append([i[0], i[1]])

        return nPath

    def smoothen(self, path):
        # Make a deep copy of path into newpath
        newpath = self.copy(path)

        change = 100
        while change >= self.tolerance:
            change = 0
            for i in range(1,len(newpath)-1):
                for j in range(2):
                    upd = self.weightData * (path[i][j] - newpath[i][j]) + self.weightSmooth * (newpath[i-1][j] + newpath[i+1][j] - 2.0 * newpath[i][j])
                    newpath[i][j] += upd
                    change += abs(upd)

        return newpath

    # thank you to EnTerr for posting this on our discussion forum
    def printPaths(self, path,newpath):
        for old,new in zip(path,newpath):
            print('['+ ', '.join('%.3f'%x for x in old) + \
                '] -> ['+ ', '.join('%.3f'%x for x in new) +']')