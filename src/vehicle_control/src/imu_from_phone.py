import socket
from time import sleep
import math
import numpy
import pyrr
import json
import urllib.request
import time
from madgwickahrs import MadgwickAHRS
from quaternion import Quaternion

URL = "http://192.168.1.110:8080/sensors.json"
FREQ = 3 # Hertz
INTERVAL = 0.2




class IMU_Distance():
    def __init__(self,imu=0, master=None):
        self.INTERVAL = 0.2
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.index=0
        self.positionVectors=numpy.zeros(3)
        self.imu=imu
        self.currentAccelerations=numpy.zeros(3)
        self.previousAccelerations=numpy.zeros(3)
        self.currentQuaternion=""
        self.currentVelocities=numpy.zeros(3)
        self.previousVelocities=numpy.zeros(3)
        self.initialQuaternion=""
        self.previousTime=0.0
        self.previousMotion=0
        self.madgwickahrs=0
        self.recieveLoop()
    def recieveLoop(self):
        index = 0
        while True:
            response = urllib.request.urlopen(URL)
            recievedData=json.loads(response.read().decode())
            self.newUpdateStateFunction(recievedData)
            sleep(1/3)

    def updateState(self,out):
        #print(out)
        recievedQuaternion=out['rot_vector']['data'][1][1]
        timeDifference=0
        currentTime=float(out['rot_vector']['data'][1][0])
        if(self.previousTime!=0.0):
            timeDifference= (currentTime - self.previousTime) /1000
            self.previousTime=currentTime
        else:
            self.previousTime=currentTime
        
        print(str(timeDifference)+ " s")
        self.currentQuaternion=pyrr.quaternion.create(recievedQuaternion[0],recievedQuaternion[1],recievedQuaternion[2],recievedQuaternion[3]) 
    
        rotationMatrix=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentQuaternion)
        #print(out['rot_vector']['data'][1][0])
        #print(out['lin_accel']['data'][1][1])
        #print(out['accel']['data'][1][1])
        #print(self.currentVelocities)
        #print(rotationMatrix)
        linAcceleration=numpy.array(out['lin_accel']['data'][1][1])
        adjustedAccelerations=pyrr.matrix33.apply_to_vector(rotationMatrix, linAcceleration)

        #adjustedVelocities=self.currentVelocities + (adjustedAccelerations * timeDifference)+ ((adjustedAccelerations - self.currentAccelerations)*timeDifference *timeDifference * 0.5)
        adjustedVelocities= numpy.add(adjustedAccelerations,self.currentAccelerations) * timeDifference
        
        #self.positionVectors= self.positionVectors + (self.currentVelocities * timeDifference) + (adjustedAccelerations * (0.5 * timeDifference * timeDifference)) + ((adjustedAccelerations - self.currentAccelerations) * (timeDifference * timeDifference * timeDifference  ) / 6)
        
        self.positionVectors= numpy.add(self.positionVectors, numpy.array(adjustedVelocities * timeDifference))
        print(out['rot_vector']['data'])
        print()
        print(out['lin_accel']['data'])
        print()
        print("Adjusted Acceleration"+ str(adjustedAccelerations))
        print("Adjusted Velocities"+ str(adjustedVelocities))
        print("Distance " + str(self.positionVectors))
    
        self.currentVelocities=adjustedVelocities
        self.currentAccelerations= adjustedAccelerations

    def extractUsefulData(self,data):
        deserializedData=json.loads(data)
        print(deserializedData.encode("utf-8"))
        self.currentQuaternion=pyrr.quaternion.create() 
        #rotationMatrix=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentQuaternion)
        #vecotors=pyrr.matrix33.apply_to_vector(rotationMatrix, velocity_vector)

    def newUpdateStateFunction(self, data):
        rotationData=data['rot_vector']['data']
        linAccelerationData=data['lin_accel']['data']
        accelerometerData=data['accel']['data']
        magData=data['mag']['data']
        gyroData=data['gyro']['data']
        

        #print(linAccelerationData)
        for index,i in enumerate(rotationData,start=0):
            self.currentTime=i[0]
            timeDifference=(self.currentTime-self.previousTime)/1000
            
                
            self.currentQuaternion=pyrr.quaternion.create(i[1][0],i[1][1],i[1][2],i[1][3])
                
            
            if self.currentTime!=self.previousTime and self.previousTime!=0:
                
                if index < len(gyroData) and  index < len(accelerometerData) and  index < len(magData) :
                    self.madgwickahrs.update(gyroData[index][1],accelerometerData[index][1],magData[index][1],timeDifference)
                elif  index < len(gyroData) and  index < len(accelerometerData):
                    self.madgwickahrs.update_imu(gyroData[index][1],accelerometerData[index][1] ,timeDifference)
                else:
                    return
                a=pyrr.matrix33.create_from_inverse_of_quaternion(self.currentQuaternion)
                b= pyrr.matrix33.create_from_quaternion(self.initialQuaternion)
                rotationMatrix=a+b
                adjustedAccelerations=pyrr.matrix33.apply_to_vector(rotationMatrix, linAccelerationData[index][1])
                # for i in range(0,len(adjustedAccelerations)):
                #     if abs(adjustedAccelerations[i]) < 0.01:
                #         adjustedAccelerations[i]=0
                #adjustedVelocities= numpy.add(self.currentVelocities,numpy.add(adjustedAccelerations * timeDifference, numpy.array(numpy.add(adjustedAccelerations,self.currentAccelerations*-1)*timeDifference *timeDifference * 0.5)))
               

                


                #print("quaternion from madgwick"+str(self.madgwickahrs.quaternion._get_q()))
                #print("quaternion measured"+str(self.currentQuaternion))



                interm1=numpy.add(adjustedAccelerations, numpy.negative(self.currentAccelerations))/timeDifference
                interm2= interm1 * timeDifference *timeDifference * 0.5
                interm3=adjustedAccelerations * timeDifference
                interm4= numpy.add(interm2, interm3)

                
                adjustedVelocities= numpy.add(self.currentVelocities,interm4)
                # for i in range(0,len(adjustedVelocities)):
                #     if abs(adjustedVelocities[i]) < 0.01:
                #         adjustedVelocities[i]=0

                self.positionVectors= numpy.add(self.positionVectors, numpy.array(adjustedVelocities * timeDifference))
                self.previousTime=self.currentTime
                self.currentAccelerations=adjustedAccelerations
                self.currentVelocities=adjustedVelocities
                #print(str(timeDifference)+ " s")
                #print("Adjusted Acceleration"+ str(adjustedAccelerations))
                #print("unadjusted acceleration"+ str(data['accel']['data'][index][1]))
                #print("rotation vectors"+str(self.currentQuaternion))
                #print("madgwick vectors"+str(self.madgwickahrs.quaternion._get_q()))
                #print("Adjusted Velocities"+ str(adjustedVelocities))

                #print("Distance " + str(self.positionVectors))

                print("euler", Quaternion(self.currentQuaternion).to_euler123()[0]*180/math.pi)
                print()

            elif self.previousTime==0:
                self.previousTime=self.currentTime
                self.currentAccelerations=linAccelerationData[index][1]
                #self.madgwickahrs=MadgwickAHRS(quaternion=Quaternion(i[1][0],i[1][1],i[1][2],i[1][3]))
                self.madgwickahrs=MadgwickAHRS(quaternion=Quaternion(self.currentQuaternion))
                self.initialQuaternion=self.currentQuaternion
class imu():
    def __init__(self,master=None):
        self.f = open("sensors.json", "r", encoding="utf-8")
    def getData(self):
       return(self.f.read())




# IMU_Distance()