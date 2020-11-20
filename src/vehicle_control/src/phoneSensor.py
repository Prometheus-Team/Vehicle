from quaternion import Quaternion
import pyrr, urllib, json, time, math

URL = "http://192.168.1.103:8080/sensors.json"
FREQ = 20 # Hertz

def readSensor():
    while True:
        response = urllib.urlopen(URL)
        data=json.loads(response.read())
        i=data['rot_vector']['data'][-1]
        currentQuaternion=pyrr.quaternion.create(i[1][0],i[1][1],i[1][2],i[1][3])
        print("euler", Quaternion(currentQuaternion).to_euler123()[0]*180/math.pi)

readSensor()