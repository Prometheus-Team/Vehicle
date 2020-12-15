import socket, time, threading
import numpy as np
# import cPickle as pickle
import pickle

def cmdSend():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('192.168.137.1', 9999))

        # s.bind(('192.168.0.103',5555))
    except:
        print("Socket not created")
    # s.sendall("GET /\r\n")
    # s.sendall(b"GET /sensors.json HTTP/1.1\r\nHost: webcode.me\r\nAccept: application/json\r\nConnection: open\r\n\r\n")
    while True:
        x = {
            'cmd':'systemCheck',
            'args':True
        }
        x = {
            'cmd':'startExplore',
            'args':{
                'left':9.5,
                'right':5,
                'up':3,
                'down':4
            }
        }
        x = {
            'cmd':'manualControl',
            'args':'forward'
        }
        # x = {
        #     'cmd':'manualControlChange',
        #     'args':True
        # }
        # s.sendall(pickle.dumps(x))
        # break
        # r = s.recv(1024)
        r = s.recv(62556)
        if r is not None:
            print(pickle.loads(r))
        break
        # if r is None:
        #     break
        # print(pickle.loads(r))


    s.close()

def sensorRead():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    except:
        print("Socket not created")

    s.bind(('', 8008))
    s.listen(5)
    print("Server started...")
    c, addr = s.accept()

    while True:
        # try:
        data = c.recv(62556)
        sensor = pickle.loads(data)
        print(len(sensor))
        print(sensor)

        # except Exception as e:
        #     print(e)
        #     break

    s.close()

def main():
    cmd = threading.Thread(target=cmdSend)

    sense = threading.Thread(target=sensorRead)

    cmd.start()
    cmd.join()
    # sense.start()
    # sense.join()

main()