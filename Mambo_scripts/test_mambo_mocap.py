
from pyparrot.Minidrone import Mambo
import socket
import numpy as np
import transforms3d
import time
from math import degrees, radians



# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect the socket to the port where the server is listening
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 8000        # Port to listen on (non-privileged ports are > 1023)
server_address = (HOST, PORT)
print("connecting to", server_address)
sock.connect(server_address)



# you will need to change this to the address of YOUR mambo
# this is for Mambo_628236
mamboAddr = "D0:3A:93:36:E6:21"
# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):
    # get the state information
    print("sleeping")
    mambo.smart_sleep(2)
    mambo.ask_for_state_update()
    mambo.smart_sleep(2)

    print("taking off!")
    mambo.safe_takeoff(5)

    idx = -10
    while idx < 8:
        t0 = time.time()

        msg = sock.recv(4096)
        if msg:
            if idx < 0:
                # Do nothing
                a = 1
            else:
                #mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)
                data = np.fromstring(msg, dtype=float)
                data = data[-6:]
                yaw = degrees(data[3])
                pitch = degrees(data[4])
                roll = degrees(data[5])
                print(yaw)
                print(pitch)
                print(roll)
                #print(data)
                mambo.smart_sleep(0.2)



            t1 = time.time()
            dt = t1 - t0
            print("time interval for fly command")
            print(dt)
            idx += 1

        else:
            break
            print("Didn't receive the motion capture message")
            print("Land!")
            mambo.safe_land(5)




    


    print("landing")
    mambo.safe_land(5)

    print("disconnect")
    mambo.disconnect()
