import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

from pyparrot.Minidrone import Mambo
import socket
import numpy as np
import transforms3d
import time
from math import degrees, radians

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# initialization
time_list = []
yaw_list = []
pitch_list = []
roll_list = []
t_now = 0.0


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect the socket to the port where the server is listening
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 9000        # Port to listen on (non-privileged ports are > 1023)
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
    # calibrate the Mambo
    mambo.flat_trim()
    print("calibrated the Mambo")

    # set the maximum tilt angle in degrees
    mambo.set_max_tilt(25.0)
    print("set the maximum tilt angle")

    # set the maximum vertical speed in m/s
    mambo.set_max_vertical_speed(2.0)
    print("set the maximum vertical speed")

    # get the state information
    print("sleeping")
    mambo.smart_sleep(2)
    mambo.ask_for_state_update()
    mambo.smart_sleep(2)

    print("taking off!")
    mambo.safe_takeoff(5)

    idx = -10
    while t_now < 1.0:
        t0 = time.time()
        if idx == 0:
            t_start = t0

        msg = sock.recv(4096)
        if msg:
            if idx < 0:
                # Do nothing
                a = 1
                mambo.smart_sleep(0.1)
            else:
                t_now = t0 - t_start
                data = np.fromstring(msg, dtype=float)
                data = data[-6:]
                yaw = degrees(data[3])
                pitch = degrees(data[4])
                roll = degrees(data[5])

                yaw_list.append(yaw)
                pitch_list.append(pitch)
                roll_list.append(roll)
                time_list.append(t_now)


                #mambo.smart_sleep(0.1)
                mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.1)
                
                

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

    mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=0, duration=0.5)

    print("landing")
    mambo.safe_land(5)

    print("disconnect")
    mambo.disconnect()


    # plot
    fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
    ax1.plot(time_list, yaw_list, color="red", linestyle="-", label="yaw")
    ax1.legend(loc="lower right")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Yaw [degree]")
    ax2.plot(time_list, pitch_list, color="red", linestyle="-", label="pitch")
    ax2.legend(loc="lower right")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Pitch [degree]")
    ax3.plot(time_list, roll_list, color="red", linestyle="-", label="roll")
    ax3.legend(loc="lower right")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Roll [degree]")
    plt.tight_layout()

    plt.show()