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
px_list = []
py_list = []
pz_list = []

t_now = 0.0


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect the socket to the port where the server is listening
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 9000        # Port to listen on (non-privileged ports are > 1023)
server_address = (HOST, PORT)
print("connecting to", server_address)
sock.connect(server_address)

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
            time.sleep(0.5)
        else:
            t_now = t0 - t_start
            data = np.fromstring(msg, dtype=float)
            data = data[-7:]

            posi_now = data[0:3]
            # w, x, y, z
            ori_quat = data[-4:].tolist()

            Rot_Mat = transforms3d.quaternions.quat2mat(ori_quat)
            
            yaw, pitch, roll = transforms3d.euler.quat2euler(ori_quat, axes='syzx')
            
            print("posi_now")
            print(posi_now)
            print("ori_quat")
            print(ori_quat)


            print(Rot_Mat)

            #px_list.append(px)
            #py_list.append(py)
            #pz_list.append(pz)
            yaw_list.append(yaw)
            pitch_list.append(pitch)
            roll_list.append(roll)
            time_list.append(t_now)
            time.sleep(0.1)
                

        t1 = time.time()
        dt = t1 - t0
        print("time interval for fly command")
        print(dt)
        idx += 1

    else:
        break
        print("Didn't receive the motion capture message")



"""

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


fig_2, (ax4, ax5, ax6) = plt.subplots(3, 1)
ax4.plot(time_list, px_list, color="red", linestyle="-", label="px")
ax4.legend(loc="lower right")
ax4.set_xlabel("Time [s]")
ax4.set_ylabel("px [mm]")
ax5.plot(time_list, py_list, color="red", linestyle="-", label="py")
ax5.legend(loc="lower right")
ax5.set_xlabel("Time [s]")
ax5.set_ylabel("py [mm]")
ax6.plot(time_list, pz_list, color="red", linestyle="-", label="pz")
ax6.legend(loc="lower right")
ax6.set_xlabel("Time [s]")
ax6.set_ylabel("pz [mm]")
plt.tight_layout()

plt.show()


"""
