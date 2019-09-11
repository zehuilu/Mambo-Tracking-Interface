import socket
import numpy as np
import time
import json

################################################
# Create a TCP/IP socket
sock_planner = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
HOST_planner = '127.0.0.2'  # Standard loopback interface address (localhost)
PORT_planner = 12000        # Port to listen on (non-privileged ports are > 1023)
server_address_planner = (HOST_planner, PORT_planner)

print("connecting to", server_address_planner)
sock_planner.connect(server_address_planner)
################################################


a = """
################################################
# Create a TCP/IP socket
sock_mocap = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
HOST_mocap = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT_mocap = 8000        # Port to listen on (non-privileged ports are > 1023)
server_address_mocap = (HOST_mocap, PORT_mocap)

print("connecting to", server_address_mocap)
sock_mocap.connect(server_address_mocap)
################################################
"""

px = []
py = []
pz = []
vx = []
vy = []
vz = []
ax = []
ay = []
az = []
time_list = []

time.sleep(1.0)

idx = -30
t_now = 0.0
#while True:
#while t_now < 3.0:
while idx < 30:
    msg_planner = sock_planner.recv(4096)
    #msg_mocap = sock_mocap.recv(4096)
    #if msg_planner and msg_mocap:
    if msg_planner:
        data_planner = msg_planner.decode()
        
        if idx > -30:
            data_planner = json.loads(data_planner)
            flag_append = data_planner.get("flag_append")
            print(data_planner["px"])
            print(flag_append)
        
        
        time.sleep(0.1)

    else:
        break
        print("Didn't receive the message")

    idx += 1
    print(idx)


a = """
# plot
# plot 1, desired positions
fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
ax1.plot(time_list, px, color="red", linestyle="-", label="desired px")
ax1.legend(loc="upper left")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("px [m]")
ax2.plot(time_list, py, color="red", linestyle="-", label="desired py")
ax2.legend(loc="upper left")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("py [m]")
ax3.plot(time_list, pz, color="red", linestyle="-", label="desired pz")
ax3.legend(loc="upper left")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("pz [m]")
plt.tight_layout()

# plot 5, desired velocities
fig_5, (ax14, ax15, ax16) = plt.subplots(3, 1)
ax14.plot(time_list, vx, color="red", linestyle="-", label="desired vx")
ax14.legend(loc="upper left")
ax14.set_xlabel("Time [s]")
ax14.set_ylabel("vx [m/s]")
ax15.plot(time_list, vy, color="red", linestyle="-", label="desired vy")
ax15.legend(loc="upper left")
ax15.set_xlabel("Time [s]")
ax15.set_ylabel("vy [m/s]")
ax16.plot(time_list, vz, color="red", linestyle="-", label="desired vz")
ax16.legend(loc="upper left")
ax16.set_xlabel("Time [s]")
ax16.set_ylabel("vz [m/s]")
plt.tight_layout()

plt.show()
"""