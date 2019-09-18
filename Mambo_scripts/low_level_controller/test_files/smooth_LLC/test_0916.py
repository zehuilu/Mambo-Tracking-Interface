import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

import socket
import numpy as np
import time
from math import degrees, radians, sin, cos, sqrt
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import helper_function as hf


if __name__ == '__main__':

#######################################################
    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 9000        # Port to listen on (non-privileged ports are > 1023)
    # you will need to change this to the address of your mambo
    # this is for Mambo_628236

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect the socket to the port where the server is listening
    server_address = (HOST, PORT)
    print("connecting to", server_address)
    sock.connect(server_address)
    
#######################################################
    # define some variables
    tilt_max = radians(25.0) # in degrees to radians
    vz_max = 2.0 # in m/s
    yaw_rate_max = 3.14 # in radians/sec

    # remember: in mocap system, euler angles depends on the fixed global axes,
    # but for drone commands, euler angles depends on the current orientation
    # need to stablize yaw very quickly
    yaw_des = 0.0 # in radians

    # PID terms
    P_now = np.array([[0.0], [0.0], [0.0], [0.0]])

    dt_traj = 0.10
    yaw_prev = 0.5 # non-zero initial value
    posi_pre = np.array([[0.0], [0.0], [0.0]])
    velo_now = np.array([[0.0], [0.0], [0.0]])
    dt = dt_traj
    t_now = 0.0

    states_history = 0.0
    time_plot = []
    pv_plot = 0.0
    angle_and_cmd_plot = 0.0


# control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.
#######################################################
    if True:
#######################################################

        idx = -20

# Remember to change the total time!!!!!!!!!!!!!!!!!

        while True:
            t0 = time.time()
            if idx == 0:
                t_start = t0

            # get states
            # notice that the current function works only for one rigid body
            posi_now, velo_now, ori_quat = hf.get_states_mocap_wo_mambo(sock)
            print(velo_now)
    
            if idx < 0:
                # Do nothing
                time.sleep(dt_traj)

            else:
                t_now = t0 - t_start
                print("Total Time")
                print(t_now)
              # send commands
                time.sleep(dt_traj)
                
            t1 = time.time()
            dt = t1 - t0
            print("time interval for fly command")
            print(dt)
            idx += 1
