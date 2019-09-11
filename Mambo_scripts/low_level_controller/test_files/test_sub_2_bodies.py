import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

import socket
import numpy as np
import transforms3d
import time
from math import degrees, radians, sin, cos, sqrt
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def get_states(sock):
    msg = sock.recv(4096)
    if msg:
        data = np.fromstring(msg, dtype=float)
        data = data[-14:]
        print(data)
        # 2-D numpy array, 3 by 1, [px; py; pz], in meters
        posi_now = np.reshape(data[0:3], (-1, 1))
        #posi_now = np.reshape(data[7:10], (-1, 1))
        # 1-D numpy array to list, [w, x, y, z]
        #ori_quat = data[10:].tolist()
        ori_quat = data[3:7].tolist()
        return posi_now, ori_quat
    else:
        posi_now = np.array([[0.0], [0.0], [0.0]])
        ori_quat = [1.0, 0.0, 0.0, 0.0]
        print("Didn't receive the mocap data via socket")
        return posi_now, ori_quat


if __name__ == '__main__':

    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 9000        # Port to listen on (non-privileged ports are > 1023)
    # you will need to change this to the address of your mambo
    # this is for Mambo_628236
    mamboAddr = "D0:3A:93:36:E6:21"

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect the socket to the port where the server is listening
    server_address = (HOST, PORT)
    print("connecting to", server_address)
    sock.connect(server_address)
    

#######################################################

    # initialization
    time_list = []
    yaw_list = []
    pitch_list = []
    roll_list = []
    px_list = []
    py_list = []
    pz_list = []
    vx_list = []
    vy_list = []
    vz_list = []
    yaw_des_list = []
    pitch_des_list = []
    roll_des_list = []
    yaw_cmd_list = []
    pitch_cmd_list = []
    roll_cmd_list = []
    vz_cmd_list = []
    px_actual_des_list = []
    py_actual_des_list = []
    pz_actual_des_list = []
    t_now = 0.0

    for i in [1,2,3,4,5]:
        posi_now, ori_quat = get_states(sock)
        time.sleep(0.1)


    idx = -20
    while t_now < 5.0:
        t0 = time.time()
        if idx == 0:
            t_start = t0


        # get states
        posi_now, ori_quat = get_states(sock)
        posi_pre = posi_now

        # current rotation matrix
        Rot_Mat = transforms3d.quaternions.quat2mat(ori_quat)
        # current euler angles in radians
        yaw_now, pitch_now, roll_now = transforms3d.euler.quat2euler(ori_quat, axes='syzx')

        if idx >= 0:
            t_now = t0 - t_start

        # record
        px_list.append(posi_now[0, 0])
        py_list.append(posi_now[1, 0])
        pz_list.append(posi_now[2, 0])

        yaw_list.append(yaw_now)
        pitch_list.append(pitch_now)
        roll_list.append(roll_now)
        time_list.append(t_now)


        time.sleep(0.1)
                
        
        t1 = time.time()
        dt = t1 - t0
        print("time interval for fly command")
        print(dt)
        idx += 1


    print(px_list)
    print(py_list)
    print(pz_list)

    # plot

    # plot 0, actual/desired 3-D trajectory
    fig_0 = plt.figure()
    ax0 = fig_0.gca(projection="3d", title="Trajectory")
    ax0.plot(pz_list, px_list, py_list, color="red", linestyle="-", label="Actual Trajectory") 
    ax0.plot([pz_list[0]], [px_list[0]], [py_list[0]], marker="s", label="Actual Origin") 
    ax0.plot([pz_list[-1]], [px_list[-1]], [py_list[-1]], marker="v", label="Actual Destination") 
    ax0.set_xlabel("Z Label")
    ax0.set_ylabel("X Label")
    ax0.set_zlabel("Y Label")
    X = np.array(pz_list)
    Y = np.array(px_list)
    Z = np.array(py_list)
    max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
    mid_x = (X.max() + X.min()) * 0.5
    mid_y = (Y.max() + Y.min()) * 0.5
    mid_z = (Z.max() + Z.min()) * 0.5
    ax0.set_xlim(mid_x - max_range, mid_x + max_range)
    ax0.set_ylim(mid_y - max_range, mid_y + max_range)
    ax0.set_zlim(mid_z - max_range, mid_z + max_range)
    plt.legend(loc="upper left", shadow=False)
    plt.legend(loc="upper left", shadow=False)
    plt.draw()

    # plot 1, actual/desired positions
    fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
    ax1.plot(time_list, px_list, color="red", linestyle="-", label="actual px")
    ax1.legend(loc="upper left")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("px [m]")
    ax2.plot(time_list, py_list, color="red", linestyle="-", label="actual py")
    ax2.legend(loc="upper left")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("py [m]")
    ax3.plot(time_list, pz_list, color="red", linestyle="-", label="actual pz")
    ax3.legend(loc="upper left")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("pz [m]")
    plt.tight_layout()

    # plot 2, actual euler angles
    fig_2, (ax4, ax5, ax6) = plt.subplots(3, 1)
    ax4.plot(time_list, yaw_list, color="red", linestyle="-", label="actual yaw")
    ax4.legend(loc="upper left")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Yaw [radian]")
    ax5.plot(time_list, pitch_list, color="red", linestyle="-", label="actual pitch")
    ax5.legend(loc="upper left")
    ax5.set_xlabel("Time [s]")
    ax5.set_ylabel("Pitch [radian]")
    ax6.plot(time_list, roll_list, color="red", linestyle="-", label="actual roll")
    ax6.legend(loc="upper left")
    ax6.set_xlabel("Time [s]")
    ax6.set_ylabel("Roll [radian]")
    plt.tight_layout()

    plt.show()

