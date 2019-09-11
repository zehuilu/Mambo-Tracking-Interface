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
        data = data[-26:]
        # 2-D numpy array, 3 by 1, [px; py; pz], in meters
        posi_now = np.reshape(data[13:16], (-1, 1))
        velo_now = np.reshape(data[16:19], (-1, 1))
        # 1-D numpy array to list, [w, x, y, z]
        ori_quat = data[22:26].tolist()
        return posi_now, velo_now, ori_quat
    else:
        posi_now = np.array([[0.0], [0.0], [0.0]])
        velo_now = posi_now
        ori_quat = [1.0, 0.0, 0.0, 0.0]
        print("Didn't receive the mocap data via socket")
        return posi_now, velo_now, ori_quat


def prediction_kinetic(posi_now, velo_now, dt):
    # dt is the prediction time
    posi_next = posi_now + velo_now * dt
    return posi_next


if __name__ == '__main__':

####################################################################################
    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 9000        # Port to listen on (non-privileged ports are > 1023)
    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect the socket to the port where the server is listening
    server_address = (HOST, PORT)
    print("connecting to", server_address)
    sock.connect(server_address)
####################################################################################

    time.sleep(2.0)
    idx = 0
    dt = 0.5

    px_list = []
    py_list = []
    pz_list = []
    vx_list = []
    vy_list = []
    vz_list = []
    px_predic_list = []
    py_predic_list = []
    pz_predic_list = []
    time_list = []

    t_end = time.time()

    while idx < 20:
        t_start = time.time()
        t_now = t_start - t_end
        time_list.append(t_now)

        posi_now, velo_now, ori_quat = get_states(sock)
        posi_next = prediction_kinetic(posi_now, velo_now, dt)
        print("prediction completed")

        px_list.append(posi_now[0, 0])
        py_list.append(posi_now[1, 0])
        pz_list.append(posi_now[2, 0])

        vx_list.append(velo_now[0, 0])
        vy_list.append(velo_now[1, 0])
        vz_list.append(velo_now[2, 0])

        px_predic_list.append(posi_next[0, 0])
        py_predic_list.append(posi_next[1, 0])
        pz_predic_list.append(posi_next[2, 0])

        idx += 1
        time.sleep(dt)

    a = """
    px_list = px_list[1:]
    py_list = py_list[1:]
    pz_list = pz_list[1:]
    time_list = time_list[1:]

    px_predic_list = px_predic_list[:len(px_predic_list)-1]
    py_predic_list = py_predic_list[:len(py_predic_list)-1]
    pz_predic_list = pz_predic_list[:len(pz_predic_list)-1]
    """

    print(vx_list)
    print(vy_list)
    print(vz_list)

    # plot

    # plot 0, actual/desired 3-D trajectory
    fig_0 = plt.figure()
    ax0 = fig_0.gca(projection="3d", title="Trajectory")
    ax0.plot(pz_predic_list, px_predic_list, py_predic_list, color="blue", linestyle="-", label="Predicted Trajectory")
    ax0.plot(pz_list, px_list, py_list, color="red", linestyle="-", label="Actual Trajectory") 
    ax0.set_xlabel("Z Label")
    ax0.set_ylabel("X Label")
    ax0.set_zlabel("Y Label")
    X = np.array(pz_list)
    Y = np.array(px_list)
    Z = np.array(py_list)
    max_range = np.array([X.max()-X.min(), \
        Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
    mid_x = (X.max() + X.min()) * 0.5
    mid_y = (Y.max() + Y.min()) * 0.5
    mid_z = (Z.max() + Z.min()) * 0.5
    ax0.set_xlim(mid_x - max_range, mid_x + max_range)
    ax0.set_ylim(mid_y - max_range, mid_y + max_range)
    ax0.set_zlim(mid_z - max_range, mid_z + max_range)
    plt.legend(loc="upper left", shadow=False)
    plt.legend(loc="upper left", shadow=False)
    plt.draw()

    # plot 1, actual/predicted positions
    fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
    ax1.plot(time_list, px_list, color="red", linestyle="-", label="actual px")
    ax1.plot(time_list, px_predic_list, color="blue", linestyle="--", label="predicted px")
    ax1.legend(loc="upper left")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("px [m]")
    ax2.plot(time_list, py_list, color="red", linestyle="-", label="actual py")
    ax2.plot(time_list, py_predic_list, color="blue", linestyle="--", label="predicted py")
    ax2.legend(loc="upper left")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("py [m]")
    ax3.plot(time_list, pz_list, color="red", linestyle="-", label="actual pz")
    ax3.plot(time_list, pz_predic_list, color="blue", linestyle="--", label="predicted pz")
    ax3.legend(loc="upper left")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("pz [m]")
    plt.tight_layout()

    # plot 3, position tracking errors
    error_px = (np.array(px_list) - np.array(px_predic_list)).tolist()
    error_py = (np.array(py_list) - np.array(py_predic_list)).tolist()
    error_pz = (np.array(pz_list) - np.array(pz_predic_list)).tolist()
    fig_3, (ax7, ax8, ax9) = plt.subplots(3, 1)
    ax7.plot(time_list, error_px, color="red", linestyle="-", label="error px")
    ax7.legend(loc="upper left")
    ax7.set_xlabel("Time [s]")
    ax7.set_ylabel("position error [m]")
    ax8.plot(time_list, error_py, color="red", linestyle="-", label="error py")
    ax8.legend(loc="upper left")
    ax8.set_xlabel("Time [s]")
    ax8.set_ylabel("position error [m]")
    ax9.plot(time_list, error_pz, color="red", linestyle="-", label="error pz")
    ax9.legend(loc="upper left")
    ax9.set_xlabel("Time [s]")
    ax9.set_ylabel("position error [m]")
    plt.tight_layout()


    # plot 5, actual/desired velocities
    fig_5, (ax14, ax15, ax16) = plt.subplots(3, 1)
    ax14.plot(time_list, vx_list, color="red", linestyle="-", label="actual vx")
    ax14.legend(loc="upper left")
    ax14.set_xlabel("Time [s]")
    ax14.set_ylabel("vx [m/s]")
    ax15.plot(time_list, vy_list, color="red", linestyle="-", label="actual vy")
    ax15.legend(loc="upper left")
    ax15.set_xlabel("Time [s]")
    ax15.set_ylabel("vy [m/s]")
    ax16.plot(time_list, vz_list, color="red", linestyle="-", label="actual vz")
    ax16.legend(loc="upper left")
    ax16.set_xlabel("Time [s]")
    ax16.set_ylabel("vz [m/s]")
    plt.tight_layout()

    plt.show()
