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

import generate_spline_peak_speed as gen_spline

import csv
import glob
import os


if __name__ == '__main__':

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


    dt_traj = 0.10
    yaw_prev = 0.5 # non-zero initial value
    posi_pre = np.array([[0.0], [0.0], [0.0]])
    velo_now = np.array([[0.0], [0.0], [0.0]])
    dt = dt_traj
    t_now = 0.0


    if True:
        Directory = '/home/roahmlab/Mambo_scripts/combo_old/traj_lib/'
        BaseName = 'traj_'
        idx_traj = 1
        idx_csv = 1
        FileName = '/home/roahmlab/Mambo_scripts/combo_old/traj_lib/traj_0.csv'
        Z = np.genfromtxt(FileName, dtype=float, delimiter=',')
        print(Z)
        traj_ref = Z[0 : 6, 0 : 6]
        idx = -20
        print(traj_ref)
        while t_now < 3.0:
            t0 = time.time()
            if idx == 0:
                t_start = t0
            

            if idx < 0:
                # Do nothing
                time.sleep(0.1)

            else:
                t_now = t0 - t_start
                if idx % 5 == 0 and idx > 0:
                    csv_file_list = sorted(glob.glob("/home/roahmlab/Mambo_scripts/combo_old/traj_lib/*.csv"), key=os.path.getmtime)
                    if idx_csv < len(csv_file_list):
                        FileName = Directory + BaseName + str(idx_csv) + '.csv'
                        Z = np.genfromtxt(FileName, dtype=float, delimiter=',')
                        traj_ref = np.concatenate((traj_ref, Z[0 : 6, 1 : 6]), axis=1)
                        idx_csv += 1
                        idx_traj = 1
                        print("new csv file")
                    else:
                        if idx_traj < 6:
                            traj_ref = np.concatenate((traj_ref, Z[0 : 6, idx_traj*5+1 : (idx_traj+1)*5+1]), axis=1)
                            print("checkpoint 111")
                        else:
                            traj_temp = np.array([\
                            [Z[0,-1], Z[0,-1], Z[0,-1], Z[0,-1], Z[0,-1]], \
                            [Z[1,-1], Z[1,-1], Z[1,-1], Z[1,-1], Z[1,-1]], \
                            [Z[2,-1], Z[2,-1], Z[2,-1], Z[2,-1], Z[2,-1]], \
                            [Z[3,-1], Z[3,-1], Z[3,-1], Z[3,-1], Z[3,-1]], \
                            [Z[4,-1], Z[4,-1], Z[4,-1], Z[4,-1], Z[4,-1]], \
                            [Z[5,-1], Z[5,-1], Z[5,-1], Z[5,-1], Z[5,-1]]])
                            traj_ref = np.concatenate((traj_ref, traj_temp), axis=1)
                            print("checkpoint 222")

                        idx_traj += 1
            



                a = """
                # load the current and the next desired points
                # 2-D numpy array, 9 by 1, px, py, pz, vx, vy, vz, ax, ay, az
                point_ref_0 = gen_spline.do_interpolate(t_now, traj_ref, dt_traj)
                point_ref_1 = gen_spline.do_interpolate(t_now + dt_traj, traj_ref, dt_traj)
                
                px_actual_des_list.append(point_ref_0[0, 0])
                py_actual_des_list.append(point_ref_0[1, 0])
                pz_actual_des_list.append(point_ref_0[2, 0])
                """

                # record



                time.sleep(0.1)
                
            t1 = time.time()
            dt = t1 - t0
            print("time interval for fly command")
            print(dt)
            print("time now")
            print(t_now)
            print(traj_ref)
            idx += 1


        time_ref = (np.arange(traj_ref.shape[1]) * 0.1)
        print(traj_ref)

        print("the shape of traj_ref")
        print(traj_ref.shape)

        # plot

        # plot 0, actual/desired 3-D trajectory
        fig_0 = plt.figure()
        ax0 = fig_0.gca(projection="3d", title="Trajectory")
        ax0.plot(traj_ref[2, :], traj_ref[0, :], traj_ref[1, :], color="blue", linestyle="-", label="Reference Trajectory")
        ax0.plot([traj_ref[2, 0]], [traj_ref[0, 0]], [traj_ref[1, 0]], marker="D", label="Reference Origin")
        ax0.plot([traj_ref[2, -1]], [traj_ref[0, -1]], [traj_ref[1, -1]], marker="x", label="Reference Destination")
        ax0.set_xlabel("Z Label")
        ax0.set_ylabel("X Label")
        ax0.set_zlabel("Y Label")
        X = np.array(traj_ref[2, :])
        Y = np.array(traj_ref[0, :])
        Z = np.array(traj_ref[1, :])
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

        # plot 1, actual/desired positions
        fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(time_ref, traj_ref[0, :].tolist(), color="blue", linestyle="--", label="desired px")
        ax1.legend(loc="upper left")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("px [m]")
        ax2.plot(time_ref.tolist(), traj_ref[1, :].tolist(), color="blue", linestyle="--", label="desired py")
        ax2.legend(loc="upper left")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("py [m]")
        ax3.plot(time_ref.tolist(), traj_ref[2, :].tolist(), color="blue", linestyle="--", label="desired pz")
        ax3.legend(loc="upper left")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("pz [m]")
        plt.tight_layout()


        # plot 5, actual/desired velocities
        fig_5, (ax14, ax15, ax16) = plt.subplots(3, 1)
        ax14.plot(time_ref.tolist(), traj_ref[3, :].tolist(), color="blue", linestyle="--", label="desired vx")
        ax14.legend(loc="upper left")
        ax14.set_xlabel("Time [s]")
        ax14.set_ylabel("vx [m/s]")
        ax15.plot(time_ref.tolist(), traj_ref[4, :].tolist(), color="blue", linestyle="--", label="desired vy")
        ax15.legend(loc="upper left")
        ax15.set_xlabel("Time [s]")
        ax15.set_ylabel("vy [m/s]")
        ax16.plot(time_ref.tolist(), traj_ref[5, :].tolist(), color="blue", linestyle="--", label="desired vz")
        ax16.legend(loc="upper left")
        ax16.set_xlabel("Time [s]")
        ax16.set_ylabel("vz [m/s]")
        plt.tight_layout()

        plt.show()
