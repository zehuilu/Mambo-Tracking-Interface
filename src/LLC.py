#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/lib')

from pyparrot.Minidrone import Mambo
import socket
import numpy as np
import time
from math import degrees, radians, sin, cos
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct

import helper_function as hf
import csv_helper_module as csv_helper


if __name__ == '__main__':
#######################################################


    # user define
    # if fly backward, choose pi, otherwise, choose 0.0
    yaw_des = 0.0 # in radians
    #yaw_des = np.pi # in radians


#######################################################
    # you will need to change this to the address of your mambo
    # Mambo_628236
    mamboAddr = "D0:3A:93:36:E6:21"
#######################################################
    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 9000        # Port to listen on (non-privileged ports are > 1023)

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect the socket to the port where the server is listening
    server_address = (HOST, PORT)
    print("connecting to", server_address)
    sock.connect(server_address)
    
    # make my mambo object
    # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
    mambo = Mambo(mamboAddr, use_wifi=False)
    print("trying to connect")
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)


    # Sending real-time positions and velocities to MATLAB via UDP
    HOST_matlab = '127.0.0.1'
    PORT_matlab = 11000
    # Connect the socket to the port where the server is listening
    server_address_matlab = (HOST_matlab, PORT_matlab)

    # Create a UDP socket
    sock_matlab = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#######################################################
    # define some variables
    Directory_sysid = os.getcwd() + '/sysid_data/'
    Directory_traj = os.getcwd() + '/traj_csv_files/'
    Directory_delete = os.getcwd() + '/traj_csv_files/*'

    tilt_max = radians(25.0) # in degrees to radians
    vz_max = 2.0 # in m/s
    yaw_rate_max = 3.14 # in radians/sec

    # remember: in mocap system, euler angles depends on the fixed global axes,
    # but for drone commands, euler angles depends on the current orientation
    # need to stablize yaw very quickly

    # PID terms
    P_now = np.array([[0.0], [0.0], [0.0], [0.0]])

    t_look_ahead = 0.2 # seconds

    dt_traj = 0.10
    yaw_prev = 0.5 # non-zero initial value
    posi_pre = np.array([[0.0], [0.0], [0.0]])
    velo_now = np.array([[0.0], [0.0], [0.0]])
    dt = dt_traj
    t_now = 0.0
    hover_flag = True
    hover_flag_pre = True

    states_history_mocap = 0.0
    states_history_cmd = 0.0

    time_plot = []
    pv_plot = 0.0
    angle_and_cmd_plot = 0.0

    csv_length_now = 1
    csv_length_pre = -1
    t_stop = 100.0 # is a large enough number

    csv_helper.remove_traj_ref_lib(Directory_delete)

# control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.
#######################################################
    if (success):
        # calibrate the Mambo
        mambo.flat_trim()
        print("calibrated the Mambo")
        # set the maximum tilt angle in degrees
        # tilt_max is in radians!!!
        result_set_tilt = mambo.set_max_tilt(degrees(tilt_max))
        print("set the maximum tilt angle")
        # set the maximum vertical speed in m/s
        result_set_vz = mambo.set_max_vertical_speed(vz_max)
        print("set the maximum vertical speed")
        while not (result_set_tilt and result_set_vz):
            print("Failed to set the maximum tilt angle and vz!")
        print("Setup successed!")
        # get the state information
        print("sleeping")
        mambo.smart_sleep(1)
        mambo.ask_for_state_update()
        mambo.smart_sleep(1)

        battery_0 = mambo.sensors.battery
        print("The battery percentage is ", battery_0)

        while battery_0 <= 62:
            print("The battery voltage is low!!!")

        mambo.safe_takeoff(5)
        print("taking off!")
        mambo.fly_direct(0, 0, 0, 0, 1.0)
        print("zero input first!")
#######################################################

        idx = 0

# Remember to change the total time!!!!!!!!!!!!!!!!!

        while t_now < t_stop:
            t0 = time.time()

            # get states
            # notice that the current function works only for one rigid body
            posi_now, ori_quat, mambo, data_for_csv = hf.get_states_mocap(sock, mambo)
            # compute some variables
            posi_now, posi_pre, velo_now, Rot_Mat, yaw_now, pitch_now, roll_now, yaw_prev, velo_body, yaw_rate = \
            hf.compute_states(posi_now, posi_pre, ori_quat, yaw_prev, dt)


            # send positions and velocities to MATLAB via UDP
            msg = struct.pack('dddddd', posi_now[0,0], posi_now[1,0], posi_now[2,0], velo_now[0,0], velo_now[1,0], velo_now[2,0])
            #data_test = struct.unpack('dddddd', msg)
            print("sending message to matlab")
            #print(msg)
            #print(data_test)
            sock_matlab.sendto(msg, server_address_matlab)


            traj_ref, T, hover_flag, csv_length_now = csv_helper.update_csv(Directory_traj)

            if not hover_flag:
                #if hover_flag != hover_flag_pre:
                if (hover_flag == False) & (hover_flag_pre == True):
                    t_start = t0
                    idx = 0

                t_now = t0 - t_start
                print("Total Time")
                print(t_now)

                if True:
                    if csv_length_now == csv_length_pre:
                        #t_stop = T[-1] - 1.9 # this is caused by a bug from RTD planner, Nov. 15, 2019
                        t_stop = T[-1]       # bug fixed, Nov. 21, 2019

                    csv_length_pre = csv_length_now

                    # load the current and the next desired points
                    # 2-D numpy array, 6 by 1, px, py, pz, vx, vy, vz
                    point_ref_0 = hf.interpolate_my(t_now, T, traj_ref, 'traj')
                    point_ref_1 = hf.interpolate_my(t_now + dt_traj + t_look_ahead, T, traj_ref, 'traj')

                    P_now, pitch_cmd, roll_cmd, vz_cmd, yaw_rate_cmd = hf.LLC_PD(idx, posi_now, point_ref_0, point_ref_1, yaw_now, yaw_des, Rot_Mat, P_now, velo_body, yaw_rate, vz_max, dt)

                    # record
                    states_history_mocap, states_history_cmd = hf.record_sysid(idx, states_history_mocap, states_history_cmd, yaw_rate_cmd, pitch_cmd, roll_cmd, vz_cmd, t0, data_for_csv)

                    # p, v, yaw, pitch, roll, yaw, pitch, roll, vz
                    time_plot, pv_plot, angle_and_cmd_plot = hf.record_states_plot(idx, t_now, posi_now, velo_now, yaw_now, pitch_now, roll_now, yaw_rate_cmd, pitch_cmd, roll_cmd, vz_cmd, time_plot, pv_plot, angle_and_cmd_plot)
                    # send commands
                    #mambo.smart_sleep(dt_traj)
                    mambo.fly_direct(roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd, dt_traj)
                else:
                    mambo.fly_direct(0.0, 0.0, 0.0, 0.0, dt_traj)
                
            else:
                print("Haven't generated csv file in MATLAB")
                print("You can run planner in MATLAB to generate the csv files after 1 second")
                mambo.fly_direct(0.0, 0.0, 0.0, 0.0, dt_traj)

            hover_flag_pre = hover_flag
            t1 = time.time()
            dt = t1 - t0
            print("time interval for fly command")
            print(dt)
            print("current time")
            print(t_now)
            idx += 1


        mambo.fly_direct(0, 0, 0, 0, 1.0)
    
        print("landing")
        mambo.safe_land(5)

        print("disconnect")
        mambo.disconnect()


        hf.process_and_save_csv_sysid(states_history_mocap, states_history_cmd, Directory_sysid, t_start)

        battery_1 = mambo.sensors.battery
        print("The battery percentage is ", battery_1)

        battery_used = battery_0 - battery_1
        print("The used battery percentage is", battery_used)


#######################################################
        # plot

        # load the data
        px_list = pv_plot[0, :].tolist()
        py_list = pv_plot[1, :].tolist()
        pz_list = pv_plot[2, :].tolist()
        vx_list = pv_plot[3, :].tolist()
        vy_list = pv_plot[4, :].tolist()
        vz_list = pv_plot[5, :].tolist()
        yaw_list = angle_and_cmd_plot[0, :].tolist()
        pitch_list = angle_and_cmd_plot[1, :].tolist()
        roll_list = angle_and_cmd_plot[2, :].tolist()
        yaw_cmd_list = angle_and_cmd_plot[3, :].tolist()
        pitch_cmd_list = angle_and_cmd_plot[4, :].tolist()
        roll_cmd_list = angle_and_cmd_plot[5, :].tolist()
        vz_cmd_list = angle_and_cmd_plot[6, :].tolist()

        # plot 0, actual/desired 3-D trajectory
        fig_0 = plt.figure()
        ax0 = fig_0.gca(projection="3d", title="Trajectory")
        ax0.plot(traj_ref[2, :], traj_ref[0, :], traj_ref[1, :], color="blue", linestyle="-", label="Reference Trajectory")
        ax0.plot([traj_ref[2, 0]], [traj_ref[0, 0]], [traj_ref[1, 0]], marker="D", label="Reference Origin")
        ax0.plot([traj_ref[2, -1]], [traj_ref[0, -1]], [traj_ref[1, -1]], marker="x", label="Reference Destination")
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
        ax1.plot(time_plot, px_list, color="red", linestyle="-", label="actual px")
        ax1.plot(T.tolist(), traj_ref[0, :].tolist(), color="blue", linestyle="--", label="desired px")
        ax1.legend(loc="upper left")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("px [m]")
        ax2.plot(time_plot, py_list, color="red", linestyle="-", label="actual py")
        ax2.plot(T.tolist(), traj_ref[1, :].tolist(), color="blue", linestyle="--", label="desired py")
        ax2.legend(loc="upper left")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("py [m]")
        ax3.plot(time_plot, pz_list, color="red", linestyle="-", label="actual pz")
        ax3.plot(T.tolist(), traj_ref[2, :].tolist(), color="blue", linestyle="--", label="desired pz")
        ax3.legend(loc="upper left")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("pz [m]")
        plt.tight_layout()

        # plot 2, actual euler angles
        fig_2, (ax4, ax5, ax6) = plt.subplots(3, 1)
        ax4.plot(time_plot, yaw_list, color="red", linestyle="-", label="actual yaw")
        ax4.legend(loc="upper left")
        ax4.set_xlabel("Time [s]")
        ax4.set_ylabel("Yaw [radian]")
        ax5.plot(time_plot, pitch_list, color="red", linestyle="-", label="actual pitch")
        ax5.legend(loc="upper left")
        ax5.set_xlabel("Time [s]")
        ax5.set_ylabel("Pitch [radian]")
        ax6.plot(time_plot, roll_list, color="red", linestyle="-", label="actual roll")
        ax6.legend(loc="upper left")
        ax6.set_xlabel("Time [s]")
        ax6.set_ylabel("Roll [radian]")
        plt.tight_layout()

        
        # plot 3, position tracking errors

        p_actual_des = hf.interpolate_my(time_plot, T, traj_ref, 'traj')[0 : 3, :]
        error = p_actual_des - pv_plot[0 : 3, :]

        fig_3, (ax7, ax8, ax9) = plt.subplots(3, 1)
        ax7.plot(time_plot, error[0, :].tolist(), color="red", linestyle="-", label="error px")
        ax7.legend(loc="upper left")
        ax7.set_xlabel("Time [s]")
        ax7.set_ylabel("position error [m]")
        ax8.plot(time_plot, error[1, :].tolist(), color="red", linestyle="-", label="error py")
        ax8.legend(loc="upper left")
        ax8.set_xlabel("Time [s]")
        ax8.set_ylabel("position error [m]")
        ax9.plot(time_plot, error[2, :].tolist(), color="red", linestyle="-", label="error pz")
        ax9.legend(loc="upper left")
        ax9.set_xlabel("Time [s]")
        ax9.set_ylabel("position error [m]")
        plt.tight_layout()
        

        # plot 4, inputs which were sent to the Mambo
        fig_4, ((ax10, ax11), (ax12, ax13)) = plt.subplots(2, 2)
        ax10.plot(time_plot, yaw_cmd_list, color="red", linestyle="-", label="yaw rate command")
        ax10.legend(loc="upper left")
        ax10.set_xlabel("Time [s]")
        ax10.set_ylabel("yaw rate command [percentage]")
        ax11.plot(time_plot, pitch_cmd_list, color="red", linestyle="-", label="pitch command")
        ax11.legend(loc="upper left")
        ax11.set_xlabel("Time [s]")
        ax11.set_ylabel("pitch command [percentage]")
        ax12.plot(time_plot, roll_cmd_list, color="red", linestyle="-", label="roll command")
        ax12.legend(loc="upper left")
        ax12.set_xlabel("Time [s]")
        ax12.set_ylabel("roll command [percentage]")
        ax13.plot(time_plot, vz_cmd_list, color="red", linestyle="-", label="vertical speed command")
        ax13.legend(loc="upper left")
        ax13.set_xlabel("Time [s]")
        ax13.set_ylabel("vertical speed command [percentage]")
        plt.tight_layout()

        # plot 5, actual/desired velocities
        fig_5, (ax14, ax15, ax16) = plt.subplots(3, 1)
        ax14.plot(time_plot, vx_list, color="red", linestyle="-", label="actual vx")
        ax14.plot(T.tolist(), traj_ref[3, :].tolist(), color="blue", linestyle="--", label="desired vx")
        ax14.legend(loc="upper left")
        ax14.set_xlabel("Time [s]")
        ax14.set_ylabel("vx [m/s]")
        ax15.plot(time_plot, vy_list, color="red", linestyle="-", label="actual vy")
        ax15.plot(T.tolist(), traj_ref[4, :].tolist(), color="blue", linestyle="--", label="desired vy")
        ax15.legend(loc="upper left")
        ax15.set_xlabel("Time [s]")
        ax15.set_ylabel("vy [m/s]")
        ax16.plot(time_plot, vz_list, color="red", linestyle="-", label="actual vz")
        ax16.plot(T.tolist(), traj_ref[5, :].tolist(), color="blue", linestyle="--", label="desired vz")
        ax16.legend(loc="upper left")
        ax16.set_xlabel("Time [s]")
        ax16.set_ylabel("vz [m/s]")
        plt.tight_layout()

        # plot 6, actual euler angles
        yaw_cal = np.array(yaw_list)
        yaw_cal = np.sin(yaw_cal).tolist()
        fig_6, (ax17, ax18, ax19) = plt.subplots(3, 1)
        ax17.plot(time_plot, yaw_cal, color="red", linestyle="-", label="actual sin(yaw)")
        ax17.legend(loc="upper left")
        ax17.set_xlabel("Time [s]")
        ax17.set_ylabel("sin(yaw)")
        ax18.plot(time_plot, pitch_list, color="red", linestyle="-", label="actual pitch")
        ax18.legend(loc="upper left")
        ax18.set_xlabel("Time [s]")
        ax18.set_ylabel("Pitch [radian]")
        ax19.plot(time_plot, roll_list, color="red", linestyle="-", label="actual roll")
        ax19.legend(loc="upper left")
        ax19.set_xlabel("Time [s]")
        ax19.set_ylabel("Roll [radian]")
        plt.tight_layout()

        plt.show()

    else:
        print("Mambo connection failed!")