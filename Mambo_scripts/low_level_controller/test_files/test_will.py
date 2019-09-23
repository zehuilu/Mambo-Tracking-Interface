import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

from pyparrot.Minidrone import Mambo
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
import helper_function as hf


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
    
    # make my mambo object
    # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
    mambo = Mambo(mamboAddr, use_wifi=False)
    print("trying to connect")
    success = mambo.connect(num_retries=3)
    print("connected: %s" % success)


#######################################################
    Directory = '/home/roahmlab/Mambo_scripts/combo_old/traj_lib/'
    BaseName = 'traj_'
    idx_traj = 1
    idx_csv = 1
    FileName = '/home/roahmlab/Mambo_scripts/combo_old/traj_lib/traj_0.csv'
    
    traj_ref = hf.import_csv(FileName)[:, 0 : 6]


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

    tilt_max = radians(25.0) # in degrees to radians
    vz_max = 2.0 # in m/s
    yaw_rate_max = 3.14 # in radians/sec

    # some variables
    # PID terms
    P_now = np.array([[0.0], [0.0], [0.0], [0.0]])
    P_pre = np.array([[0.0], [0.0], [0.0], [0.0]])
    I_now = np.array([[0.0], [0.0], [0.0], [0.0]])
    D_now = np.array([[0.0], [0.0], [0.0], [0.0]])

    dt_traj = 0.10
    yaw_prev = 0.5 # non-zero initial value
    posi_pre = np.array([[0.0], [0.0], [0.0]])
    velo_now = np.array([[0.0], [0.0], [0.0]])
    dt = dt_traj
    t_now = 0.0


    # remember: in mocap system, euler angles depends on the fixed global axes,
    # but for drone commands, euler angles depends on the current orientation
    # need to stablize yaw very quickly
    yaw_des = 0.0 # in radians


    # PID control gains for Vertical Velocity Command

    # tune the height and yaw first, because they are decoupled!!!
    # control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.

    # height controller variables and gains
    # see below
    fwdfeedheight = 0.0           #forward feed coefficient 
    Kp_height = 10.0              #pid proportional gain
    Ki_height = 0.0               #pid integral gain
    Kd_height = 0.0               #pid derivative gain
    #pitch/forward velocity controller gains
    fwdfeedpitch = 130.0          #forward feed coefficient 
    Kp_pitch = 18.0               #pid proportional gain 
    Ki_pitch = 0.0                #pid integral gain
    Kd_pitch = 8.0                #pid derivative gain
    #roll/lateral velocity controller gains
    fwdfeedroll = 120.0            #forward feed coefficient 
    Kp_roll = 17.0                 #pid proportional gain 
    Ki_roll = 0.0                  #pid integral gain
    Kd_roll = 5.0                  #pid derivative gain
    #yaw rate controller gains
    fwdfeedyaw = 20.0             #forward feed coefficient 
    Kp_psi = 40.0                 #pid proportional gain
    Ki_psi = 0.0                  #pid integral gain
    Kd_psi = 0.0                  #pid derivative gain
#####################################################################

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

        print("taking off!")
        mambo.safe_takeoff(5)

        idx = -20
        while t_now < 3.0:
            t0 = time.time()
            if idx == 0:
                t_start = t0


            # get states
            # notice that this function works only for one rigid body
            posi_now, ori_quat, mambo = hf.get_states(sock, mambo)
            # compute the velocity
            velo_now = (posi_now - posi_pre) / dt
            posi_pre = posi_now

            # current rotation matrix
            Rot_Mat = transforms3d.quaternions.quat2mat(ori_quat)
            # current euler angles in radians
            yaw_now, pitch_now, roll_now = transforms3d.euler.quat2euler(ori_quat, axes='syzx')
            yaw_rate = (yaw_now - yaw_prev) / dt
            yaw_prev = yaw_now

            # Rotate into Body-fixed Frame
            velo_body = np.dot(np.linalg.pinv(Rot_Mat), velo_now)

            if idx < 0:
                # Do nothing
                mambo.smart_sleep(dt_traj)

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
                # we could use scipy.interpolate.interp1d


                # load the current and the next desired points
                # 2-D numpy array, 9 by 1, px, py, pz, vx, vy, vz, ax, ay, az
                point_ref_0 = gen_spline.do_interpolate(t_now, traj_ref, dt_traj)
                point_ref_1 = gen_spline.do_interpolate(t_now + dt_traj, traj_ref, dt_traj)
                
                px_actual_des_list.append(point_ref_0[0, 0])
                py_actual_des_list.append(point_ref_0[1, 0])
                pz_actual_des_list.append(point_ref_0[2, 0])

                # position feedforward vector in global frame
                feedforward_posi = -1.0 * (posi_now - np.reshape(point_ref_1[0:3, 0], (-1, 1))) # 3 by 1
                feedforward_yaw = -1.0 * (yaw_now - yaw_des)

                # position feedforward vector in body frame
                feedforward_posi_body = np.dot(np.linalg.pinv(Rot_Mat), feedforward_posi)

                P_pre = P_now
                P_now = np.vstack((feedforward_posi_body - velo_body, feedforward_yaw - yaw_rate))
                D_now = (P_now - P_pre) / dt
                

                pitch_cmd = Kp_pitch*P_now[0, 0] + Kd_pitch*D_now[0, 0] + fwdfeedpitch*feedforward_posi_body[0, 0]
                roll_cmd = Kp_roll*P_now[2, 0] + Kd_roll*D_now[2, 0] + fwdfeedroll*feedforward_posi_body[2, 0]
                vz_cmd = point_ref_1[4, 0] / vz_max * 100.0 + Kp_height * (point_ref_0[1, 0] - posi_now[1, 0])
                yaw_rate_cmd = Kp_psi*P_now[3, 0] + fwdfeedyaw*feedforward_yaw 

########################################
                yaw_rate_cmd = -1.0 * yaw_rate_cmd
########################################

                # record
                px_list.append(posi_now[0, 0])
                py_list.append(posi_now[1, 0])
                pz_list.append(posi_now[2, 0])

                vx_list.append(velo_now[0, 0])
                vy_list.append(velo_now[1, 0])
                vz_list.append(velo_now[2, 0])

                yaw_list.append(yaw_now)
                pitch_list.append(pitch_now)
                roll_list.append(roll_now)
                time_list.append(t_now)

                yaw_cmd_list.append(yaw_rate_cmd)
                pitch_cmd_list.append(pitch_cmd)
                roll_cmd_list.append(roll_cmd)
                vz_cmd_list.append(vz_cmd)


                #mambo.smart_sleep(dt_traj)
                mambo.fly_direct(roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd, dt_traj)
                
                
            t1 = time.time()
            dt = t1 - t0
            print("time interval for fly command")
            print(dt)
            idx += 1


        mambo.fly_direct(0, 0, 0, 0, dt_traj)         
    
        print("landing")
        mambo.safe_land(5)

        print("disconnect")
        mambo.disconnect()
        time_ref = (np.arange(traj_ref.shape[1]) * 0.1)

        # plot

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
        ax1.plot(time_list, px_list, color="red", linestyle="-", label="actual px")
        ax1.plot(time_ref.tolist(), traj_ref[0, :].tolist(), color="blue", linestyle="--", label="desired px")
        ax1.legend(loc="upper left")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("px [m]")
        ax2.plot(time_list, py_list, color="red", linestyle="-", label="actual py")
        ax2.plot(time_ref.tolist(), traj_ref[1, :].tolist(), color="blue", linestyle="--", label="desired py")
        ax2.legend(loc="upper left")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("py [m]")
        ax3.plot(time_list, pz_list, color="red", linestyle="-", label="actual pz")
        ax3.plot(time_ref.tolist(), traj_ref[2, :].tolist(), color="blue", linestyle="--", label="desired pz")
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

        a = '''
        # plot 3, position tracking errors
        error_px = (np.array([px_actual_des_list]) - np.array([px_list])).tolist()
        error_py = (np.array([py_actual_des_list]) - np.array([py_list])).tolist()
        error_pz = (np.array([pz_actual_des_list]) - np.array([pz_list])).tolist()
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
        '''

        # plot 4, inputs which were sent to the Mambo
        fig_4, ((ax10, ax11), (ax12, ax13)) = plt.subplots(2, 2)
        ax10.plot(time_list, yaw_cmd_list, color="red", linestyle="-", label="yaw rate command")
        ax10.legend(loc="upper left")
        ax10.set_xlabel("Time [s]")
        ax10.set_ylabel("yaw rate command [percentage]")
        ax11.plot(time_list, pitch_cmd_list, color="red", linestyle="-", label="pitch command")
        ax11.legend(loc="upper left")
        ax11.set_xlabel("Time [s]")
        ax11.set_ylabel("pitch command [percentage]")
        ax12.plot(time_list, roll_cmd_list, color="red", linestyle="-", label="roll command")
        ax12.legend(loc="upper left")
        ax12.set_xlabel("Time [s]")
        ax12.set_ylabel("roll command [percentage]")
        ax13.plot(time_list, vz_cmd_list, color="red", linestyle="-", label="vertical speed command")
        ax13.legend(loc="upper left")
        ax13.set_xlabel("Time [s]")
        ax13.set_ylabel("vertical speed command [percentage]")
        plt.tight_layout()

        # plot 5, actual/desired velocities
        fig_5, (ax14, ax15, ax16) = plt.subplots(3, 1)
        ax14.plot(time_list, vx_list, color="red", linestyle="-", label="actual vx")
        ax14.plot(time_ref.tolist(), traj_ref[3, :].tolist(), color="blue", linestyle="--", label="desired vx")
        ax14.legend(loc="upper left")
        ax14.set_xlabel("Time [s]")
        ax14.set_ylabel("vx [m/s]")
        ax15.plot(time_list, vy_list, color="red", linestyle="-", label="actual vy")
        ax15.plot(time_ref.tolist(), traj_ref[4, :].tolist(), color="blue", linestyle="--", label="desired vy")
        ax15.legend(loc="upper left")
        ax15.set_xlabel("Time [s]")
        ax15.set_ylabel("vy [m/s]")
        ax16.plot(time_list, vz_list, color="red", linestyle="-", label="actual vz")
        ax16.plot(time_ref.tolist(), traj_ref[5, :].tolist(), color="blue", linestyle="--", label="desired vz")
        ax16.legend(loc="upper left")
        ax16.set_xlabel("Time [s]")
        ax16.set_ylabel("vz [m/s]")
        plt.tight_layout()

        plt.show()

    else:
        print("Failed the Mambo connection!")