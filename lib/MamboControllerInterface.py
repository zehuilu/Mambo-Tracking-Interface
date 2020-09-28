#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/lib')
import socket
import time
import json
import struct
import transforms3d
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyparrot.Minidrone import Mambo
from math import degrees, radians, sin, cos
import numpy as np

import csv_helper_module as csv_helper
from interpolate_traj import interpolate_traj


class MamboControllerInterface(object):
    def __init__(self, config_file_name, mocap_string):
        """
        Constructor
        """

        # Read the configuration from the json file
        json_file = open(config_file_name)
        self.config_data = json.load(json_file)
        self.mocap_string = mocap_string
        self.flag_tuning_LLC = bool(self.config_data["FLAG_TUNING_LLC"])

        # desired yaw angle, if fly backward, choose pi; otherwise, choose 0.0, in radians
        self.yaw_des = float(self.config_data["LOW_LEVEL_CONTROLLER"]["YAW_DES"])

        # make my mambo object
        # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
    
        self.mambo = Mambo(self.config_data["MAMBO"]["BLUETOOTH_ADDRESS"], use_wifi=False)
        self.flag_mambo_connection = self.mambo.connect(num_retries=3)
        print("Mambo connected: %s" % self.flag_mambo_connection)

        # Create a TCP/IP socket
        self.sock_states = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        server_address_states = (self.config_data[self.mocap_string]["IP_STATES_ESTIMATION"], int(self.config_data[self.mocap_string]["PORT_STATES_ESTIMATION"]))
        print("connecting to", server_address_states)
        self.sock_states.connect(server_address_states)
        # maximum bytes received from TCP socket
        self.data_bytes_max = int(self.config_data[self.mocap_string]["DATA_BYTES_MAX"])
        # how many numbers the TCP socket is sending
        self.data_number_integer = int(self.config_data[self.mocap_string]["DATA_NUMBERS_STATES_ESTIMATION"])

        # Sending real-time positions and velocities to MATLAB via UDP
        # Connect the socket to the port where the server is listening
        self.server_address_matlab = (self.config_data["RTD_MATLAB"]["IP_MATLAB"], int(self.config_data["RTD_MATLAB"]["PORT_MATLAB"]))
        # Create a UDP socket
        self.sock_matlab = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # define the path for system id and csv trajectories
        self.directory_sysid = os.getcwd() + self.config_data["DIRECTORY_SYSID"]
        self.directory_traj = os.getcwd() + self.config_data["DIRECTORY_TRAJ"]
        self.directory_delete = os.getcwd() + self.config_data["DIRECTORY_TRAJ"] + "*"

        # define max tilt angles, vertical velocity, and yaw rate
        self.tilt_max = float(self.config_data["MAMBO"]["TILT_MAX"]) # in degrees
        self.vz_max = float(self.config_data["MAMBO"]["VERTICAL_SPEED_MAX"]) # in m/s
        self.yaw_rate_max = float(self.config_data["MAMBO"]["YAW_RATE_MAX"]) # in radians/sec

        # initialize some variables

        # initialize the current position and yaw angle
        self.t_look_ahead = float(self.config_data["LOW_LEVEL_CONTROLLER"]["TIME_LOOK_AHEAD"])
        self.dt_traj = float(self.config_data["LOW_LEVEL_CONTROLLER"]["TIME_STEP"])
        # previous yaw angle, random non-zero initial value
        self.yaw_prev = 0.5
        self.posi_pre = np.array([[0.0], [0.0], [0.0]])
        self.velo_now = np.array([[0.0], [0.0], [0.0]])

        self.posi_now = np.array([[0.0], [0.0], [0.0]])
        self.ori_quat = [1.0, 0.0, 0.0, 0.0]
        self.Rot_Mat = np.identity(3)
        self.velo_body = np.array([[0.0], [0.0], [0.0]])
        self.yaw_now = self.yaw_des
        self.pitch_now = 0.0
        self.roll_now = 0.0
        self.yaw_rate = 0.0


        self.idx_iter = 0
        self.dt = self.dt_traj
        self.t_now = 0.0
        self.hover_flag = True
        self.hover_flag_pre = True

        self.states_history_mocap = 0.0
        self.states_history_cmd = 0.0

        self.time_plot = []
        self.pv_plot = 0.0
        self.angle_and_cmd_plot = 0.0

        self.csv_length_now = 1
        self.csv_length_pre = -1
        self.t_stop = 100.0 # is a large enough number

        # load gains for PID controller
        # height controller gains
        self.fwdfeedheight = float(self.config_data["LOW_LEVEL_CONTROLLER"]["FWDFEED_HEIGHT"])
        self.Kp_height = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KP_HEIGHT"])
        self.Ki_height = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KI_HEIGHT"])
        self.Kd_height = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KD_HEIGHT"])
        #pitch/forward velocity controller gains
        self.fwdfeedpitch = float(self.config_data["LOW_LEVEL_CONTROLLER"]["FWDFEED_PITCH"])
        self.Kp_pitch = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KP_PITCH"])
        self.Ki_pitch = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KI_PITCH"])
        self.Kd_pitch = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KD_PITCH"])
        #roll/lateral velocity controller gains
        self.fwdfeedroll = float(self.config_data["LOW_LEVEL_CONTROLLER"]["FWDFEED_ROLL"])
        self.Kp_roll = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KP_ROLL"])
        self.Ki_roll = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KI_ROLL"])
        self.Kd_roll = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KD_ROLL"])
        #yaw rate controller gains
        self.fwdfeedyaw = float(self.config_data["LOW_LEVEL_CONTROLLER"]["FWDFEED_YAW"])
        self.Kp_psi = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KP_YAW"])
        self.Ki_psi = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KI_YAW"])
        self.Kd_psi = float(self.config_data["LOW_LEVEL_CONTROLLER"]["KD_YAW"])

        # if flag==1, don't remove current csv files for tuning LLC
        # if flag==0, remove all the current file under this directory
        if not self.flag_tuning_LLC:
            csv_helper.remove_traj_ref_lib(self.directory_delete)


    def run_LLC(self):

        if not (self.flag_mambo_connection):
            print("Mambo connection failed!")
        else:
            # calibrate the Mambo
            self.mambo.flat_trim()
            print("calibrated the Mambo")
            # set the maximum tilt angle in degrees
            # tilt_max is in radians!!!
            result_set_tilt = self.mambo.set_max_tilt(self.tilt_max)
            print("set the maximum tilt angle")
            # set the maximum vertical speed in m/s
            result_set_vz = self.mambo.set_max_vertical_speed(self.vz_max)
            print("set the maximum vertical speed")
            if not (result_set_tilt and result_set_vz):
                raise Exception("Failed to set the maximum tilt angle and vz!")
            print("Setup successed!")
            # get the state information
            print("sleeping")
            self.mambo.smart_sleep(0.1)
            self.mambo.ask_for_state_update()
            # self.mambo.smart_sleep(0.5)

            self.battery_ini = self.mambo.sensors.battery
            print("The battery percentage is ", self.battery_ini)

            if self.battery_ini <= 60:
                raise Exception("The battery voltage is low!!!")

            self.mambo.safe_takeoff(5)
            print("taking off!")
            self.mambo.fly_direct(0, 0, 0, 0, 1.0)
            print("zero input first!")


            # Remember to change the total time!
            while self.t_now < self.t_stop:
                t0 = time.time()

                # get states
                # notice that the current function works only for one rigid body
                data_for_csv = self.get_states_mocap()
                # compute some variables
                self.compute_states()

                # send positions and velocities to MATLAB via UDP
                msg = struct.pack('dddddd', self.posi_now[0,0], self.posi_now[1,0], self.posi_now[2,0], self.velo_now[0,0], self.velo_now[1,0], self.velo_now[2,0])
                #data_test = struct.unpack('dddddd', msg)
                print("sending message to matlab")
                self.sock_matlab.sendto(msg, self.server_address_matlab)

                traj_ref, T, self.hover_flag, self.csv_length_now = csv_helper.update_csv(self.directory_traj)

                if not self.hover_flag:
                    if (self.hover_flag == False) & (self.hover_flag_pre == True):
                        t_start = t0
                        self.idx_iter = 0

                    self.t_now = t0 - t_start
                    print("Total Time")
                    print(self.t_now)

                    if True:
                        if self.csv_length_now == self.csv_length_pre:
                            self.t_stop = T[-1]

                        self.csv_length_pre = self.csv_length_now

                        # load the current and the next desired points
                        # 2-D numpy array, 6 by 1, px, py, pz, vx, vy, vz
                        point_ref_0 = interpolate_traj(self.t_now, T, traj_ref, 'traj')
                        point_ref_1 = interpolate_traj(self.t_now + self.dt_traj + self.t_look_ahead, T, traj_ref, 'traj')

                        roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd = self.PID_controller(point_ref_0, point_ref_1)

                        # record
                        self.record_sysid(roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd, t0, data_for_csv)

                        # p, v, yaw, pitch, roll, yaw, pitch, roll, vz
                        self.record_states_plot(roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd)

                        # send commands
                        #mambo.smart_sleep(self.dt_traj)
                        self.mambo.fly_direct(roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd, self.dt_traj)
                    else:
                        # this part is for debugging
                        self.mambo.fly_direct(0.0, 0.0, 0.0, 0.0, self.dt_traj)
                    
                else:
                    print("Haven't generated csv file in MATLAB")
                    print("You can run planner in MATLAB to generate the csv files after 1 second")
                    self.mambo.fly_direct(0.0, 0.0, 0.0, 0.0, self.dt_traj)

                self.hover_flag_pre = self.hover_flag
                t1 = time.time()
                self.dt = t1 - t0
                print("time interval for fly command")
                print(self.dt)
                print("current time")
                print(self.t_now)
                self.idx_iter += 1

            # after the iterations(trajectory) completes
            self.mambo.fly_direct(0, 0, 0, 0, 1.0)
            print("landing")
            self.mambo.safe_land(5)
            print("disconnect")
            self.mambo.disconnect()

            # save csv file
            self.process_and_save_csv_sysid(t_start)

            battery_after = self.mambo.sensors.battery
            print("The battery percentage is ", battery_after)
            print("The used battery percentage is", self.battery_ini - battery_after)

            # plot
            if bool(self.config_data["FLAG_PLOT"]):
                self.visuaslize_result(traj_ref, T)


    def get_states_mocap(self):
        # """Get real-time states from mocap system."""

        msg = self.sock_states.recv(self.data_bytes_max)
        if msg:
            data = np.frombuffer(msg, dtype=float)
            num_data_group = int(np.size(data)/self.data_number_integer)
            data_all = data[-self.data_number_integer*num_data_group:]
            data_for_csv = np.transpose(np.reshape(data_all, (num_data_group, self.data_number_integer)))

            data_for_LLC = data[-self.data_number_integer:]
            # 2-D numpy array, 3 by 1, [px; py; pz], in meters
            self.posi_now = np.reshape(data_for_LLC[0:3], (-1, 1))
            # 1-D numpy array to list, [w, x, y, z]
            self.ori_quat = data_for_LLC[3:self.data_number_integer-1].tolist()
        else:
            self.posi_now = np.array([[0.0], [0.0], [0.0]])
            self.ori_quat = [1.0, 0.0, 0.0, 0.0]
            print("Didn't receive the mocap data via socket")
            print("Land!")
            self.mambo.fly_direct(0, 0, 0, 0, 0.1)
            self.mambo.safe_land(5)
        return data_for_csv


    def compute_states(self):
    # based on function get_states_mocap(), compute some necessary states
        # compute the velocity
        self.velo_now = (self.posi_now - self.posi_pre) / self.dt
        self.posi_pre = self.posi_now

        # current rotation matrix
        self.Rot_Mat = transforms3d.quaternions.quat2mat(self.ori_quat)
        # current euler angles in radians
        self.yaw_now, self.pitch_now, self.roll_now = transforms3d.euler.quat2euler(self.ori_quat, axes='syzx')
        self.yaw_rate = (self.yaw_now - self.yaw_prev) / self.dt
        self.yaw_prev = self.yaw_now

        # Rotate into Body-fixed Frame
        self.velo_body = np.dot(np.linalg.pinv(self.Rot_Mat), self.velo_now)


    def PID_controller(self, point_ref_0, point_ref_1):
        # control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.

        # PID control gains for Vertical Velocity Command
        # tune the height and yaw first, because they are decoupled!!!
        # control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.

        feedforward_yaw = -1.0 * (sin(self.yaw_now) - sin(self.yaw_des))
        yaw_proportion_term = feedforward_yaw - sin(self.yaw_rate)
        yaw_rate_cmd = self.Kp_psi*yaw_proportion_term + self.fwdfeedyaw*feedforward_yaw 

        velo_des_body = np.dot(np.transpose(self.Rot_Mat), np.reshape(point_ref_1[3:6, 0], (-1, 1)))
        proportion_term = np.dot(np.transpose(self.Rot_Mat), np.reshape(point_ref_1[0:3, 0], (-1, 1)) - self.posi_now)
        deri_term = np.dot(np.transpose(self.Rot_Mat), np.reshape(point_ref_1[3:6, 0], (-1, 1))) - self.velo_body

        pitch_cmd = self.Kp_pitch*proportion_term[0, 0] + self.Kd_pitch*deri_term[0, 0] + self.fwdfeedpitch*velo_des_body[0, 0]
        roll_cmd = self.Kp_roll*proportion_term[2, 0] + self.Kd_roll*deri_term[2, 0] + self.fwdfeedroll*velo_des_body[2, 0]

        vz_cmd = point_ref_1[4, 0] / self.vz_max * 100.0 + self.Kp_height * (point_ref_1[1, 0] - self.posi_now[1, 0])

        if self.yaw_des == np.pi:
            yaw_rate_cmd = 1.0 * yaw_rate_cmd
        else:
            yaw_rate_cmd = -1.0 * yaw_rate_cmd

        return roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd


    def record_sysid(self, roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd, t0, data_for_csv):
    # t0 is the machine timestamp
    # record a numpy array for system id

        if self.idx_iter == 0:
            self.states_history_mocap = data_for_csv
            self.states_history_cmd = np.array([[yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd], [t0]], dtype=float)
        else:
            self.states_history_mocap = np.concatenate((self.states_history_mocap, data_for_csv), axis=1)
            self.states_history_cmd = np.concatenate((self.states_history_cmd, \
                np.array([[yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd], [t0]], dtype=float)), axis=1)


    def record_states_plot(self, roll_cmd, pitch_cmd, yaw_rate_cmd, vz_cmd):
    # record the states for plotting
        self.time_plot.append(self.t_now)
        if self.idx_iter == 0:
            self.pv_plot = np.concatenate((self.posi_now, self.velo_now), axis=0)
            self.angle_and_cmd_plot = np.array([[self.yaw_now], [self.pitch_now], [self.roll_now], [yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd]], dtype=float)
        else:
            self.pv_plot = np.concatenate((self.pv_plot, np.concatenate((self.posi_now, self.velo_now), axis=0)), axis=1)
            self.angle_and_cmd_plot = np.concatenate((self.angle_and_cmd_plot, np.array([[self.yaw_now], [self.pitch_now], [self.roll_now], [yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd]], dtype=float)), axis=1)


    def process_and_save_csv_sysid(self, t_start):
        # save the data for system id
        time_name = time.strftime("%Y%m%d%H%M%S")
        FileName = self.directory_sysid + time_name + '.csv'

        t_queue = self.states_history_mocap[-1, :]
        t_have = self.states_history_cmd[-1, :]
        t_relative = t_queue - t_start

        cmd_have = self.states_history_cmd[0:4, :]
        cmd_queue = interpolate_traj(t_queue, t_have, cmd_have, 'cmd')

        states_history_uncompleted = np.concatenate((self.states_history_mocap, cmd_queue), axis=0)
        states_history = np.concatenate((states_history_uncompleted, np.reshape(t_relative,(1,np.size(t_relative)))), axis=0)

        np.savetxt(FileName, states_history, delimiter=",")

        # only for debugging
        #FileName1 = Directory_sysid + time_name + 'test.csv'
        #t_re_temp = t_have - t_start
        #cmd_history_test = np.concatenate((states_history_cmd, np.reshape(t_re_temp,(1,np.size(t_re_temp)))), axis=0)
        #np.savetxt(FileName1, cmd_history_test, delimiter=",")


    def visuaslize_result(self, traj_ref, T):
        # load the data
        px_list = self.pv_plot[0, :].tolist()
        py_list = self.pv_plot[1, :].tolist()
        pz_list = self.pv_plot[2, :].tolist()
        vx_list = self.pv_plot[3, :].tolist()
        vy_list = self.pv_plot[4, :].tolist()
        vz_list = self.pv_plot[5, :].tolist()
        yaw_list = self.angle_and_cmd_plot[0, :].tolist()
        pitch_list = self.angle_and_cmd_plot[1, :].tolist()
        roll_list = self.angle_and_cmd_plot[2, :].tolist()
        yaw_cmd_list = self.angle_and_cmd_plot[3, :].tolist()
        pitch_cmd_list = self.angle_and_cmd_plot[4, :].tolist()
        roll_cmd_list = self.angle_and_cmd_plot[5, :].tolist()
        vz_cmd_list = self.angle_and_cmd_plot[6, :].tolist()

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
        fig_1.suptitle("Actual & desired positions")
        ax1.plot(self.time_plot, px_list, color="red", linestyle="-", label="actual px")
        ax1.plot(T.tolist(), traj_ref[0, :].tolist(), color="blue", linestyle="--", label="desired px")
        ax1.legend(loc="upper left")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("px [m]")
        ax2.plot(self.time_plot, py_list, color="red", linestyle="-", label="actual py")
        ax2.plot(T.tolist(), traj_ref[1, :].tolist(), color="blue", linestyle="--", label="desired py")
        ax2.legend(loc="upper left")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("py [m]")
        ax3.plot(self.time_plot, pz_list, color="red", linestyle="-", label="actual pz")
        ax3.plot(T.tolist(), traj_ref[2, :].tolist(), color="blue", linestyle="--", label="desired pz")
        ax3.legend(loc="upper left")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("pz [m]")
        plt.tight_layout()

        # plot 2, actual euler angles
        fig_2, (ax4, ax5, ax6) = plt.subplots(3, 1)
        fig_2.suptitle("Actual attitude")
        ax4.plot(self.time_plot, yaw_list, color="red", linestyle="-", label="actual yaw")
        ax4.legend(loc="upper left")
        ax4.set_xlabel("Time [s]")
        ax4.set_ylabel("Yaw [radian]")
        ax5.plot(self.time_plot, pitch_list, color="red", linestyle="-", label="actual pitch")
        ax5.legend(loc="upper left")
        ax5.set_xlabel("Time [s]")
        ax5.set_ylabel("Pitch [radian]")
        ax6.plot(self.time_plot, roll_list, color="red", linestyle="-", label="actual roll")
        ax6.legend(loc="upper left")
        ax6.set_xlabel("Time [s]")
        ax6.set_ylabel("Roll [radian]")
        plt.tight_layout()

        
        # plot 3, position tracking errors
        p_actual_des = interpolate_traj(self.time_plot, T, traj_ref, 'traj')[0 : 3, :]
        error = p_actual_des - self.pv_plot[0 : 3, :]

        fig_3, (ax7, ax8, ax9) = plt.subplots(3, 1)
        fig_3.suptitle("Tracking errors in position")
        ax7.plot(self.time_plot, error[0, :].tolist(), color="red", linestyle="-", label="error px")
        ax7.legend(loc="upper left")
        ax7.set_xlabel("Time [s]")
        ax7.set_ylabel("position error [m]")
        ax8.plot(self.time_plot, error[1, :].tolist(), color="red", linestyle="-", label="error py")
        ax8.legend(loc="upper left")
        ax8.set_xlabel("Time [s]")
        ax8.set_ylabel("position error [m]")
        ax9.plot(self.time_plot, error[2, :].tolist(), color="red", linestyle="-", label="error pz")
        ax9.legend(loc="upper left")
        ax9.set_xlabel("Time [s]")
        ax9.set_ylabel("position error [m]")
        plt.tight_layout()
        

        # plot 4, inputs which were sent to the Mambo
        fig_4, ((ax10, ax11), (ax12, ax13)) = plt.subplots(2, 2)
        fig_4.suptitle("Control commands")
        ax10.plot(self.time_plot, yaw_cmd_list, color="red", linestyle="-", label="yaw rate command")
        ax10.legend(loc="upper left")
        ax10.set_xlabel("Time [s]")
        ax10.set_ylabel("yaw rate command [percentage]")
        ax11.plot(self.time_plot, pitch_cmd_list, color="red", linestyle="-", label="pitch command")
        ax11.legend(loc="upper left")
        ax11.set_xlabel("Time [s]")
        ax11.set_ylabel("pitch command [percentage]")
        ax12.plot(self.time_plot, roll_cmd_list, color="red", linestyle="-", label="roll command")
        ax12.legend(loc="upper left")
        ax12.set_xlabel("Time [s]")
        ax12.set_ylabel("roll command [percentage]")
        ax13.plot(self.time_plot, vz_cmd_list, color="red", linestyle="-", label="vertical speed command")
        ax13.legend(loc="upper left")
        ax13.set_xlabel("Time [s]")
        ax13.set_ylabel("vertical speed command [percentage]")
        plt.tight_layout()

        # plot 5, actual/desired velocities
        fig_5, (ax14, ax15, ax16) = plt.subplots(3, 1)
        fig_5.suptitle("Actual & desired velocities")
        ax14.plot(self.time_plot, vx_list, color="red", linestyle="-", label="actual vx")
        ax14.plot(T.tolist(), traj_ref[3, :].tolist(), color="blue", linestyle="--", label="desired vx")
        ax14.legend(loc="upper left")
        ax14.set_xlabel("Time [s]")
        ax14.set_ylabel("vx [m/s]")
        ax15.plot(self.time_plot, vy_list, color="red", linestyle="-", label="actual vy")
        ax15.plot(T.tolist(), traj_ref[4, :].tolist(), color="blue", linestyle="--", label="desired vy")
        ax15.legend(loc="upper left")
        ax15.set_xlabel("Time [s]")
        ax15.set_ylabel("vy [m/s]")
        ax16.plot(self.time_plot, vz_list, color="red", linestyle="-", label="actual vz")
        ax16.plot(T.tolist(), traj_ref[5, :].tolist(), color="blue", linestyle="--", label="desired vz")
        ax16.legend(loc="upper left")
        ax16.set_xlabel("Time [s]")
        ax16.set_ylabel("vz [m/s]")
        plt.tight_layout()

        # plot 6, actual euler angles
        yaw_cal = np.array(yaw_list)
        yaw_cal = np.sin(yaw_cal).tolist()
        fig_6, (ax17, ax18, ax19) = plt.subplots(3, 1)
        fig_6.suptitle("Actual attitude with sin(yaw)")
        ax17.plot(self.time_plot, yaw_cal, color="red", linestyle="-", label="actual sin(yaw)")
        ax17.legend(loc="upper left")
        ax17.set_xlabel("Time [s]")
        ax17.set_ylabel("sin(yaw)")
        ax18.plot(self.time_plot, pitch_list, color="red", linestyle="-", label="actual pitch")
        ax18.legend(loc="upper left")
        ax18.set_xlabel("Time [s]")
        ax18.set_ylabel("Pitch [radian]")
        ax19.plot(self.time_plot, roll_list, color="red", linestyle="-", label="actual roll")
        ax19.legend(loc="upper left")
        ax19.set_xlabel("Time [s]")
        ax19.set_ylabel("Roll [radian]")
        plt.tight_layout()

        plt.show()


if __name__ == "__main__":
    config_file_name = "config.json"
    mocap_string = "QUALISYS"

    Controller = MamboControllerInterface(config_file_name, mocap_string)
    Controller.run_LLC()
    