#!/usr/bin/env python3
import os
import numpy as np
from scipy import interpolate
import transforms3d
from math import sin, cos


def get_states_mocap(sock, mambo):
# get real-time states from phasespace, this function assumes only 1 rigid body working
    msg = sock.recv(4096)
    if msg:
        data = np.fromstring(msg, dtype=float)
        num_data_group = int(np.size(data)/8)
        data_all = data[-8*num_data_group:]
        data_for_csv = np.transpose(np.reshape(data_all, (num_data_group, 8)))
        #print('num_data_group')
        #print(num_data_group)

        data_for_LLC = data[-8:]
        # 2-D numpy array, 3 by 1, [px; py; pz], in meters
        posi_now = np.reshape(data_for_LLC[0:3], (-1, 1))
        # 1-D numpy array to list, [w, x, y, z]
        ori_quat = data_for_LLC[3:7].tolist()
    else:
        posi_now = np.array([[0.0], [0.0], [0.0]])
        ori_quat = [1.0, 0.0, 0.0, 0.0]
        print("Didn't receive the mocap data via socket")
        print("Land!")
        mambo.fly_direct(0, 0, 0, 0, 0.1)
        mambo.safe_land(5)
    return posi_now, ori_quat, mambo, data_for_csv


def compute_states(posi_now, posi_pre, ori_quat, yaw_prev, dt):
# based on function get_states_mocap(), compute some necessary states
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
    return posi_now, posi_pre, velo_now, Rot_Mat, yaw_now, pitch_now, roll_now, yaw_prev, velo_body, yaw_rate


def record_sysid(idx, states_history_mocap, states_history_cmd, yaw_rate_cmd, pitch_cmd, roll_cmd, vz_cmd, t0, data_for_csv):
# t0 is the machine timestamp

# record a numpy array for system id
    state_vector_mocap = data_for_csv
    state_vector_cmd = np.array([[yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd], [t0]], dtype=float)
    if idx == 0:
        states_history_mocap = state_vector_mocap
        states_history_cmd = state_vector_cmd
    else:
        states_history_mocap = np.concatenate((states_history_mocap, state_vector_mocap), axis=1)
        states_history_cmd = np.concatenate((states_history_cmd, state_vector_cmd), axis=1)
    return states_history_mocap, states_history_cmd


def record_states_plot(idx, t_now, posi_now, velo_now, yaw_now, pitch_now, roll_now, yaw_rate_cmd, pitch_cmd, roll_cmd, vz_cmd, time_plot, pv_plot, angle_and_cmd_plot):
# record the states for plotting
    time_plot.append(t_now)
    if idx == 0:
        pv_plot = np.concatenate((posi_now, velo_now), axis=0)
        angle_and_cmd_plot = np.array([[yaw_now], [pitch_now], [roll_now], [yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd]], dtype=float)
    else:
        pv_plot = np.concatenate((pv_plot, np.concatenate((posi_now, velo_now), axis=0)), axis=1)
        angle_and_cmd_plot = np.concatenate((angle_and_cmd_plot, np.array([[yaw_now], [pitch_now], [roll_now], [yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd]], dtype=float)), axis=1)
    return time_plot, pv_plot, angle_and_cmd_plot


def process_and_save_csv_sysid(states_history_mocap, states_history_cmd, Directory_sysid, t_start):
# save the data for system id
    time_name = time.strftime("%Y%m%d%H%M%S")
    FileName = Directory_sysid + time_name + '.csv'

    t_queue = states_history_mocap[-1, :]
    t_have = states_history_cmd[-1, :]
    t_relative = t_queue - t_start

    cmd_have = states_history_cmd[0:4, :]
    cmd_queue = interpolate_my(t_queue, t_have, cmd_have, 'cmd')

    states_history_uncompleted = np.concatenate((states_history_mocap, cmd_queue), axis=0)
    states_history = np.concatenate((states_history_uncompleted, np.reshape(t_relative,(1,np.size(t_relative)))), axis=0)

    np.savetxt(FileName, states_history, delimiter=",")

    # only for verify
    
    #FileName1 = Directory_sysid + time_name + 'test.csv'
    #t_re_temp = t_have - t_start
    #cmd_history_test = np.concatenate((states_history_cmd, np.reshape(t_re_temp,(1,np.size(t_re_temp)))), axis=0)
    #np.savetxt(FileName1, cmd_history_test, delimiter=",")



def interpolate_my(x_queue, x, y, interpolate_kind):
# if interpolate_kind == 'traj'
# interpolate the desired trajectory by time value
# x_queue is the interpolated point/points
# if x_queue is 1-D numpy array, then the output is 2-D numpy array
# if x_queue is a float, then the output is 2-D numpy array, but only has one column
# x is 1-D numpy array (1 by n), y is 2-D numpy array (m by n)
# default is linear interpolation
# when x_queue exceeds the range of x, function returns the boundary value of y

# if interpolate_kind == 'cmd'
# interpolate the commands by the machine time by Zero-Order Hold
# x_queue is the interpolated machine time
# assume x_queue is always 1-D numpy array, and the output is 2-D numpy array
# time before the first timestamp, commands are zeros
# time after the last timestamp, commands are the last ones.

    if interpolate_kind == 'traj':
        boundary = (y[:, 0], y[:, -1])
        f = interpolate.interp1d(x, y, kind='linear', bounds_error=False, fill_value=boundary)
        y_raw = f(x_queue)
        if isinstance(x_queue, float):
            y_queue = y_raw.reshape(y_raw.shape[0], -1)
        else:
            y_queue = y_raw

    elif interpolate_kind == 'cmd':
        boundary = (np.zeros(4), y[:, -1])
        f = interpolate.interp1d(x, y, kind='zero', bounds_error=False, fill_value=boundary)
        y_queue = f(x_queue)

    else:
        print("The interpolation type is wrong!")
        y_queue = []

    return y_queue


def LLC_PD(idx, posi_now, point_ref_0, point_ref_1, yaw_now, yaw_des, Rot_Mat, P_now, velo_body, yaw_rate, vz_max, dt):
#####################################################################
# control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.

    # PID control gains for Vertical Velocity Command
    # tune the height and yaw first, because they are decoupled!!!
    # control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.

    # height controller variables and gains
    # see below
    fwdfeedheight = 0.0           #forward feed coefficient 
    Kp_height = 0.0              #pid proportional gain
    # 1
    Ki_height = 0.0               #pid integral gain
    Kd_height = 0.0               #pid derivative gain
    #pitch/forward velocity controller gains
    fwdfeedpitch = 00.0           #forward feed coefficient 
    Kp_pitch = 20.0               #pid proportional gain 
    Ki_pitch = 0.0                #pid integral gain
    Kd_pitch = 30.0                #pid derivative gain
    # kd was 30
    #roll/lateral velocity controller gains
    fwdfeedroll = 00.0            #forward feed coefficient 
    Kp_roll = 38.0                 #pid proportional gain 
    Ki_roll = 0.0                 #pid integral gain
    Kd_roll = 38.0                 #pid derivative gain
    # kd was 38
    #yaw rate controller gains
    fwdfeedyaw = 20.0             #forward feed coefficient 
    Kp_psi = 40.0                 #pid proportional gain
    Ki_psi = 0.0                  #pid integral gain
    Kd_psi = 0.0                  #pid derivative gain
#####################################################################

    feedforward_yaw = -1.0 * (sin(yaw_now) - sin(yaw_des))
    yaw_proportion_term = feedforward_yaw - sin(yaw_rate)
    yaw_rate_cmd = Kp_psi*yaw_proportion_term + fwdfeedyaw*feedforward_yaw 

# change point_ref_0 to 1
    velo_des_body = np.dot(np.transpose(Rot_Mat), np.reshape(point_ref_1[3:6, 0], (-1, 1)))
    proportion_term = np.dot(np.transpose(Rot_Mat), np.reshape(point_ref_1[0:3, 0], (-1, 1)) - posi_now)
    deri_term = np.dot(np.transpose(Rot_Mat), np.reshape(point_ref_1[3:6, 0], (-1, 1))) - velo_body

    pitch_cmd = Kp_pitch*proportion_term[0, 0] + Kd_pitch*deri_term[0, 0] + fwdfeedpitch*velo_des_body[0, 0]
    roll_cmd = Kp_roll*proportion_term[2, 0] + Kd_roll*deri_term[2, 0] + fwdfeedroll*velo_des_body[2, 0]

    vz_cmd = point_ref_1[4, 0] / vz_max * 100.0 + Kp_height * (point_ref_1[1, 0] - posi_now[1, 0])

    if yaw_des == np.pi:
        yaw_rate_cmd = 1.0 * yaw_rate_cmd
    else:
        yaw_rate_cmd = -1.0 * yaw_rate_cmd

    return P_now, pitch_cmd, roll_cmd, vz_cmd, yaw_rate_cmd

