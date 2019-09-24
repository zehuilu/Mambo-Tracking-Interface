# -*- coding: UTF-8 -*-
import numpy as np
from scipy import interpolate
import transforms3d
import os
import glob
import csv
import time
from math import sin, cos


def remove_traj_ref_lib(Directory):
    files = glob.glob(Directory)
    for f in files:
        os.remove(f)


def get_states_mocap(sock, mambo):
# get real-time states from phasespace, this function assumes only 1 rigid body working
    msg = sock.recv(4096)
    if msg:
        data = np.fromstring(msg, dtype=float)
        data = data[-7:]
        # 2-D numpy array, 3 by 1, [px; py; pz], in meters
        posi_now = np.reshape(data[0:3], (-1, 1))
        # 1-D numpy array to list, [w, x, y, z]
        ori_quat = data[-4:].tolist()
    else:
        posi_now = np.array([[0.0], [0.0], [0.0]])
        ori_quat = [1.0, 0.0, 0.0, 0.0]
        print("Didn't receive the mocap data via socket")
        print("Land!")
        mambo.fly_direct(0, 0, 0, 0, 0.1)
        mambo.safe_land(5)
    return posi_now, ori_quat, mambo


def get_states_mocap_wo_mambo(sock):
# get real-time states from phasespace, this function assumes only 1 rigid body working
    msg = sock.recv(4096)
    if msg:
        data = np.fromstring(msg, dtype=float)
        data = data[-10:]
        # 2-D numpy array, 3 by 1, [px; py; pz], in meters
        posi_now = np.reshape(data[0:3], (-1, 1))
        velo_now = np.reshape(data[3:6], (-1, 1))
        # 1-D numpy array to list, [w, x, y, z]
        ori_quat = data[-4:].tolist()
    else:
        posi_now = np.array([[0.0], [0.0], [0.0]])
        velo_now = np.array([[0.0], [0.0], [0.0]])
        ori_quat = [1.0, 0.0, 0.0, 0.0]
    return posi_now, velo_now, ori_quat


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


def import_csv(FileName):
# import csv file from a file folder
# only export desired positions and velocities
    Z = np.genfromtxt(FileName, dtype=float, delimiter=',')
    print("checkpoint for import_csv")
    print(Z)
    T = Z[0, :]
    traj_ref = Z[1:7, :]
    return traj_ref, T


def update_csv(Directory):
# update the desired trajectory from csv file folder in 10 Hz
# assume traj_ref has the whole time trajectory information
# assume traj_ref only has positions and velocities
    csv_file_list = sorted(glob.glob(Directory + '*.csv'), key=os.path.getmtime)
    if not csv_file_list:
        hover_flag = True
        traj_ref = np.zeros((6, 2), dtype=float)
        T = np.array([0.0, 1.0], dtype=float)
    else:
        FileName = csv_file_list[-1]
        traj_ref, T = import_csv(FileName)
        hover_flag = False
    return traj_ref, T, hover_flag


def record_sysid(idx, states_history, t_now, posi_now, yaw_now, pitch_now, roll_now, yaw_rate_cmd, pitch_cmd, roll_cmd, vz_cmd):
# record a numpy array for system id
    state_vector = np.array([[t_now],[posi_now[0, 0]],[posi_now[1, 0]],[posi_now[2, 0]], [yaw_now], [pitch_now], [roll_now], [yaw_rate_cmd], [pitch_cmd], [roll_cmd], [vz_cmd]], dtype=float)
    if idx == 0:
        states_history = state_vector
    else:
        states_history = np.concatenate((states_history, state_vector), axis=1)
    return states_history


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


def save_csv_sysid(states_history, Directory_sysid):
# save the data for system id
    time_name = time.strftime("%Y%m%d%H%M%S")
    FileName = Directory_sysid + time_name + '.csv'
    np.savetxt(FileName, states_history, delimiter=",")


def interpolate_traj(x_queue, x, y):
# interpolate the desired trajectory by time value
# x_queue is the interpolated point/points
# if x_queue is 1-D numpy array, then the output is 2-D numpy array
# if x_queue is a float, then the output is 2-D numpy array, but only has one column
# x is 1-D numpy array (1 by n), y is 2-D numpy array (m by n)
# default is linear interpolation
# when x_queue exceeds the range of x, function returns the boundary value of y
    boundary = (y[:, 0], y[:, -1])
    f = interpolate.interp1d(x, y, kind='linear', bounds_error=False, fill_value=boundary)
    y_raw = f(x_queue)
    if isinstance(x_queue, float):
        y_queue = y_raw.reshape(y_raw.shape[0], -1)
    else:
        y_queue = y_raw
    return y_queue


def LLC_PID(idx, posi_now, point_ref_0, point_ref_1, yaw_now, yaw_des, Rot_Mat, P_now, velo_body, yaw_rate, vz_max, dt):
#####################################################################
# control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.

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
    fwdfeedroll = 30.0            #forward feed coefficient 
    Kp_roll = 1.0                 #pid proportional gain 
    Ki_roll = 0.0                 #pid integral gain
    Kd_roll = 0.2                 #pid derivative gain
    # 50 1 0.1
    #yaw rate controller gains
    fwdfeedyaw = 20.0             #forward feed coefficient 
    Kp_psi = 40.0                 #pid proportional gain
    Ki_psi = 0.0                  #pid integral gain
    Kd_psi = 0.0                  #pid derivative gain
#####################################################################
    a = '''
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
    '''
#####################################################################
    # position feedforward vector in global frame
    feedforward_posi = -1.0 * (posi_now - np.reshape(point_ref_1[0:3, 0], (-1, 1))) # 3 by 1
    feedforward_yaw = -1.0 * (yaw_now - yaw_des)
    # position feedforward vector in body frame
    feedforward_posi_body = np.dot(np.linalg.pinv(Rot_Mat), feedforward_posi)

    P_pre = P_now
    P_now = np.vstack((feedforward_posi_body - velo_body, feedforward_yaw - yaw_rate))
    D_now = (P_now - P_pre) / dt

    if idx == 0:
        pitch_cmd = Kp_pitch*P_now[0, 0] + fwdfeedpitch*feedforward_posi_body[0, 0]
        roll_cmd = Kp_roll*P_now[2, 0] + fwdfeedroll*feedforward_posi_body[2, 0]
    else:
        pitch_cmd = Kp_pitch*P_now[0, 0] + Kd_pitch*D_now[0, 0] + fwdfeedpitch*feedforward_posi_body[0, 0]
        roll_cmd = Kp_roll*P_now[2, 0] + Kd_roll*D_now[2, 0] + fwdfeedroll*feedforward_posi_body[2, 0]

    vz_cmd = point_ref_1[4, 0] / vz_max * 100.0 + Kp_height * (point_ref_0[1, 0] - posi_now[1, 0])
    yaw_rate_cmd = Kp_psi*P_now[3, 0] + fwdfeedyaw*feedforward_yaw 

    yaw_rate_cmd = -1.0 * yaw_rate_cmd

    return P_now, pitch_cmd, roll_cmd, vz_cmd, yaw_rate_cmd


def LLC_PID_sin(idx, posi_now, point_ref_0, point_ref_1, yaw_now, yaw_des, Rot_Mat, P_now, velo_body, yaw_rate, vz_max, dt):
#####################################################################
# control input vz depends on the ultrasonic sensor, which means there shouldn't be any obstacles under the drone.

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
    fwdfeedroll = 30.0            #forward feed coefficient 
    Kp_roll = 1.0                 #pid proportional gain 
    Ki_roll = 0.0                 #pid integral gain
    Kd_roll = 0.2                 #pid derivative gain
    # 50 1 0.1
    #yaw rate controller gains
    fwdfeedyaw = 20.0             #forward feed coefficient 
    Kp_psi = 0.0                 #pid proportional gain
    Ki_psi = 0.0                  #pid integral gain
    Kd_psi = 0.0                  #pid derivative gain
#####################################################################
    # position feedforward vector in global frame
    feedforward_posi = -1.0 * (posi_now - np.reshape(point_ref_1[0:3, 0], (-1, 1))) # 3 by 1
    feedforward_yaw = -1.0 * (sin(yaw_now) - sin(yaw_des))
    # position feedforward vector in body frame
    feedforward_posi_body = np.dot(np.linalg.pinv(Rot_Mat), feedforward_posi)

    P_pre = P_now
    P_now = np.vstack((feedforward_posi_body - velo_body, feedforward_yaw - sin(yaw_rate)))
    D_now = (P_now - P_pre) / dt

    if idx == 0:
        pitch_cmd = Kp_pitch*P_now[0, 0] + fwdfeedpitch*feedforward_posi_body[0, 0]
        roll_cmd = Kp_roll*P_now[2, 0] + fwdfeedroll*feedforward_posi_body[2, 0]
    else:
        pitch_cmd = Kp_pitch*P_now[0, 0] + Kd_pitch*D_now[0, 0] + fwdfeedpitch*feedforward_posi_body[0, 0]
        roll_cmd = Kp_roll*P_now[2, 0] + Kd_roll*D_now[2, 0] + fwdfeedroll*feedforward_posi_body[2, 0]

    vz_cmd = point_ref_1[4, 0] / vz_max * 100.0 + Kp_height * (point_ref_0[1, 0] - posi_now[1, 0])
    yaw_rate_cmd = Kp_psi*P_now[3, 0] + fwdfeedyaw*feedforward_yaw 

    yaw_rate_cmd = -1.0 * yaw_rate_cmd

    return P_now, pitch_cmd, roll_cmd, vz_cmd, yaw_rate_cmd


if __name__ == '__main__':
    x = np.array([0, 2, 4, 6, 8, 10])
    y = np.array([[0, 1, 2, 3, 4, 5], [0, -1, -2, -3, -4, -5], [10, 20, 30, 40, 50, 60], [-10, -20, -30, -40, -50, -60], [0.5, 1.5, 2.5, 3.5, 4.5, 5.5]])
    x_queue = -100
    y_queue = interpolate_traj(x_queue, x, y)
    print(y_queue)

    