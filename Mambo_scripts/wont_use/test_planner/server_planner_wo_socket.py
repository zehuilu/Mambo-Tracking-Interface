import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

import socket
import numpy as np
import time
import json

import generate_spline_peak_speed as gen_spline


flag_append_list = []
px_list = []
py_list = []
pz_list = []
vx_list = []
vy_list = []
vz_list = []
ax_list = []
ay_list = []
az_list = []



# generate a desired trajectory
# those are all in the phasespace frame
# vx points front, vy points up, vz points right, always check the length of trajectory
dt_traj = 0.10
p_0 = np.array([[0.0], [0.6], [0.0]])
v_0 = np.array([[0.0], [0.0], [0.0]])
a_0 = np.array([[0.0], [0.0], [0.0]])
t_peak = 2.5
t_total = 3.0


traj_change_flag = [1, 0, 0, 0, 0, 0]
v_peak_dic = {\
    0: np.array([[0.0], [0.0], [0.0]]), \
    1: np.array([[0.0], [0.0], [0.0]]), \
    2: np.array([[0.0], [0.0], [0.0]]), \
    3: np.array([[0.0], [0.0], [0.0]]), \
    4: np.array([[0.0], [0.0], [0.0]]), \
    5: np.array([[0.0], [0.0], [0.0]])}


data = {"flag_append": 0, "px": 0.0, "py": 0.0, "pz": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0}
    
# idx is the index for each step for the while loop
idx = -1
# num is the index for # of section in a 3.0 sec trajectory
num = 0
# idx_change_traj is the index for # of trajectory change times
idx_change_traj = 0

while idx <= 29:
    if idx % 5 == 0 and idx >= 0 :
        if traj_change_flag[idx_change_traj] == 1:
            v_peak = v_peak_dic[idx_change_traj]

            if idx == 0:
                num = 0
                ##########
                gen_class = gen_spline.generate_spline_by_peak_speed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt_traj)
                time_ref, traj_ref = gen_class.do_calculation()
                ##########
                data = {\
                    "flag_append": 1, \
                    "px": traj_ref[0, num*5:(num+1)*5+1].tolist(), \
                    "py": traj_ref[1, num*5:(num+1)*5+1].tolist(), \
                    "pz": traj_ref[2, num*5:(num+1)*5+1].tolist(), \
                    "vx": traj_ref[3, num*5:(num+1)*5+1].tolist(), \
                    "vy": traj_ref[4, num*5:(num+1)*5+1].tolist(), \
                    "vz": traj_ref[5, num*5:(num+1)*5+1].tolist(), \
                    "ax": traj_ref[6, num*5:(num+1)*5+1].tolist(), \
                    "ay": traj_ref[7, num*5:(num+1)*5+1].tolist(), \
                    "az": traj_ref[8, num*5:(num+1)*5+1].tolist()}


                traj_ref_list = np.concatenate((\
                    np.array([data["px"]]), np.array([data["py"]]), np.array([data["pz"]]), \
                    np.array([data["vx"]]), np.array([data["vy"]]), np.array([data["vz"]]), \
                    np.array([data["ax"]]), np.array([data["ay"]]), np.array([data["az"]])), axis=0)


            else:
                num = 0

                p_0 = np.array([[data["px"][-1]], [data["py"][-1]], [data["pz"][-1]]])
                v_0 = np.array([[data["vx"][-1]], [data["vy"][-1]], [data["vz"][-1]]])
                a_0 = np.array([[data["ax"][-1]], [data["ay"][-1]], [data["az"][-1]]])
                ##########
                gen_class = gen_spline.generate_spline_by_peak_speed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt_traj)
                time_ref, traj_ref = gen_class.do_calculation()
                ##########

                data = {\
                    "flag_append": 1, \
                    "px": traj_ref[0, num*5+1:(num+1)*5+1].tolist(), \
                    "py": traj_ref[1, num*5+1:(num+1)*5+1].tolist(), \
                    "pz": traj_ref[2, num*5+1:(num+1)*5+1].tolist(), \
                    "vx": traj_ref[3, num*5+1:(num+1)*5+1].tolist(), \
                    "vy": traj_ref[4, num*5+1:(num+1)*5+1].tolist(), \
                    "vz": traj_ref[5, num*5+1:(num+1)*5+1].tolist(), \
                    "ax": traj_ref[6, num*5+1:(num+1)*5+1].tolist(), \
                    "ay": traj_ref[7, num*5+1:(num+1)*5+1].tolist(), \
                    "az": traj_ref[8, num*5+1:(num+1)*5+1].tolist()}

            idx_change_traj += 1


        else:
            if num > 5:
                data = {\
                    "flag_append": 1, \
                    "px": [traj_ref[0, -1]] * 5, \
                    "py": [traj_ref[1, -1]] * 5, \
                    "pz": [traj_ref[2, -1]] * 5, \
                    "vx": [traj_ref[3, -1]] * 5, \
                    "vy": [traj_ref[4, -1]] * 5, \
                    "vz": [traj_ref[5, -1]] * 5, \
                    "ax": [traj_ref[6, -1]] * 5, \
                    "ay": [traj_ref[7, -1]] * 5, \
                    "az": [traj_ref[8, -1]] * 5}

            else:
                data = {\
                    "flag_append": 1, \
                    "px": traj_ref[0, num*5+1:(num+1)*5+1].tolist(), \
                    "py": traj_ref[1, num*5+1:(num+1)*5+1].tolist(), \
                    "pz": traj_ref[2, num*5+1:(num+1)*5+1].tolist(), \
                    "vx": traj_ref[3, num*5+1:(num+1)*5+1].tolist(), \
                    "vy": traj_ref[4, num*5+1:(num+1)*5+1].tolist(), \
                    "vz": traj_ref[5, num*5+1:(num+1)*5+1].tolist(), \
                    "ax": traj_ref[6, num*5+1:(num+1)*5+1].tolist(), \
                    "ay": traj_ref[7, num*5+1:(num+1)*5+1].tolist(), \
                    "az": traj_ref[8, num*5+1:(num+1)*5+1].tolist()}

        # record
        if idx > 0:
            traj_ref_list = np.concatenate((traj_ref_list, np.array([data["px"], data["py"], data["pz"], data["vx"], data["vy"], data["vz"], data["ax"], data["ay"], data["az"]])), axis=1)

        num += 1
        print("sending message to the client")
    else:
        print("sending data, but no new data")
        data["flag_append"] = 0

    # record
    flag_append_list.append(data["flag_append"])


    idx += 1
    #time.sleep(0.1)


# plot
print(traj_ref_list)
print(traj_ref_list.shape)