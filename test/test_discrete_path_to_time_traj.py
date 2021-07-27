#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/lib')
import random
import matplotlib.pyplot as plt
from discrete_path_to_time_traj import discrete_path_to_time_traj, plot_traj


if __name__ == "__main__":
    # path = [[1,2], [2,5], [3,8], [5,10], [12,12]]
    # path = [[1,2,0], [2,5,1], [3,8,3], [5,10,6], [12,12,7]]

    # generate a random path
    length_path = 6
    dimension = 3
    # dimension = 2

    path = []
    for idx in range(0, length_path):
        if dimension == 3:
            path.append([random.uniform(-3,3)+2*idx, random.uniform(-3,3)+2*idx, random.uniform(-3,3)+2*idx])
        elif dimension == 2:
            path.append([random.uniform(-3,3)+2*idx, random.uniform(-3,3)+2*idx])
        else:
            raise Exception("Path dimension only support 2 and 3!")

    dt = 0.1
    velocity_ave = 0.5

    # call discrete_path_to_time_traj to generate trajectories
    time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
        path, dt, velocity_ave, interp_kind='linear',
        velocity_flag=True, ini_velocity_zero_flag=True)

    # plot path, and position/velocity trajectories
    plot_traj(path, time_queue_vec, position_traj, velocity_traj)
    plt.show()
