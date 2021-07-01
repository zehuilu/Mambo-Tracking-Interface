#!/usr/bin/env python3
import os
import sys
sys.path.append('/Users/zehui/Real-time-Task-Allocation-and-Path-Planning/build')
sys.path.append('/Users/zehui/Real-time-Task-Allocation-and-Path-Planning/src')
sys.path.append(os.getcwd()+'/lib')
import TAMP_SOLVER
import time
import json
import matplotlib.pyplot as plt
import numpy as np
from itertools import chain
from Simulator import Simulator
from discrete_path_to_time_traj import discrete_path_to_time_traj, plot_traj


if __name__ == "__main__":
    # define the world
    map_width_meter = 25.0
    map_height_meter = 25.0
    map_resolution = 2
    value_non_obs = 0  # the cell is empty
    value_obs = 255  # the cell is blocked
    # create a simulator
    Simulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)

    # generate obstacles manually
    # Simulator.map_array[5][6] = Simulator.value_obs
    # generate random obstacles
    Simulator.generate_random_obs(40, [1,1])
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()

    # generate agent and target manually
    num_agents = 1
    num_targets = 5
    agent_position = [3, 5]
    targets_position = [23,30, 45,43]

    t0 = time.time()
    # solve it
    path_index, task_allocation_result = TAMP_SOLVER.SolveOneAgent(agent_position, targets_position, world_map, Simulator.map_width, Simulator.map_height)
    t1 = time.time()

    print("path_index")
    print(path_index)

    # for each sub-list of path_index, the last two elements (position index) are the same with the first two elements of next sub-list. delete the duplicated ones.
    for idx in range(0, len(path_index)-1):
        del path_index[idx][-1]
        del path_index[idx][-1]
    # flatten the path list, and reshape it to N by #dimension array
    path_index_1d = list(chain.from_iterable(path_index))
    path_output = np.reshape(path_index_1d, (-1,2)).tolist()

    print("path_output")
    print(path_output)

    # generate position and velocity trajectories
    dt = 0.1
    velocity_ave = 1.0
    # generate trajectories
    time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(path_output, dt, velocity_ave, interp_kind='quadratic')

    # visualization
    Simulator.plot_many_path_single_agent(path_index, agent_position, targets_position, task_allocation_result)
    # plot path, and position/velocity trajectories
    plot_traj(path_output, time_queue_vec, position_traj, velocity_traj)
    plt.show()

    # output trajectories as a CSV file
    array_csv = np.vstack((time_queue_vec, np.array(position_traj).T, np.array(velocity_traj).T))
    json_file = open("config_aimslab.json")
    config_dict = json.load(json_file)
    filename_csv = os.getcwd() + config_dict["DIRECTORY_TRAJ"] + "traj.csv"
    np.savetxt(filename_csv, array_csv, delimiter=",")
