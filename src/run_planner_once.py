#!/usr/bin/env python3
import os
import sys

# sys.path.append('/Users/zehui/Real-time-Task-Allocation-and-Path-Planning/build')
# sys.path.append('/Users/zehui/Real-time-Task-Allocation-and-Path-Planning/src')

sys.path.append('/home/aims-zehui/Real-time-Task-Allocation-and-Path-Planning/build')
sys.path.append('/home/aims-zehui/Real-time-Task-Allocation-and-Path-Planning/src')

sys.path.append(os.getcwd()+'/lib')
import TAMP_SOLVER
import time
import json
import matplotlib.pyplot as plt
import numpy as np
from itertools import chain
from Simulator import Simulator
from discrete_path_to_time_traj import discrete_path_to_time_traj, plot_traj
import coordinate_transformation_aims as coord_aims


if __name__ == "__main__":
    # define the world
    map_width_meter = 3.4
    map_height_meter = 4.8 
    map_resolution = 20
    value_non_obs = 0  # the cell is empty
    value_obs = 255  # the cell is blocked
    # create a simulator
    Simulator = Simulator(map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)

    # generate obstacles manually
    # Simulator.map_array[5][6] = Simulator.value_obs
    # generate random obstacles
    Simulator.generate_random_obs(30, [0.2,0.2])
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()

    # generate agent and targets manually, qualisys coordinates
    height = 1.0 # height is fixed at 0.6 meter
    agent_position_qualisys = [-1.8, -0.9, height]
    targets_position_qualisys = [[0.2, -0.4, height], [1.8, 0.9, height]]

    # qualisys coordinate to map array coordinate (meter)
    agent_position_meter = coord_aims.qualisys_to_map_meter(agent_position_qualisys)
    targets_position_meter = []
    for idx in range(len(targets_position_qualisys)):
        targets_position_meter.append(coord_aims.qualisys_to_map_meter(targets_position_qualisys[idx]))
    # map array coordinate (meter) to map array coordinate (index)
    agent_position_index = Simulator.position_to_map_index(agent_position_meter[:-1]) # 2D, no height
    targets_position_index = []
    for idx in range(len(targets_position_meter)):
        target_position_meter_now = targets_position_meter[idx]
        targets_position_index.extend(Simulator.position_to_map_index(target_position_meter_now[:-1])) # 2D, no height


    # high-level planning
    t0 = time.time()
    path_index, task_allocation_result = TAMP_SOLVER.SolveOneAgent(agent_position_index, targets_position_index, world_map, Simulator.map_width, Simulator.map_height)
    t1 = time.time()
    print("High-level planning. Time used [sec]:" + str(t1 - t0))

    print("path_index")
    print(path_index)


    # coordinate transformation
    t0 = time.time()
    # for each sub-list of path_index, the last two elements (position index) are the same with the first two elements of next sub-list.
    # delete the duplicated ones.
    for idx in range(len(path_index)-1):
        del path_index[idx][-1]
        del path_index[idx][-1]
    # flatten the path list, and reshape it to N by #dimension array
    path_index_1d = list(chain.from_iterable(path_index))
    path_index_2d = np.reshape(path_index_1d, (-1,2))

    # NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

    # map array coordinate (index) to map array coordinate (meter)
    path_meter_2d = np.zeros(path_index_2d.shape)
    for idx in range(path_index_2d.shape[0]):
        path_meter_2d[idx] = Simulator.map_index_to_position(path_index_2d[idx])

    # stack the constant height in meter
    height_array = height * np.ones((path_meter_2d.shape[0], 1))
    path_meter_3d = np.hstack((path_meter_2d, height_array))

    # map array coordinate (meter) to qualisys coordinate
    path_qualisys = np.zeros(path_meter_3d.shape)
    for idx in range(path_meter_3d.shape[0]):
        path_qualisys[idx] = coord_aims.map_meter_to_qualisys(path_meter_3d[idx])
    t1 = time.time()
    print("Coordinate transformation. Time used [sec]:" + str(t1 - t0))

    print("path_qualisys")
    print(path_qualisys)

    # generate position and velocity trajectories
    dt = 0.1
    velocity_ave = 0.25
    # generate trajectories
    time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(path_qualisys.tolist(), dt, velocity_ave, interp_kind='quadratic')

    # visualization
    Simulator.plot_many_path_single_agent(path_index, agent_position_index, targets_position_index, task_allocation_result)
    # plot path, and position/velocity trajectories
    plot_traj(path_qualisys.tolist(), time_queue_vec, position_traj, velocity_traj)
    plt.show()

    # output trajectories as a CSV file
    array_csv = np.vstack((time_queue_vec, np.array(position_traj).T, np.array(velocity_traj).T))
    json_file = open("config_aimslab.json")
    config_dict = json.load(json_file)
    filename_csv = os.getcwd() + config_dict["DIRECTORY_TRAJ"] + "traj.csv"
    np.savetxt(filename_csv, array_csv, delimiter=",")
