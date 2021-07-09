#!/usr/bin/env python3
import os
import time
import json
import matplotlib.pyplot as plt
import numpy as np
from itertools import chain
import pathmagic
with pathmagic.context(EXTERNAL_FLAG=True):
    import TAMP_SOLVER
    from SimulatorAimsLab import SimulatorAimsLab
from discrete_path_to_time_traj import discrete_path_to_time_traj, plot_traj
import coordinate_transformation_qualisys as coord_qualisys


def qualisys_to_map_index_all(agent_position_qualisys: list, targets_position_qualisys: list, Simulator):
    """
    Transform the qualisys coordinates (meter) of agent and targets to the map array (index) coordinates.
    NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.
    """

    # qualisys coordinate to map array coordinate (meter)
    agent_position_meter = coord_qualisys.qualisys_to_map_meter(agent_position_qualisys)

    targets_position_meter = []
    for idx in range(len(targets_position_qualisys)):
        targets_position_meter.append(coord_qualisys.qualisys_to_map_meter(targets_position_qualisys[idx]))

    # map array coordinate (meter) to map array coordinate (index)
    agent_position_index = Simulator.position_to_map_index(agent_position_meter[:-1])  # 2D, no height

    targets_position_index = []
    for idx in range(len(targets_position_meter)):
        target_position_meter_now = targets_position_meter[idx]
        targets_position_index.extend(Simulator.position_to_map_index(target_position_meter_now[:-1]))  # 2D, no height

    return agent_position_index, targets_position_index


def map_index_to_qualisys_all(path_index: list, height: float, Simulator):
    """
    Transform the map array (index) coordinates of agent and targets to the qualisys coordinates (meter).
    NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.
    """

    # for each sub-list of path_index, the last two elements (position index) are the same with the first two elements of next sub-list.
    # delete the duplicated ones.
    for idx in range(len(path_index)-1):
        del path_index[idx][-1]
        del path_index[idx][-1]

    # flatten the path list, and reshape it to N by #dimension array
    path_index_1d = list(chain.from_iterable(path_index))
    path_index_2d = np.reshape(path_index_1d, (-1,2))

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
        path_qualisys[idx] = coord_qualisys.map_meter_to_qualisys(path_meter_3d[idx])

    return path_qualisys.tolist()


if __name__ == "__main__":
    # create a simulator
    Simulator = SimulatorAimsLab(map_resolution=20)
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()

    # generate agent and targets manually in qualisys coordinates (meter)
    height = 1.0  # height is fixed during the flight
    agent_position_qualisys = [-1.8, -0.9, height]
    targets_position_qualisys = [[0.2, -0.4, height], [1.8, 0.9, height]]
    # transform qualisys coordinates (meter) to map array (index)
    t0 = time.time()
    agent_position_index, targets_position_index = qualisys_to_map_index_all(
        agent_position_qualisys, targets_position_qualisys, Simulator)
    t1 = time.time()
    print("Qualisys coordinates to map index. Time used [sec]: " + str(t1 - t0))

    # high-level planning
    t0 = time.time()
    path_index, task_allocation_result = TAMP_SOLVER.SolveOneAgent(
        agent_position_index, targets_position_index, world_map, Simulator.map_width, Simulator.map_height)
    t1 = time.time()
    print("High-level planning. Time used [sec]: " + str(t1 - t0))
    # visualization
    Simulator.plot_many_path_single_agent(path_index, agent_position_index, targets_position_index, task_allocation_result)

    # transform  map array (index) to qualisys coordinates (meter)
    t0 = time.time()
    path_qualisys = map_index_to_qualisys_all(path_index, height, Simulator)
    t1 = time.time()
    print("Map index to qualisys coordinate. Time used [sec]: " + str(t1 - t0))

    # generate position and velocity trajectories
    dt = 0.1
    velocity_ave = 0.25
    # generate trajectories
    time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
        path_qualisys, dt, velocity_ave, interp_kind='quadratic', ini_velocity_zero_flag=True)

    # plot path, and position/velocity trajectories
    plot_traj(path_qualisys, time_queue_vec, position_traj, velocity_traj)
    plt.show()

    # output trajectories as a CSV file
    array_csv = np.vstack((time_queue_vec, np.array(position_traj).T, np.array(velocity_traj).T))
    json_file = open("config_aimslab.json")
    config_dict = json.load(json_file)
    filename_csv = os.getcwd() + config_dict["DIRECTORY_TRAJ"] + "traj.csv"
    np.savetxt(filename_csv, array_csv, delimiter=",")
