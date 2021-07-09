#!/usr/bin/env python3
import os
import time
import socket
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

class SocketQualisys(object):
    def __init__(config_data: dict):
        """
        Create a TCP/IP socket to subscribe 6 DOF state estimation from motion capture system Qualisys.
        Load the configuration as a dictionary
            json_file = open("config_aimslab.json")
            config_data = json.load(json_file)
        """
        # NOTE: by TCP/IP, two client may not be able to listen to the server via the same port.
        # NOTE: I need to test it. If not be able to, create another port in run_mocap_qualisys.py

        self.config_data = config_data
        # Create a TCP/IP socket
        self.sock_states = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mocap_type = "QUALISYS"
        # Connect the socket to the port where the mocap server is publishing
        server_address_states = (self.config_data[self.mocap_type]["IP_STATES_ESTIMATION"], int(self.config_data[self.mocap_type]["PORT_STATES_ESTIMATION"]))
        print("connecting to", server_address_states)
        self.sock_states.connect(server_address_states)
        # maximum bytes received from TCP socket
        self.data_bytes_max = int(self.config_data[self.mocap_type]["DATA_BYTES_MAX"])
        # how many numbers the TCP socket is sending
        self.data_number_integer = int(self.config_data[self.mocap_type]["DATA_NUMBERS_STATES_ESTIMATION"])

    def update_states_mocap(self):
        """
        Update the positions from motion capture system Qualisys.
        """
        msg = self.sock_states.recv(self.data_bytes_max)
        if msg:
            data = np.frombuffer(msg, dtype=float)
            num_data_group = int(np.size(data)/self.data_number_integer)
            data_all = data[-self.data_number_integer*num_data_group:]
            data_one_sample = data[-self.data_number_integer:]
            # 1D list for position, [px, py, pz], in meters
            posi_now = data_one_sample[0:3].tolist()
        return posi_now


if __name__ == "__main__":
    # load the configuration as a dictionary
    json_file = open("config_aimslab.json")
    config_data = json.load(json_file)

    # create a simulator
    Simulator = SimulatorAimsLab(map_resolution=20)
    # convert 2D numpy array to 1D list
    world_map = Simulator.map_array.flatten().tolist()

    # create a TCP/IP socket to subscribe states from mocap Qualisys
    MySocketQualisys = SocketQualisys(config_data)

    frequency = 2  # planner frequency in Hz
    height = 1.0  # height is fixed during the flight
    iter_idx = 0
    while(iter_idx < 5):
        t_start = time.time()

        # update the agent position
        # agent_position_qualisys = [-1.8, -0.9, height]  # meter
        agent_position_qualisys = MySocketQualisys.update_states_mocap()

        # update the targets positions
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
        # # visualization
        # Simulator.plot_many_path_single_agent(path_index, agent_position_index, targets_position_index, task_allocation_result)

        # transform map array (index) to qualisys coordinates (meter)
        t0 = time.time()
        path_qualisys = map_index_to_qualisys_all(path_index, agent_position_qualisys[2], Simulator)
        t1 = time.time()
        print("Map index to qualisys coordinate. Time used [sec]: " + str(t1 - t0))

        # generate position and velocity trajectories
        dt = 0.1
        velocity_ave = 0.25
        # generate trajectories
        time_queue_vec, position_traj, velocity_traj = discrete_path_to_time_traj(
            path_qualisys, dt, velocity_ave, interp_kind='quadratic', ini_velocity_zero_flag=False)

        # # plot path, and position/velocity trajectories
        # plot_traj(path_qualisys, time_queue_vec, position_traj, velocity_traj)
        # plt.show()

        # output trajectories as a CSV file
        array_csv = np.vstack((time_queue_vec, np.array(position_traj).T, np.array(velocity_traj).T))
        time_name = time.strftime("%Y%m%d%H%M%S")
        filename_csv = os.getcwd() + config_data["DIRECTORY_TRAJ"] + time_name + ".csv"
        np.savetxt(filename_csv, array_csv, delimiter=",")

        iter_idx += 1
        t_end = time.time()
        print("A planning action. Time used [sec]: " + str(t_end - t_start))
        time_sleep = max(0, 1 / frequency - t_end + t_start)
        time.sleep(time_sleep)
        time_after_sleep = time.time()
        print("Time used [used] for a single iteration: " + str(time_after_sleep - t_start))
