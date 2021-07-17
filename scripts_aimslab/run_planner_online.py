#!/usr/bin/env python3
import os
import time
import socket
import json
import matplotlib.pyplot as plt
import numpy as np
import pathmagic
with pathmagic.context(EXTERNAL_FLAG=True):
    import csv_helper
    import TAMP_SOLVER
    from SimulatorAimsLab import SimulatorAimsLab
    from discrete_path_to_time_traj import discrete_path_to_time_traj, plot_traj


class SocketQualisys(object):
    def __init__(self, config_data: dict):
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

        # create a UDP socket
        # IP for publisher
        HOST_OBS = self.config_data[self.mocap_type]["IP_OBS_POSITION"]
        # Port for publisher
        PORT_OBS = int(self.config_data[self.mocap_type]["PORT_OBS_POSITION"])
        self.server_address_obs = (HOST_OBS, PORT_OBS)
        # Create a UDP socket
        self.sock_obs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_obs.bind(self.server_address_obs)
        # maximum bytes received from UDP socket
        self.data_bytes_max_obs = int(self.config_data[self.mocap_type]["DATA_BYTES_MAX_OBS"])
        # how many numbers the UDP socket publisher is sending
        self.data_number_integer_obs = int(self.config_data[self.mocap_type]["DATA_NUMBERS_OBS"])

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

    def update_obs_mocap(self):
        """
        Update the obstacle positions from motion capture system Qualisys.
        """
        msg = self.sock_obs.recv(self.data_bytes_max_obs)
        if msg:
            data = np.frombuffer(msg, dtype=float)
            num_data_group = int(np.size(data)/self.data_number_integer_obs)
            data_all = data[-self.data_number_integer_obs*num_data_group:]
            data_one_sample = data[-self.data_number_integer_obs:]
            # 1D list for position, [px, py, pz], in meters
            posi_obs = data_one_sample[0:3].tolist()
        return posi_obs


if __name__ == "__main__":
    # load the configuration as a dictionary
    json_file = open("config_aimslab.json")
    config_data = json.load(json_file)

    # remove all the existing files in the trajectory directory
    directory_delete = os.getcwd() + config_data["DIRECTORY_TRAJ"] + "*"
    csv_helper.remove_traj_ref_lib(directory_delete)

    # create a simulator
    MySimulator = SimulatorAimsLab(map_resolution=20)

    # size of the obstacle in qualisys coordinate [x, y] (meter) with infinite height
    size_obs_qualisys = [0.1, 0.1]

    # # create a TCP/IP socket to subscribe states from mocap Qualisys
    # MySocketQualisys = SocketQualisys(config_data)

    frequency = 2  # planner frequency in Hz
    height = 1.0  # height is fixed during the flight
    time_escape = 5  # shut down the planner after this time [sec]
    iter_idx = 0
    while(iter_idx < (time_escape * frequency)):
        t_start = time.time()

        # update the agent position
        agent_position_qualisys = [[-1.8, -0.9, height]]  # meter
        # agent_position_qualisys = MySocketQualisys.update_states_mocap()

        # # update the obstacle position
        # obs_position = MySocketQualisys.update_obs_mocap()
        # print("obs_position")
        # print(obs_position)
        # # update the map
        # self.update_obs_map(obs_position, size_obs_qualisys)

        # convert 2D numpy array to 1D list
        world_map = MySimulator.map_array.flatten().tolist()

        # update the targets positions
        targets_position_qualisys = [[0.2, -0.4, height], [1.8, 0.9, height]]  # meter

        # transform qualisys coordinates (meter) to map array (index)
        t0 = time.time()
        agent_position_index = MySimulator.qualisys_to_map_index_all(agent_position_qualisys)
        targets_position_index = MySimulator.qualisys_to_map_index_all(targets_position_qualisys)
        t1 = time.time()
        print("Qualisys coordinates to map index. Time used [sec]: " + str(t1 - t0))

        # high-level planning
        t0 = time.time()
        path_index, task_allocation_result = TAMP_SOLVER.SolveOneAgent(
            agent_position_index, targets_position_index, world_map, MySimulator.map_width, MySimulator.map_height)
        t1 = time.time()
        print("High-level planning. Time used [sec]: " + str(t1 - t0))
        # # visualization
        # MySimulator.plot_many_path_single_agent(path_index, agent_position_index, targets_position_index, task_allocation_result)

        # transform map array (index) to qualisys coordinates (meter)
        t0 = time.time()
        path_qualisys = MySimulator.path_index_to_qualisys(path_index, height)
        t1 = time.time()
        print("Map index to qualisys coordinate. Time used [sec]: " + str(t1 - t0))

        # generate position and velocity trajectories (as a motion planner)
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
