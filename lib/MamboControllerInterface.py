#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/lib')
import socket
import time
import json
import time
import struct
import transforms3d
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pyparrot.Minidrone import Mambo
from math import degrees, radians, sin, cos
import numpy as np

import csv_helper_module as csv_helper


class MamboControllerInterface:
    def __init__(self, config_file_name):
        # Read the configuration from the json file
        json_file = open(config_file_name)
        config_data = json.load(json_file)

        # desired yaw angle, if fly backward, choose pi; otherwise, choose 0.0, in radians
        self.yaw_des = float(config_data["LOW_LEVEL_CONTROLLER"]["YAW_DES"])

        # make my mambo object
        # remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
        self.mambo = Mambo(config_data["MAMBO"]["BLUETOOTH_ADDRESS"], use_wifi=False)
        self.flag_mambo_connection = mambo.connect(num_retries=3)
        print("Mambo connected: %s" % self.flag_mambo_connection)

        # Create a TCP/IP socket
        self.sock_states = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect the socket to the port where the server is listening
        server_address_states = (config_data["QUALISYS"]["IP_STATES_ESTIMATION"], int(config_data["QUALISYS"]["PORT_STATES_ESTIMATION"]))
        print("connecting to", server_address_states)
        self.sock_states.connect(server_address_states)

        # Sending real-time positions and velocities to MATLAB via UDP
        # Connect the socket to the port where the server is listening
        server_address_matlab = (config_data["RTD_MATLAB"]["IP_MATLAB"], int(config_data["RTD_MATLAB"]["PORT_MATLAB"]))
        # Create a UDP socket
        sock_matlab = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # define the path for system id and csv trajectories
        self.directory_sysid = os.getcwd() + '/sysid_data/'
        self.directory_traj = os.getcwd() + '/traj_csv_files/'
        self.directory_delete = os.getcwd() + '/traj_csv_files/*'

        # define max tilt angles, vertical velocity, and yaw rate
        self.tilt_max = radians(float(config_data["MAMBO"]["TILT_MAX"])) # in degrees to radians
        self.vz_max = float(config_data["MAMBO"]["VERTICAL_SPEED_MAX"]) # in m/s
        self.yaw_rate_max = float(config_data["MAMBO"]["YAW_RATE_MAX"]) # in radians/sec

        # initialize more variables



    def run_LLC(self):









if __name__ == "__main__":
    config_file_name = 'config.json'

    