#!/usr/bin/env python3
import os
import glob
import csv
import numpy as np


"""
    This is a helper module to process desired trajectoroes which are 
    represented by csv files.
"""


def remove_traj_ref_lib(Directory):
    """
    Delete all the files under a specific directory

    Args:
        Directory: A string for the directory

    Returns:
        void

    Usage:
        Directory_delete = '/home/blah/RTD_Mambo_interface/traj_csv_files/*'
        remove_traj_ref_lib(Directory_delete)
    """

    files = glob.glob(Directory)
    for f in files:
        os.remove(f)


def import_csv(FileName):
    """
    Import a csv file, which contains the desired positions, velocities, and accelerations.
    And it returns the desired positions and velocities.
    If offset_flag is True, use the original reference height, it's only used in the end.
    If offset_flag is False, substract 0.1 meters to the height, it's used during the loop.

    Args:
        FileName: A string for the absoluate file path

    Returns:
        traj_ref: A 2D Numpy array, each column stands for the position (px, py, pz) and velocity (vx, vy, vz) at a timestamp
        T: A 1D Numpy array, the time trajectory

    Usage:
        Directory_traj = os.getcwd() + '/traj_csv_files/'
        csv_file_list = sorted(glob.glob(Directory + '*.csv'), key=os.path.getmtime)
        traj_ref, T = import_csv(csv_file_list[-1])
    """

    Z = np.genfromtxt(FileName, dtype=float, delimiter=',')
    T = Z[0, :]
    traj_ref = Z[1:7, :]

    return traj_ref, T


def update_csv(Directory):
    """
    Update the desired trajectory from the csv file folder.
    Assume traj_ref has the whole time trajectory information.
    Assume traj_ref only has positions and velocities.

    Args:
        Directory: A string for the absoluate file path

    Returns:
        traj_ref: A 2D Numpy array, each column stands for the position and velocity at a timestamp
        T: A 1D Numpy array, the time trajectory
        hover_flag: A bool, true when there is no csv files(at the very beginning, let the drone hover);
            false when there is at least one csv file (let the drone execute the csv trajectory)
        csv_length_now: A integer, indicates the length of the desired trajectory

    Usage:
        Directory_traj = os.getcwd() + '/traj_csv_files/'
        traj_ref, T, hover_flag, csv_length_now = update_csv(Directory_traj)
    """

    csv_file_list = sorted(glob.glob(Directory + '*.csv'), key=os.path.getmtime)
    
    if not csv_file_list:
        # at the very beginning, hover_flag is true
        hover_flag = True
        traj_ref = np.zeros((6, 2), dtype=float)
        T = np.array([0.0, 1.0], dtype=float)
        csv_length_now = np.shape(traj_ref)[1]
    else:
        try:
            traj_ref, T = import_csv(csv_file_list[-1])
            csv_length_now = np.shape(traj_ref)[1]
        except:
            traj_ref = list()
            T = list()
            csv_length_now = 0
            print("The lastest one is empty!!!")
        hover_flag = False

    return traj_ref, T, hover_flag, csv_length_now


if __name__ == "__main__":
    Directory_traj = os.getcwd() + '/traj_csv_files/'
    traj_ref, T, hover_flag, csv_length_now = update_csv(Directory_traj)

    print(traj_ref)
    print(T)
    print(hover_flag)
    print(csv_length_now)
