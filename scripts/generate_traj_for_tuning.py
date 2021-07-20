#!/usr/bin/env python3
import os
import numpy as np
import json
import pathmagic
with pathmagic.context():
    from SplineByPeakSpeed import SplineByPeakSpeed


if __name__ == "__main__":
    p_0 = np.array([[-1.8], [-0.9], [0.6]])
    # p_0 = np.array([[0.0], [-0.9], [0.6]])
    v_0 = np.array([[0.0], [0.0], [0.0]])
    a_0 = np.array([[0.0], [0.0], [0.0]])
    v_peak = np.array([[0.6], [0.3], [0.0]])
    t_peak = 3.0
    t_total = 6.0
    dt = 0.1

    # initialize the class
    spline_class = SplineByPeakSpeed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt)
    # generate the spline trajectory
    T, traj = spline_class.do_calculation()

    array_csv = np.vstack((T, traj[0:6, :]))
    print("This is csv.")
    print(array_csv)

    json_file = open("config_aimslab.json")
    config_dict = json.load(json_file)
    filename_csv = os.getcwd() + config_dict["DIRECTORY_TRAJ"] + "traj.csv"
    np.savetxt(filename_csv, array_csv, delimiter=",")
