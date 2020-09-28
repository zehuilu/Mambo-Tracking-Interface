#!/usr/bin/env python3
import os
import sys
sys.path.append(os.getcwd()+'/lib')
import numpy as np
from SplineByPeakSpeed import SplineByPeakSpeed


if __name__ == "__main__":
    p_0 = np.array([[1.0], [1.0], [1.0]])
    v_0 = np.array([[0.0], [0.0], [0.0]])
    a_0 = np.array([[0.0], [0.0], [0.0]])
    v_peak = np.array([[0.5], [0.5], [0.5]])
    t_peak = 1.5
    t_total = 3.0
    dt = 0.1

    # initialize the class
    spline_class = SplineByPeakSpeed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt)
    # generate the spline trajectory
    T, traj = spline_class.do_calculation()

    print("This is T.")
    print(T)
    print("This is traj.")
    print(traj)