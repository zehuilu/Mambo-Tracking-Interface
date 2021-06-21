#!/usr/bin/env python3
import numpy as np
from scipy import interpolate


def discrete_path_to_time_traj(path: list, dt: float, velocity_ave: float):
    # 1d array, each element is the distance between the current node position and the previou node position
    distance_vec = np.sum(np.abs(np.diff(path, axis=0))**2, axis=-1) ** 0.5
    # 1d array, each element is the duration between two adjacent nodes
    duration_vec = distance_vec / velocity_ave

    # time stamp vector for input path
    time_vec = np.zeros(len(path))
    for idx in range(1, len(path)):
        time_vec[idx] = time_vec[idx-1] + duration_vec[idx-1]

    # generate a queue time trajectory, the time interval is dt
    time_queue_vec = np.arange(time_vec[0], time_vec[-1], dt).tolist()
    if not (time_vec[-1] - dt).is_integer():
        time_queue_vec.append(time_vec[-1])

    print("path")
    print(path)

    print("distance_vec")
    print(distance_vec)

    print("duration_vec")
    print(duration_vec)

    print("time_vec")
    print(time_vec)

    print("time_queue_vec")
    print(time_queue_vec)

    boundary = (path[0], path[-1])
    f = interpolate.interp1d(time_vec, path, kind='linear',
        bounds_error=False, fill_value=boundary, axis=0)

    # each row is a instance of position
    position_traj = f(time_queue_vec)

    print("position_traj")
    print(position_traj)

    # now is about estimate the velocity


    



    return time_queue_vec, position_traj


if __name__ == "__main__":
    path = [[1,2], [2,5], [3,8], [5,10], [12,12]]
    dt = 0.1
    velocity_ave = 0.5

    discrete_path_to_time_traj(path, dt, velocity_ave)
