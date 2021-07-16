#!/usr/bin/env python3
import numpy as np
from itertools import chain
import coordinate_transformation_qualisys as coord_qualisys
import pathmagic
with pathmagic.context(EXTERNAL_FLAG=True):
    from Simulator import Simulator


class SimulatorAimsLab(Simulator):
    def __init__(self, map_resolution: int):
        # configuration for the simulator of AIMS LAB
        map_width_meter = 3.4  # Y-axis in qualisys coordinates, details in coordinate_transformation_qualisys.py
        map_height_meter = 4.8  # X-axis in qualisys coordinates, details in coordinate_transformation_qualisys.py
        value_non_obs = 0  # the cell is empty
        value_obs = 255  # the cell is blocked

        Simulator.__init__(self, map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)

        # generate obstacles manually
        # self.map_array[5][6] = self.value_obs

        # generate random obstacles
        self.generate_random_obs(30, [0.2,0.2])

    def qualisys_to_map_index_all(self, agents_positions_qualisys: list, targets_positions_qualisys: list):
        """
        Transform the qualisys coordinates (meter) of agents and targets to the map array (index) coordinates.
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            agents_positions_qualisys: [[px0,py0,pz0], [px1,py1,pz1], ...]
            targets_positions_qualisys: [[px0,py0,pz0], [px1,py1,pz1], ...]

        Output:
            agents_positions_index: [x0,y0, x1,y1, x2,y2, ...]
            targets_positions_index: [x0,y0, x1,y1, x2,y2, ...]
        """
        # qualisys coordinate to map array coordinate (meter)
        agents_positions_meter = []
        for idx in range(len(agents_positions_qualisys)):
            agents_positions_meter.append(coord_qualisys.qualisys_to_map_meter(agents_positions_qualisys[idx]))

        targets_positions_meter = []
        for idx in range(len(targets_positions_qualisys)):
            targets_positions_meter.append(coord_qualisys.qualisys_to_map_meter(targets_positions_qualisys[idx]))

        # map array coordinate (meter) to map array coordinate (index)
        agents_positions_index = []
        for idx in range(len(agents_positions_meter)):
            agent_position_meter_now = agents_positions_meter[idx]
            agents_positions_index.extend(self.position_to_map_index(agent_position_meter_now[:-1]))  # 2D, no height

        targets_positions_index = []
        for idx in range(len(targets_positions_meter)):
            target_position_meter_now = targets_positions_meter[idx]
            targets_positions_index.extend(self.position_to_map_index(target_position_meter_now[:-1]))  # 2D, no height

        return agents_positions_index, targets_positions_index

    def path_index_to_qualisys(self, path_index: list, height: float):
        """
        Transform the path index coordinates of a single agent to the qualisys coordinates (meter).
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            path_index: [[x0,y0, x1,y1, ..., x2,y2], [x2,y2, x3,y3, ..., x4,y4], [x4,y4, ..., x5,y5]]
                each sub-list is a part of path, where the last two elements (position indices) are the same with the first two elements of next sub-list.
            height: the height of the original coordinates

        Output:
            path_qualisys.tolist(): a 2D list [[x0,y0,z0], [x1,y1,z1], ..., [x2,y2,z2]] as a discrete path
                each sub-list is a qualisys coordinate as a single path point
        """
        # delete the duplicated ones.
        for idx in range(len(path_index)-1):
            if path_index[idx]:
                del path_index[idx][-1]
                del path_index[idx][-1]

        # flatten the path list, and reshape it to N by #dimension array
        path_index_1d = list(chain.from_iterable(path_index))
        path_index_2d = np.reshape(path_index_1d, (-1,2))

        # map array coordinate (index) to map array coordinate (meter)
        path_meter_2d = np.zeros(path_index_2d.shape)
        for idx in range(path_index_2d.shape[0]):
            path_meter_2d[idx] = self.map_index_to_position(path_index_2d[idx])

        # stack the constant height in meter
        height_array = height * np.ones((path_meter_2d.shape[0], 1))
        path_meter_3d = np.hstack((path_meter_2d, height_array))

        # map array coordinate (meter) to qualisys coordinate
        path_qualisys = np.zeros(path_meter_3d.shape)
        for idx in range(path_meter_3d.shape[0]):
            path_qualisys[idx] = coord_qualisys.map_meter_to_qualisys(path_meter_3d[idx])

        return path_qualisys.tolist()

    def path_index_all_to_qualisys(self, path_index_all: list, heights_all: list):
        """
        Transform the path index coordinates of multiple agents to the qualisys coordinates (meter).
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            path_index_all: [[[x0,y0,...,x1,y1],[x1,y1,...,x2,y2],[x2,y2,...,x3,y3]], ..., [[x0,y0,...,x1,y1],[x1,y1,...,x2,y2],[x2,y2,...,x3,y3]], ...]
                each sub-list is a path of an agent, meaning path_index = path_index_all[0]
            heights_all: a list of heights of multiple agents

        Output:
            path_qualisys_all: a 3D list for the discrete paths of multiple agents, path_qualisys = path_qualisys_all[0]
        """
        path_qualisys_all = []
        for idx in range(len(path_index_all)):
            path_qualisys_now = self.path_index_to_qualisys(path_index_all[idx], heights_all[idx])
            path_qualisys_all.append(path_qualisys_now)

        return path_qualisys_all
