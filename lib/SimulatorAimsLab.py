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

    def qualisys_to_map_index(self, position_qualisys: list):
        """
        Transform a single qualisys coordinate (meter) to a map array (index) coordinate.
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            position_qualisys: [px0,py0,pz0]

        Output:
            position_index: [x0, y0]
        """
        # qualisys coordinate to map array coordinate (meter)
        positions_meter = coord_qualisys.qualisys_to_map_meter(position_qualisys)
        # map array coordinate (meter) to map array coordinate (index)
        position_index = self.position_to_map_index(positions_meter[:-1])  # 2D, no height
        return position_index

    def qualisys_to_map_index_all(self, positions_qualisys: list):
        """
        Transform a list of qualisys coordinates (meter) to the map array (index) coordinates.
        NOTE: to speed up the transformation over a set of coordinates, a vectorization function can be defined.

        Input:
            positions_qualisys: [[px0,py0,pz0], [px1,py1,pz1], ...]

        Output:
            positions_index: [x0,y0, x1,y1, x2,y2, ...]
        """
        # # qualisys coordinate to map array coordinate (meter)
        # positions_meter = []
        # for idx in range(len(positions_qualisys)):
        #     positions_meter.append(coord_qualisys.qualisys_to_map_meter(positions_qualisys[idx]))

        # # map array coordinate (meter) to map array coordinate (index)
        # positions_index = []
        # for idx in range(len(positions_meter)):
        #     position_meter_now = positions_meter[idx]
        #     positions_index.extend(self.position_to_map_index(position_meter_now[:-1]))  # 2D, no height

        positions_index = list()
        for idx in range(len(positions_qualisys)):
            positions_index.extend(self.qualisys_to_map_index(positions_qualisys[idx]))
        return positions_index

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
        path_qualisys_all = list()
        for idx in range(len(path_index_all)):
            path_qualisys_now = self.path_index_to_qualisys(path_index_all[idx], heights_all[idx])
            path_qualisys_all.append(path_qualisys_now)
        return path_qualisys_all

    def update_obs_map(self, position_obs_qualisys: list, size_obs_qualisys: list):
        """
        Update the map based on a single obstacle's position and its size, assuming the obstacle is a rectangular.

        Input:
            position_obs_qualisys: [px, py, pz] (meter)
            size_obs_qualisys: [x, y] (meter)
                size x is the height in the map, and size y is the width in the map.
        """
        # obs_center_index = [height, width]
        obs_center_index = self.qualisys_to_map_index(position_obs_qualisys)

        # the half number of index cells in each direction
        half_width_index = max(int(size_obs_qualisys[1]*self.resolution/2), 1)
        half_height_index = max(int(size_obs_qualisys[0]*self.resolution/2), 1)

        # the range of index in each direction
        min_height_index = max(obs_center_index[0]-half_height_index, 0)
        max_height_index = min(obs_center_index[0]+half_height_index, self.map_height-1)

        min_width_index = max(obs_center_index[1]-half_width_index, 0)
        max_width_index = min(obs_center_index[1]+half_width_index, self.width-1)

        # update the map
        self.map_array[min_height_index : max_height_index+1][:, min_width_index : max_width_index+1] = self.value_obs * np.ones((max_height_index-min_height_index+1, max_width_index-min_width_index+1))
