#!/usr/bin/env python3
import pathmagic
with pathmagic.context(EXTERNAL_FLAG=True):
    from Simulator import Simulator


class SimulatorAimsLab(Simulator):


    def __init__(self, map_resolution: int):
        # configuration for the simulator of AIMS LAB
        map_width_meter = 3.4
        map_height_meter = 4.8 
        value_non_obs = 0  # the cell is empty
        value_obs = 255  # the cell is blocked

        Simulator.__init__(self, map_width_meter, map_height_meter, map_resolution, value_non_obs, value_obs)

        # generate obstacles manually
        # self.map_array[5][6] = self.value_obs

        # generate random obstacles
        self.generate_random_obs(30, [0.2,0.2])


