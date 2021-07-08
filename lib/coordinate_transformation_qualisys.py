#!/usr/bin/env python3
import numpy as np

"""
This file contains two coordinate transformation functions with a configuration of AIMS LAB.
The height coordinate (Z) doesn't change and always points up in any coordinates.

The map array coordinate (meter) has an origin at the left bottom corner. Positive-X points right and positive-Y points forwards. (East-North-Up)
The qualisys coordinate's Positive-X points forwards and positive-Y points right. (North-West-Up)

The qualisys coordinate is shown below.
(+2.4, +1.6, 0)------------------(+2.4, -1.8, 0)
|                                              |
|                                              |
|                                              |
|                (0, 0, 0)                     |
|                                              |
|                                              |
|                                              |
|                                              |
|                                              |
(-2.4, +1.6, 0)------------------(-2.4, -1.8, 0)

"""


def qualisys_to_map_meter(position_qualisys: list):
    """
    Translate a qualisys coordinate to a map array coordinate (meter).
    In the map array coordinate, the left bottom corner is the origin.
    """
    position_map_meter = [-position_qualisys[1]+1.6, position_qualisys[0]+2.4, position_qualisys[2]]
    return position_map_meter

def map_meter_to_qualisys(position_map_meter: list):
    """
    Translate a map array coordinate (meter) to a qualisys coordinate.
    In the map array coordinate, the left bottom corner is the origin.
    """
    position_qualisys = [position_map_meter[1]-2.4, -position_map_meter[0]+1.6, position_map_meter[2]]
    return position_qualisys


if __name__ == "__main__":
    position_qualisys = [0.2, -0.3, 0.6]
    position_map_meter = qualisys_to_map_meter(position_qualisys)
    print("position_map_meter")
    print(position_map_meter)

    position_map_meter = [1.0, 3.1, 0.6]
    position_qualisys = map_meter_to_qualisys(position_map_meter)
    print("position_qualisys")
    print(position_qualisys)
