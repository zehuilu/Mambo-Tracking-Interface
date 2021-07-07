#!/usr/bin/python3
import os
import sys
sys.path.append(os.getcwd()+'/lib')
import time
import json
import time
import numpy as np
import transforms3d

import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import qtm

import math


def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6-DOF settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


async def main(config_file_name):
    """ Main function """
    # Read the configuration from the json file
    json_file = open(config_file_name)
    config_data = json.load(json_file)

    # IP address for the mocap server
    IP_server = config_data["QUALISYS"]["IP_MOCAP_SERVER"]

    # Connect to qtm
    connection = await qtm.connect(IP_server)

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    async with qtm.TakeControl(connection, "password"):
        pass

    # Get 6-DOF settings from QTM
    xml_string = await connection.get_parameters(parameters=["6d"])

    # parser for mocap rigid bodies indexing
    body_index = create_body_index(xml_string)
    # the one we want to access
    wanted_body = config_data["QUALISYS"]["NAME_SINGLE_BODY"]

    def on_packet(packet):
        # Get the 6-DOF data
        bodies = packet.get_6d()[1]

        if wanted_body is not None and wanted_body in body_index:
            # Extract one specific body
            t_now = time.time()
            wanted_index = body_index[wanted_body]
            position, rotation = bodies[wanted_index]
            # You can use position and rotation here. Notice that the unit for position is mm!
            # print(wanted_body)

            # rotation.matrix is a tuple with 9 elements.
            rotation_np = np.asarray(rotation.matrix, dtype=float).reshape(3, 3)

            # send 6-DOF data via TCP
            # concatenate the position and rotation matrix vertically
            # msg = np.asarray((position.x/1000.0, position.y/1000.0, position.z/1000.0) + rotation.matrix, dtype=float).tostring()

            quat = transforms3d.quaternions.mat2quat(rotation_np)
            # print("quat")
            # print(quat)

            data = np.array([position.x/1000.0, position.y/1000.0, position.z/1000.0, quat[0], quat[1], quat[2], quat[3], t_now], dtype=float)

            # for debugging
            print("rotation matrix in array")
            print(rotation.matrix)
            print("rotation matrix in matrix")
            print(rotation_np)

            roll_now, pitch_now, yaw_now = transforms3d.euler.quat2euler(quat, axes='sxyz')
            print("yaw")
            print(math.degrees(yaw_now))
            print("pitch")
            print(math.degrees(pitch_now))
            print("roll")
            print(math.degrees(roll_now))
        
        else:
            # error
            raise Exception("There is no such a rigid body!")

    # Start streaming frames
    # Make sure the component matches with the data fetch function, for example: packet.get_6d() with "6d"
    # Reference: https://qualisys.github.io/qualisys_python_sdk/index.html
    await connection.stream_frames(components=["6d"], on_packet=on_packet)


if __name__ == "__main__":
    config_file_name = 'config_aimslab.json'
    # Run our asynchronous main function forever
    asyncio.ensure_future(main(config_file_name))
    asyncio.get_event_loop().run_forever()