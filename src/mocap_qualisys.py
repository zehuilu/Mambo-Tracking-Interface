#!/usr/bin/python3
import os
import sys
sys.path.append(os.getcwd()+'/lib')
import socket
import time
import json
import time
import numpy as np
import transforms3d

import asyncio
import xml.etree.ElementTree as ET
import pkg_resources
import qtm


def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6-DOF settings xml """
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


def publisher_tcp_main(config_data):
    """
    The following two lines show what is json_file_data
        json_file = open('mocap_config.json')
        json_file_data = json.load(json_file)
    """

    # IP for publisher
    HOST_TCP = config_data["QUALISYS"]["IP_STATES_ESTIMATION"]
    # Port for publisher
    PORT_TCP = int(config_data["QUALISYS"]["PORT_STATES_ESTIMATION"])

    server_address_tcp = (HOST_TCP, PORT_TCP)
    # Create a TCP/IP socket
    sock_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    return sock_tcp, server_address_tcp


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

    # Create a UDP socket for data streaming
    sock_tcp, server_address_tcp = publisher_tcp_main(config_data)

    # Bind the socket to the port
    sock_tcp.bind(server_address_tcp)

    # Listen for incoming connections
    sock_tcp.listen()

    # Wait for a connection
    print("waiting for a connection")
    print("If you're using it for mambo RTD, you can run the LLC now.")
    connection_tcp, client_address = sock_tcp.accept()
    print("Built connection with", client_address)

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

            # send 6-DOF data via TCP
            # concatenate the position and rotation matrix vertically
            msg = np.asarray((position.x/1000.0, position.y/1000.0, position.z/1000.0) + rotation.matrix + (t_now, ), dtype=float).tostring()



            # rotation.matrix is a tuple with 9 elements.
            
            # rotation_np = np.asarray(rotation.matrix, dtype=float).reshape(3, 3)
            # quat = transforms3d.quaternions.mat2quat(rotation_np)
            # data = np.array([position.x/1000.0, position.y/1000.0, position.z/1000.0, quat[0], quat[1], quat[2], quat[3], t_now], dtype=float)
            # msg = data.tostring()


            connection_tcp.sendall(msg)
            # print("6-DOF data sent via TCP!")

            # # for debugging
            # # remember to import math
            # print("rotation matrix in array")
            # print(rotation.matrix)
            # print("rotation matrix in matrix")
            # print(rotation_np)
            # roll_now, pitch_now, yaw_now = transforms3d.euler.quat2euler(quat, axes='sxyz')
            # print("yaw")
            # print(math.degrees(yaw_now))
            # print("pitch")
            # print(math.degrees(pitch_now))
            # print("roll")
            # print(math.degrees(roll_now))

        
        else:
            # error
            raise Exception("There is no such a rigid body!")

    # Start streaming frames
    # Make sure the component matches with the data fetch function, for example: packet.get_6d() with "6d"
    # Reference: https://qualisys.github.io/qualisys_python_sdk/index.html
    await connection.stream_frames(components=["6d"], on_packet=on_packet)


if __name__ == "__main__":
    config_file_name = 'config.json'
    # Run our asynchronous main function forever
    asyncio.ensure_future(main(config_file_name))
    asyncio.get_event_loop().run_forever()