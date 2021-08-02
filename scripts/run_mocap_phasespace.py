#!/usr/bin/env python3
import socket
import numpy as np
import transforms3d
import time
import json
import pathmagic
with pathmagic.context():
    from owl import Context, Type


def main_phasespace(config_file_name):
    # Read the configuration from the json file
    json_file = open(config_file_name)
    config_data = json.load(json_file)

    # connect to NETGEAR48 and it'l be this IP
    address = config_data["PHASESPACE"]["IP_MOCAP_SERVER"]
    owl = Context()
    owl.open(address)
    owl.initialize()

    # how often the mocap system will send a "frame" that contains the body location
    # you can play with this, saturates the router around 1 kHz
    owl.frequency(int(config_data["PHASESPACE"]["FREQUENCY_MOCAP"]))
    # once you hit this point, the program will connect with the receiver and the lights should turn on
    owl.streaming(1)


###########################################
    # about socket
    # Standard loopback interface address (localhost)
    HOST = config_data["PHASESPACE"]["IP_STATES_ESTIMATION"]
    # Port to listen on (non-privileged ports are > 1023)
    PORT = int(config_data["PHASESPACE"]["PORT_STATES_ESTIMATION"])
    server_address = (HOST, PORT)

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the port
    print("starting up on", server_address)
    sock.bind(server_address)

    # Listen for incoming connections
    sock.listen()

    # Wait for a connection
    print("waiting for a connection")
    print("If you're using it for mambo RTD, you can run the LLC now.")
    connection, client_address = sock.accept()
    print("connection from", client_address)


    while owl.isOpen() and owl.property("initialized"):
        event = owl.nextEvent(1000)
        if not event:
            continue
        if event.type_id == Type.ERROR:
            pass
            # print event.name, ": ", event.str
        elif event.type_id == Type.FRAME:
            if "rigids" in event:
                t_now = time.time()
                r = event.rigids[int(config_data["PHASESPACE"]["INDEX_MAMBO"])]
                if r.cond > 0:
                    #print(r.pose)
                    
                    # position in phasespace frame
                    px = r.pose[0] / 1000.0 # x in phasespace
                    py = r.pose[1] / 1000.0 # y in phasespace
                    pz = r.pose[2] / 1000.0 # z in phasespace
                    #ori_w = r.pose[3]
                    #ori_x = r.pose[4]
                    #ori_y = r.pose[5]
                    #ori_z = r.pose[6]

                    #data = np.array([px, py, pz, ori_w, ori_x, ori_y, ori_z], dtype=float)

                    # mm to m
                    data = np.array([px, py, pz, r.pose[3], r.pose[4], r.pose[5], r.pose[6], t_now], dtype=float)

                    msg = data.tobytes()
                    print("sending message to the client")
                    print(data)
                    connection.sendall(msg)

    print("not open or not initialized!")

    owl.done()
    owl.close()


if __name__ == "__main__":
    config_file_name = 'config_roahmlab.json'
    main_phasespace(config_file_name)
