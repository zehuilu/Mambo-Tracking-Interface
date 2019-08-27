
import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

from owl import Context, Type
import socket
import numpy as np
import transforms3d
import time

def main_phasespace():

    # connect to NETGEAR48 and it'l be this IP
    address = "192.168.1.11"
    owl = Context()
    owl.open(address)
    owl.initialize()

    # how often the mocap system will send a "frame" that contains the body location
    # you can play with this, saturates the router around 1 kHz
    owl.frequency(50)

    tracker_id = 0
    owl.createTracker(tracker_id, "rigid", "myrigid")

    # args: tracker_id, marker_id, a name for the marker, marker local coordinates
    # keep in mind your coordinate system, in this case we're using x-z as the ground plane
    owl.assignMarker(tracker_id, 128, "128", "pos=-45.0577,-13.9892,52.0873")
    owl.assignMarker(tracker_id, 129, "129", "pos=-4.6958,20.2641,10.9212")
    #owl.assignMarker(tracker_id, 130, "130", "pos=42.2529,-17.7398,53.2606")
    #owl.assignMarker(tracker_id, 131, "131", "pos=-6.6991,-1.1539,28.1312")
    owl.assignMarker(tracker_id, 132, "132", "pos=57.7376,-17.0178,-25.6315")
    owl.assignMarker(tracker_id, 133, "133", "pos=1.6101,-2.5444,-12.6364")
    owl.assignMarker(tracker_id, 134, "134", "pos=-35.0395,-11.4603,-41.5566")
    owl.assignMarker(tracker_id, 135, "135", "pos=23.1157,24.2598,17.0532")

    # once you hit this point, the program will connect with the receiver and the lights should turn on
    owl.streaming(1)


###########################################
    # about socket
    HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
    PORT = 9000        # Port to listen on (non-privileged ports are > 1023)
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
                # print "rigids=", len(event.rigids))
                for r in event.rigids:
                    if r.cond > 0:
                        #print(r.pose)
                        
                        # position in controller frame, rotation in phasespace

                        #py = r.pose[0] # x in phasespace, y in controller
                        #pz = r.pose[1] # y in phasespace, z in controller
                        #px = r.pose[2] # z in phasespace, x in controller
                        #ori_w = r.pose[3]
                        #ori_x = r.pose[4]
                        #ori_y = r.pose[5]
                        #ori_z = r.pose[6]

                        #data = np.array([px, py, pz, ori_w, ori_x, ori_y, ori_z], dtype=float)

                        # mm to m
                        data = np.array([r.pose[2] / 1000.0, \
                            r.pose[0] / 1000.0, \
                            r.pose[1] / 1000.0, \
                            r.pose[3], r.pose[4], r.pose[5], r.pose[6]], dtype=float)

                        msg = data.tostring()
                        print("sending message to the client")
                        print(data)
                        connection.sendall(msg)

    print("not open or not initialized!")


    owl.done()
    owl.close()


if __name__ == "__main__":
    main_phasespace()
