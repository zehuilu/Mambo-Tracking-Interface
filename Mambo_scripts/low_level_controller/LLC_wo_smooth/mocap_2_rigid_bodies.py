
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
    # this is for No.2, Mambo_628236
    owl.assignMarker(tracker_id, 120, "120", "pos=-52.5542,-10.1185,40.2710")
    #owl.assignMarker(tracker_id, 121, "121", "pos=-6.9779,23.4601,6.1246") no.121 is dead
    owl.assignMarker(tracker_id, 122, "122", "pos=42.8047,-11.8003,48.6087")
    owl.assignMarker(tracker_id, 123, "123", "pos=-7.8428,-0.8559,25.6734")
    owl.assignMarker(tracker_id, 124, "124", "pos=50.5558,-10.8666,-43.8378")
    owl.assignMarker(tracker_id, 125, "125", "pos=-4.0293,-0.4749,-26.6466")
    owl.assignMarker(tracker_id, 126, "126", "pos=-43.0056,-14.5414,-52.1294")
    owl.assignMarker(tracker_id, 127, "127", "pos=21.0530,25.1987,1.9329")
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
                r = event.rigids[1]
                if r.cond > 0:
                    #print(r.pose)
                        
                    # position in phasespace frame
                    #px = r.pose[0] # x in phasespace
                    #py = r.pose[1] # y in phasespace
                    #pz = r.pose[2] # z in phasespace
                    #ori_w = r.pose[3]
                    #ori_x = r.pose[4]
                    #ori_y = r.pose[5]
                    #ori_z = r.pose[6]

                    #data = np.array([px, py, pz, ori_w, ori_x, ori_y, ori_z], dtype=float)

                    # mm to m
                    data = np.array([r.pose[0] / 1000.0, \
                        r.pose[1] / 1000.0, \
                        r.pose[2] / 1000.0, \
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
