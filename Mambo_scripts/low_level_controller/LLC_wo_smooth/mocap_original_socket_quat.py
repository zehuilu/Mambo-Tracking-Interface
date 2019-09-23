
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
    owl.assignMarker(tracker_id, 128, "128", "pos=-52.3767,-8.7786,40.7680")
    owl.assignMarker(tracker_id, 129, "129", "pos=-7.6397,25.7523,-1.2879")
    owl.assignMarker(tracker_id, 130, "130", "pos=41.9219,-17.9627,50.4792")
    owl.assignMarker(tracker_id, 131, "131", "pos=-6.9976,-2.5442,26.2124")
    owl.assignMarker(tracker_id, 132, "132", "pos=50.9988,-15.4777,-41.9002")
    owl.assignMarker(tracker_id, 133, "133", "pos=-3.4792,3.3021,-27.9958")
    owl.assignMarker(tracker_id, 134, "134", "pos=-41.7818,-8.1816,-52.0809")
    owl.assignMarker(tracker_id, 135, "135", "pos=19.3542,23.8872,5.8064")

    a = '''
    # this is for No.1, roahm_mambo_1
    owl.assignMarker(tracker_id, 136, "136", "pos=-3.4676,2.6817,-26.6443")
    owl.assignMarker(tracker_id, 137, "137", "pos=-7.7717,-3.3691,28.0087")
    owl.assignMarker(tracker_id, 138, "138", "pos=-55.3168,-8.7290,37.2145")
    owl.assignMarker(tracker_id, 139, "139", "pos=41.6391,-12.2640,53.2985")
    owl.assignMarker(tracker_id, 140, "140", "pos=53.6591,-12.5237,-39.1312")
    owl.assignMarker(tracker_id, 141, "141", "pos=-5.9832,22.6972,-4.1788")
    owl.assignMarker(tracker_id, 142, "142", "pos=-38.0154,-10.9053,-54.9944")
    owl.assignMarker(tracker_id, 143, "143", "pos=16.3519,23.5310,6.7474")
    '''

    
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
                for r in event.rigids:
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
