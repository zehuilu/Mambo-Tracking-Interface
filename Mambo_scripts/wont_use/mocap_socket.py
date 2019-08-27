
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
    owl.assignMarker(tracker_id, 128, "128", "pos=-53.1304,-10.0298,40.5306")
    owl.assignMarker(tracker_id, 129, "129", "pos=-10.1753,23.6635,-3.2948")
    owl.assignMarker(tracker_id, 130, "130", "pos=42.2008,-11.9643,51.7040")
    owl.assignMarker(tracker_id, 131, "131", "pos=-7.9056,-1.9095,26.1111")
    owl.assignMarker(tracker_id, 132, "132", "pos=52.5650,-13.7188,-40.6436")
    owl.assignMarker(tracker_id, 133, "133", "pos=0.0916,5.0685,-29.0841")
    owl.assignMarker(tracker_id, 134, "134", "pos=-40.3587,-14.5233,-50.6817")
    owl.assignMarker(tracker_id, 135, "135", "pos=16.7125,23.4146,5.3593")

    # once you hit this point, the program will connect with the receiver and the lights should turn on
    owl.streaming(1)




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
			
                        px = r.pose[0]
                        py = r.pose[1]
                        pz = r.pose[2]
                        ori_w = r.pose[3]
                        ori_x = r.pose[4]
                        ori_y = r.pose[5]
                        ori_z = r.pose[6]

                        quaternion = (ori_w, ori_x, ori_y, ori_z)
                        yaw, pitch, roll = transforms3d.euler.quat2euler(quaternion, axes='syzx')

                        data = np.array([px, py, pz, yaw, pitch, roll], dtype=float)
                        msg = data.tostring()
                        print("sending message to the client")
                        print(data)
                        connection.sendall(msg)

    print("not open or not initialized!")


    owl.done()
    owl.close()


if __name__ == "__main__":
    main_phasespace()
