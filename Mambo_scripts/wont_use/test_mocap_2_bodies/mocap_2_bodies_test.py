
import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

from owl import Context, Type
import socket
import numpy as np
import transforms3d
import time


def saturation(x_raw, threshold):
    # x_raw is 1-D numpy array
    for idx in np.arange(x_raw.shape[0]):
        if abs(x_raw[idx]) >= threshold:
            x_raw[idx] = 0.0
    return x_raw


def main_phasespace():
    p_mambo_pre = np.array([0.0, 0.0, 0.0], dtype=float)
    p_wand_pre = np.array([0.0, 0.0, 0.0], dtype=float)
    v_mambo_pre = np.array([0.0, 0.0, 0.0], dtype=float)
    v_wand_pre = np.array([0.0, 0.0, 0.0], dtype=float)
    dt_mocap = 0.05
    threshold_velo_wand = 2.5
    threshold_velo_mambo = 4.0

##########################################################
    # connect to NETGEAR48 and it'l be this IP
    address = "192.168.1.11"
    owl = Context()
    owl.open(address)
    owl.initialize()

    # how often the mocap system will send a "frame" that contains the body location
    # you can play with this, saturates the router around 1 kHz
    owl.frequency(50)

##########################################################
    tracker_id = 0
    owl.createTracker(tracker_id, "rigid", "myrigid")

    # args: tracker_id, marker_id, a name for the marker, marker local coordinates
    # keep in mind your coordinate system, in this case we're using x-z as the ground plane
    owl.assignMarker(tracker_id, 128, "128", "pos=-52.0599,-10.4392,40.0173")
    owl.assignMarker(tracker_id, 129, "129", "pos=-8.5237,25.4002,-3.0452")
    owl.assignMarker(tracker_id, 130, "130", "pos=41.5276,-13.3359,51.4979")
    owl.assignMarker(tracker_id, 131, "131", "pos=-7.1305,-1.7352,25.9688")
    owl.assignMarker(tracker_id, 132, "132", "pos=51.8760,-14.8687,-39.9762")
    owl.assignMarker(tracker_id, 133, "133", "pos=-3.8685,1.1530,-26.6558")
    owl.assignMarker(tracker_id, 134, "134", "pos=-40.9397,-11.5180,-52.5310")
    owl.assignMarker(tracker_id, 135, "135", "pos=18.7742,24.8969,5.4536")
##########################################################

##########################################################
    tracker_id = 1
    owl.createTracker(tracker_id, "rigid", "myrigid")

    # args: tracker_id, marker_id, a name for the marker, marker local coordinates
    # keep in mind your coordinate system, in this case we're using x-z as the ground plane
    owl.assignMarker(tracker_id, 136, "136", "pos=-35.8766,1.0347,38.5895")
    owl.assignMarker(tracker_id, 137, "137", "pos=-0.7117,5.6810,1.0003")
    owl.assignMarker(tracker_id, 138, "138", "pos=-41.0381,1.5283,-39.2787")
    owl.assignMarker(tracker_id, 139, "131", "pos=37.1385,-4.8904,-38.4707")
    owl.assignMarker(tracker_id, 143, "143", "pos=40.4321,-3.4139,38.2635")

    # once you hit this point, the program will connect with the receiver and the lights should turn on
    owl.streaming(1)
##########################################################


##########################################################
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
##########################################################

    t1 = time.time()
    time.sleep(dt_mocap)

    while owl.isOpen() and owl.property("initialized"):
        event = owl.nextEvent(1000)
        t0 = time.time()
        dt = t0 - t1
        print(dt)
        if not event:
            continue
        if event.type_id == Type.ERROR:
            pass
            # print event.name, ": ", event.str
        elif event.type_id == Type.FRAME:
            if "rigids" in event:
                #print("rigids=")
                #print(len(event.rigids))

                mambo = event.rigids[1]
                wand = event.rigids[3]
                # the order of rigids are shown in phasespace client, shown as "ID"
                if mambo.cond > 0 and wand.cond > 0:
                    # this is mambo
                    #data_mambo = np.array([mambo.pose[0] / 1000.0, mambo.pose[1] / 1000.0, mambo.pose[2] / 1000.0, mambo.pose[3], mambo.pose[4], mambo.pose[5], mambo.pose[6]], dtype=float)
                    # this is wand/crazyflie
                    #data_wand = np.array([wand.pose[0] / 1000.0, wand.pose[1] / 1000.0, wand.pose[2] / 1000.0, wand.pose[3], wand.pose[4], wand.pose[5], wand.pose[6]], dtype=float)
                    quat_mambo = np.array(mambo.pose[3:], dtype=float)
                    quat_wand = np.array(wand.pose[3:], dtype=float)

                    p_mambo = np.array([mambo.pose[0] / 1000.0, mambo.pose[1] / 1000.0, mambo.pose[2] / 1000.0], dtype=float)
                    p_wand = np.array([wand.pose[0] / 1000.0, wand.pose[1] / 1000.0, wand.pose[2] / 1000.0], dtype=float)
                    v_mambo = (p_mambo - p_mambo_pre) / dt
                    v_wand = (p_wand - p_wand_pre) / dt
                    a_mambo = (v_mambo - v_mambo_pre) / dt
                    a_wand = (v_wand - v_wand_pre) / dt

                    #################################################
                    #v_mambo = saturation(v_mambo, threshold_velo_mambo)
                    #v_wand = saturation(v_wand, threshold_velo_wand)
                    #################################################

                    p_mambo_pre = p_mambo
                    p_wand_pre = p_wand
                    v_mambo_pre = v_mambo
                    v_wand_pre = v_wand

                    data = np.concatenate((p_mambo, v_mambo, a_mambo, quat_mambo, p_wand, v_wand, a_wand, quat_wand), axis=0)
                    print(data)
                    msg = data.tostring()
                    connection.sendall(msg)
                else:
                    print("Mambo/wand's LEDs are not powered on!")

        t1 = time.time()
        time.sleep(dt_mocap)

        
                        

    print("not open or not initialized!")


    owl.done()
    owl.close()


if __name__ == "__main__":
    main_phasespace()
