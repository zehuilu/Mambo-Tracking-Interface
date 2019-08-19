
from owl import Context, Type


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
    owl.assignMarker(tracker_id, 128, "128", "pos=-55.475952,-9.052754,34.571442")
    owl.assignMarker(tracker_id, 129, "129", "pos=-10.989838,25.266884,-5.049560")
    owl.assignMarker(tracker_id, 130, "130", "pos=40.103027,-14.253139,57.084076")
    owl.assignMarker(tracker_id, 131, "131", "pos=-8.999756,-2.891779,25.866425")
    owl.assignMarker(tracker_id, 132, "132", "pos=57.705383,-12.928247,-37.412307")
    owl.assignMarker(tracker_id, 133, "133", "pos=-1.728638,0.530958,-26.282715")
    owl.assignMarker(tracker_id, 134, "134", "pos=-38.930176,-10.235787,-56.164154")
    owl.assignMarker(tracker_id, 135, "135", "pos=16.935577,23.811817,7.465760")

    # once you hit this point, the program will connect with the receiver and the lights should turn on
    owl.streaming(1)

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

                        print(px)
                        print(py)
                        print(pz)

    owl.done()
    owl.close()


if __name__ == "__main__":
    main_phasespace()
