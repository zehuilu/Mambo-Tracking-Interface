#!/usr/bin/env python

from owl import Context, Type
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import std_msgs.msg

def main():
    rospy.init_node('mocap_system')

    pub = rospy.Publisher('mocap', PoseStamped, queue_size=10)

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
    owl.assignMarker(tracker_id, 128, "128", "pos=-66.9839,16.7670,1.7574")
    owl.assignMarker(tracker_id, 129, "129", "pos=95.6185,-15.4433,31.1612")
    owl.assignMarker(tracker_id, 130, "130", "pos=7.1852,6.4807,26.9088")
    owl.assignMarker(tracker_id, 131, "131", "pos=-98.5255,-5.8215,-28.6273")
    owl.assignMarker(tracker_id, 132, "132", "pos=35.1649,7.7159,-2.9266")
    owl.assignMarker(tracker_id, 133, "133", "pos=-88.1575,3.1378,35.4779")
    owl.assignMarker(tracker_id, 133, "134", "pos=-0.9705,3.4561,-40.0624")
    owl.assignMarker(tracker_id, 135, "135", "pos=115.6436,-15.0623,-24.1802")

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
                        print(r.pose)
			
			pStamp = PoseStamped()

                        ret = Pose()
                        ret.position.x = r.pose[0]
                        ret.position.y = r.pose[1]
                        ret.position.z = r.pose[2]
                        ret.orientation.w = r.pose[3]
                        ret.orientation.x = r.pose[4]
                        ret.orientation.y = r.pose[5]
                        ret.orientation.z = r.pose[6]

			header = std_msgs.msg.Header()
                        header.stamp = rospy.Time.now()
                        header.frame_id = str(r.id)

			pStamp.header = header
			pStamp.pose = ret
                        
			pub.publish(pStamp)

    owl.done()
    owl.close()


if __name__ == "__main__":
    main()
