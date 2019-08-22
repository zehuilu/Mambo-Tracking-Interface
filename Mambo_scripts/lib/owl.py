"""
libowl2 API Python Implementation
PhaseSpace, Inc. 2015

Example 1 (command-line, see __main__):
    python owl.py -h
    python owl.py --device localhost --timeout 5000000

Example 2 (python):
    #!/usr/bin/python
    OWL = Context()
    OWL.open("localhost")
    OWL.initialize()

    OWL.streaming(1)

    while OWL.isOpen() and OWL.property("initialized"):
        event = OWL.nextEvent()
        if not event: continue
        if event.type_id == Type.CAMERA:
            for camera in event.data:
                print((camera))
        elif event.type_id == Type.FRAME:
            if "markers" in event:
                for m in event.markers:
                    if m.cond > 0: print((m))
            if "rigids" in event:
                for r in event.rigids:
                    if r.cond > 0: print((r))
        elif event.type_id == Type.ERROR or event.name == "done":
            break
    OWL.done()
    OWL.close()
"""

OWL_PROTOCOL_VERSION = "2"
LIBOWL_REV = "5.1.279"

__version__ = LIBOWL_REV

import sys
import time
import struct
import socket
import errno
import select
import math
import collections
import re
import ctypes

if sys.version_info.major <= 2 and sys.version_info.minor <= 7 and sys.version_info.micro < 6:
    raise Exception("minimum python version 2.7.6 is required")

if sys.version_info.major >= 3:
    import urllib.parse
    urlunquote = urllib.parse.unquote
    pass
elif sys.version_info.major >= 2:
    import urllib
    urlunquote = urllib.unquote
    pass

class Type:
    INVALID = 0
    BYTE = 1
    STRING = BYTE
    INT = 2
    FLOAT = 3
    ERROR = 0x7F
    EVENT = 0x80
    FRAME = EVENT
    CAMERA = 0x81
    PEAK = 0x82
    PLANE = 0x83
    MARKER = 0x84
    RIGID = 0x85
    INPUT = 0x86
    MARKERINFO = 0x87
    TRACKERINFO = 0x88
    FILTERINFO = 0x89
    DEVICEINFO = 0x8A
    pass

class OWLError(Exception):
    def __init__(self, s = None):
        if s != None: Exception.__init__(self, s)
        pass
    pass

class RecvError(OWLError):
    def __init__(self, s = None):
        if s != None: OWLError.__init__(self, s)
        pass
    pass

class SendError(OWLError):
    def __init__(self, s = None):
        if s != None: OWLError.__init__(self, s)
        pass
    pass

class OpenError(OWLError):
    def __init__(self, s = None):
        if s != None: OWLError.__init__(self, s)
        pass
    pass

class InitError(OWLError):
    def __init__(self, s = None):
        if s != None: OWLError.__init__(self, s)
        pass
    pass

class Event():
    """
The primary data structure returned by the server. An Event instance stores
metadata and data from the capture hardware in its dictionary.
Frame-synced data will arrive as members inside a FRAME event.
Unsynchronized data will arrive as independent events.

Notably,  system errors are returned asynchronously from the server as Events.

Members:
    type_id = Event type as enumerated in Type
    id = Session dependent ID of the event
    flags = See PhaseSpace documentation
    time = System time of data
    type_name = Name of event type
    name = Name of event

Example:
    event = context.nextEvent()
    if event.type_id == Type.CAMERA:
        for c in event.data:
            print(c)
    if event.type_id == Type.FRAME:
        if "markers" in event:
            for m in event.markers:
                print(m)
    if event.type_id == Type.ERROR:
        raise OWLError("ERROR")
    """
    def __init__(self, type_id, id, flags=0, time=0, type_name=None, name=None, **kwargs):
        self.type_id = type_id
        self.id = id
        self.flags = flags
        self.time = time
        self.type_name = type_name
        self.name = name
        for k,v in kwargs.items(): self[k] = v
        pass
    def __str__(self):
        exclude = ["type_id", "id", "flags", "time", "type_name", "name"]
        keys = filter(lambda x: x not in exclude, self.__dict__.keys())
        keys = ",".join(keys)
        return "Event(type_id={} id={}, flags={}, time={}, type_name=\"{}\", name=\"{}\" keys={})".format(self.type_id, self.id, self.flags, self.time, self.type_name, self.name, keys)

    def __getitem__(self, key):
        return self.__dict__[key]
    def __setitem__(self, key, item):
        self.__dict__[key] = item
        pass
    def __contains__(self, key):
        return key in self.__dict__
    pass


class Camera():
    """
A data structure representing a single calibrated camera.

Members:
    id = Numeric ID of the camera
    flags = See PhaseSpace hardware documentation
    pose = 3D position and quaternion rotation of the camera (x, y, z, s, a, b ,c)
    cond = Condition number of the camera.  <=0 may mean the camera is decalibrated or missing.
    """
    def __init__(self, id, flags=0, pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], cond=-1.0):
        self.id = id
        self.flags = flags
        self.pose = pose
        self.cond = cond
        pass
    def __str__(self):
        return "Camera(id={}, flags={}, pose={}, cond={})".format(self.id, self.flags, self.pose, self.cond)
    pass

class Peak():
    """
(Advanced users only)
A data structure representing a raw peak as captured by the camera hardware

Members:
    id = Numeric ID of the peak
    flags = See PhaseSpace hardware documentation
    time = System time when the data was captured
    camera = ID of the camera this peak came from
    detector = Detector ID of the camera this peak came from
    width = Width of the peak, in pixels
    pos = Normalized position of the peak on the detector
    amp = Amplitude of the peak
    """
    __slots__ = ['id', 'flags', 'time', 'camera', 'detector', 'width', 'pos', 'amp']
    def __init__(self, id, flags=0, time=0, camera=0, detector=0, width=0, pos=0.0, amp=0.0):
        self.id = id
        self.flags = flags
        self.time = time
        self.camera = camera
        self.detector = detector
        self.width = width
        self.pos = pos
        self.amp = amp
        pass
    def __str__(self):
        return "Peak(id={}, flags={}, time={}, camera={}, detector={}, width={}, pos={}, amp={})".format(self.id, self.flags, self.time, self.camera, self.detector, self.width, self.pos, self.amp)
    pass


class Plane():
    """
(Advanced users only)
A data structure representing a projected plane from a camera.

Members:
    id = numeric id of the plane
    flags = see PhaseSpace hardware documentation
    time = system time when the data was captured
    camera = id of the camera this plane came from
    detector = detector id of the camera this plane came from
    plane = 4 floating point numbers representing the plane
    distance = intersection error.  <=0 means invalid.
    """
    __slots__ = ['id', 'flags', 'time', 'camera', 'detector', 'plane', 'cond']
    def __init__(self, id, flags=0, time=-1, camera=0, detector=0, plane=[0.0, 0.0, 0.0, 0.0], distance=-1.0):
        self.id = id
        self.flags = flags
        self.time = time
        self.camera = camera
        self.detector = detector
        self.plane = plane
        self.distance = distance
        pass
    def __str__(self):
        return "Plane(id={}, flags={}, time={}, camera={}, detector={}, plane={}, distance={})".format(self.id, self.flags, self.time, self.camera, self.detector, self.plane, self.distance)
    pass


class Marker():
    """
A data structure representing a marker.

Members:
    id = Numeric ID of the marker
    flags = See PhaseSpace hardware documentation
    time = System time when the data was captured
    x = X-axis position of the marker
    y = Y-axis position of the marker
    z = Z-axis position of the marker
    cond = condition number of the data.  <=0 means invalid
    """
    __slots__ = ['id', 'flags', 'time', 'x', 'y', 'z', 'cond']
    def __init__(self, id, flags=0, time=-1, x=0.0, y=0.0, z=0.0, cond=-1.0):
        self.id = id
        self.flags = flags
        self.time = time
        self.x = x
        self.y = y
        self.z = z
        self.cond = cond
        pass
    def __str__(self):
        return "Marker(id={}, flags={}, time={}, x={}, y={}, z={}, cond={})".format(self.id, self.flags, self.time, self.x, self.y, self.z, self.cond)
    pass


class Rigid():
    """
A data structure representing a rigid body.

Members:
    id = Numeric ID of the rigid body
    flags = See PhaseSpace hardware documentation
    time = System time when the data was captured
    pose = 3D position and quaternion rotation of the rigid body (x, y, z, s, a, b ,c)
    cond = Condition number of the data.  <=0 means invalid
    """
    __slots__ = ['id', 'flags', 'time', 'pose', 'cond']
    def __init__(self, id, flags=0, time=-1, pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], cond=-1.0):
        self.id = id
        self.flags = flags
        self.time = time
        self.pose = pose
        self.cond = cond
        pass
    def __str__(self):
        return "Rigid(id={}, flags={}, time={}, pose={}, cond={})".format(self.id, self.flags, self.time, self.pose, self.cond)
    pass


class Input():
    """
(Advanced users only)
A data structure representing auxiliary input data

Members:
    hw_id = ID of the device this data came from
    flags = See PhaseSpace hardware documentation
    time = Device time of captured data
    data = Raw bytes of the captured data
    """
    def __init__(self, hw_id, flags=0, time=-1, data=None):
        self.hw_id = hw_id
        self.flags = flags
        self.time = time
        self.data = data
        pass
    def __str__(self):
        n = 8
        _data = ",".join(["0x%x" % d for d in self.data[:n]])
        _data = str.format("\"{}...({} bytes)\"", _data, len(self.data))
        return "Input(hw_id={}, flags={}, time={}, data={})".format(self.hw_id, self.flags, self.time, _data)
    pass

class MarkerInfo():
    """
A data structure containing information about configured markers.

Members:
    id = Numeric ID of the marker
    tracker_id = ID of the tracker that the marker belongs to
    name = User configured name of the marker
    options = String of space separated list of options.  Each option is an '=' separated key-value pair.

Supported options:
    pos        Comma separated list of 3 floats representing the 3D local
               position of the marker in the assigned rigid body tracker

Example:
    t_id = 2
    m_id = 1
    context.createTracker(t_id, "rigid", "mytracker")
    context.assignMarkers(MarkerInfo(m_id, t_id, "mymarker", "pos=100,0,-200"))
    """
    def __init__(self, id, tracker_id, name=None, options=""):
        self.id = id
        self.tracker_id = tracker_id
        self.name = name
        self.options = options
        pass
    def __str__(self):
        return "MarkerInfo(id={}, tracker_id={}, name=\"{}\", options=\"{}\")".format(self.id, self.tracker_id, self.name, self.options)
    pass


class TrackerInfo():
    """
A data structure containing information about configured trackers

Members:
    id = Numeric id of tracker
    type = Type of tracker. Supported types include "point" and "rigid"
    name = User configured name
    options = String of space separated list of options.  Each option is an '=' separated key-value pair.
    marker_ids = List of ids of markers assigned to this tracker

Example:
    context.createTrackers([TrackerInfo(1, "rigid", "mytracker01", "", [0, 1, 2, 3]),
                            TrackerInfo(2, "rigid", "mytracker02", "", [4, 5, 6, 7])])
    """
    def __init__(self, id, type, name=None, options="", marker_ids=[]):
        self.id = id
        self.type = type
        self.name = name
        self.options = options
        self.marker_ids = marker_ids
        pass
    def __str__(self):
        return "TrackerInfo(id={} type={} name=\"{}\" options=\"{}\" marker_ids={} )".format(self.id, self.type, self.name, self.options, self.marker_ids)
    pass


class FilterInfo():
    """
A data structure containing information on configured filters

Members:
    period = Period of the filter
    name = User-defined name of the filter
    options = String of space separated list of options.  Each option is an '=' separated key-value pair.

Supported options:
    type        Name of the filter type.  Supported types: lerp, spline

Example:
    context.filters(FilterInfo(120, "myfilter", "type=lerp"))

    """
    def __init__(self, period, name, options=""):
        self.period = period
        self.name = name
        self.options = options
        pass
    def __str__(self):
        return "FilterInfo(period={}, name=\"{}\", options=\"{}\")".format(self.period, self.name, self.options)
    pass

class DeviceInfo():
    """

A data structure containing information on devices attached to the server

Members:
    hw_id = ID of the device
    time = Time
    name = Reported name of device
    options = Device options
    status = Device status
    """
    def __init__(self, hw_id, id, time=-1, type=None, name=None, options="", status=""):
        self.hw_id = hw_id
        self.id = id
        self.time = time
        self.type = type
        self.name = name
        self.options = options
        self.status = status
        pass
    def __str__(self):
        return "DeviceInfo(hw_id={}, id={}, time={}, type={}, name=\"{}\", options=\"{}\", status=\"{}\")".format(self.hw_id, self.id, self.time, self.type, self.name, self.options, self.status)
    pass

class Context:
    """
Main class for communicating with a PhaseSpace Impulse server through the libowl2 API.
Instantiate a Context to connect to a single server.

Members:
    debug = Set to True to enable debugging output (warning: very verbose)

Example:
    see package documentation (Ex:  help(owl))
    """
    class STATS:
        def __init__(self):
            self.udp_packet_count = 0
            self.udp_bcast_packet_count = 0
            self.tcp_packet_count = 0
            pass
        pass

    def __init__(self):
        """ """
        # print debugging output
        self.debug = False

        # for internal testing
        self.flag = 0
        self.ReceiveBufferSize = 4*1024*1024
        self.stats = Context.STATS()

        # main tcp socket
        self.socket = None

        # udp receive sockets
        self.udp = None
        self.broadcast = None

        # open state
        self.__port_offset = 0;
        self.__connect_state = 0

        # receive buffers
        self.__protocol = Protocol()
        self.__protocol_udp = ProtocolUdp()

        # session state
        self.__properties = {}
        self.__types = {}
        self.__names = {}
        self.__trackers = {}
        self.__markers = {}
        self.__filters = {}
        self.__devices = {}
        self.__options = {}
        self.__events = collections.deque()
        self.__newFrame = None

        self.__clear()

        self.__socket_connected = False
        return

    def __log(self, *args):
        if self.debug: print("[%12d] %s" % (int(time.time() * 1E9)," ".join((str(a) for a in args))))
        return

    def __clear(self):
        self.__log("clear()");
        self.__properties = {}
        self.__properties["opened"] = 0
        self.__properties["name"] = ""
        self.__properties["initialized"] = 0
        self.__properties["streaming"] = 0
        self.__properties["mode"] = ""
        self.__properties["local"] = 0
        self.__properties["systemtimebase"]= [0, 0]
        self.__properties["timebase"]= [0, 0]
        self.__properties["maxfrequency"] = 960.0
        self.__properties["systemfrequency"] = 0.0
        self.__properties["frequency"] = 0.0
        self.__properties["scale"] = 1.0
        self.__properties["slave"] = 0
        self.__properties["systempose"] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.__properties["pose"] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.__properties["options"] = ""
        self.__properties["cameras"] = {}
        self.__properties["markers"] = 0
        self.__properties["markerinfo"] = {}
        self.__properties["trackers"] = {}
        self.__properties["trackerinfo"] = {}
        self.__properties["filters"] = {}
        self.__properties["filterinfo"] = {}
        self.__properties["deviceinfo"] = {}
        self.__properties["profiles"] = ""
        self.__properties["defaultprofile"] = ""
        self.__properties["profiles.json"] = ""

        self.newFrame = None

        self.__trackers = {}
        self.__markers = {}
        self.__filters = {}
        self.__devices = {}
        self.__options = {}

        self.__connect_state = 0
        return

    def open(self, name, open_options=""):
        """
Connect to a PhaseSpace Impulse server via TCP/IP.

Arguments:
    name = Address or hostname of server device
    open_options = String of space separated list of options.  Each option is an '=' separated key-value pair.

Supported open_options:
    timeout        Number of microseconds to wait before timing out.
                   Default is 5000000.  If timeout is zero,  then open will
                   attempt to connect in asynchronous mode and will return zero
                   immediately.  Connection state must be polled continuously
                   until success.

Returns:
    0 if attempting asynchronous connection.  User must poll the connection by
      calling open again repeatedly until nonzero is returned.
    1 on success
    Raises an exception on error

Example:
    context = Context()
    context.open("localhost", "timeout=5000000")
        """
        try:
            self.__log("open(%s, %s)" % (name, open_options))
            if self.__properties["opened"] == 1:
                self.__log("    already open.");
                return 1

            # parse options
            opts = utils.tomap(open_options)
            timeout_usec = 10000000
            if "timeout" in opts: timeout_usec = int(opts["timeout"])
            timeout = timeout_usec * 1E-6

            # connect to server
            if self.__connect_state == 0:
                self.__clear()

                # parse host and port offset
                DEFAULT_PORT = 8000
                self.__port_offset = 0
                name = name.split(":")
                self.__port_offset = 0
                if len(name) > 1:
                    self.__port_offset = int(name[1])
                    pass
                port = DEFAULT_PORT + self.__port_offset
                self.__properties["name"] = name[0]
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)

                # configure socket
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.ReceiveBufferSize)
                self.socket.setblocking(0)

                # start asynchronous connect
                self.__log("    connecting to %s:%d" % (name[0], port))
                ret = self.socket.connect_ex((name[0], port))
                if ret == errno.EAGAIN or ret == errno.EWOULDBLOCK or ret == errno.EALREADY or ret == errno.EINPROGRESS:
                    # connect needs more time
                    self.__connect_state = 1
                    if timeout_usec == 0: return 0
                    pass
                elif ret == 0:
                    self.__connect_state = 2
                    pass
                else: raise OpenError("unknown socket error: %d" % ret)
                pass

            # wait for socket to be ready
            if self.__connect_state == 1:
                r,w,e = select.select([], [self.socket], [self.socket], timeout)
                if len(e) > 0: raise OpenError("socket select error")
                if len(w) == 0:
                    if timeout == 0: return 0
                    raise OpenError("connection timed out")
                err = utils.getsocketerror(self.socket) #TODO fix
                if err != 0 and err not in (errno.EWOULDBLOCK, errno.EALREADY, errno.EINPROGRESS):
                    raise socket.error("socket error: %d\n" % err)
                self.__connect_state += 1
                pass

            if self.__connect_state == 2:
                self.__log("    waiting for server state")
                self.__socket_connected = True
                self.__connect_state += 1
                pass

            # recieve state from server
            if self.__connect_state == 3:
                def wait_func():
                    self.__recv(0)
                    return self.__properties["opened"] == 1
                if self.__wait(timeout, wait_func):
                    self.__connect_state += 1
                    pass
                pass

            if self.__properties["opened"] == 1:
                self.__log("    success")
                # TODO test
                # upload version info
                if self.__connect_state == 4:
                    self.__send(Type.BYTE, "internal", "protocol=%s libowl=%s" % (OWL_PROTOCOL_VERSION, LIBOWL_REV))
                    self.__connect_state += 1
                    pass
                # tcp connection established, open udp receive socket
                try:
                    self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
                    self.udp.setblocking(0)
                    self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.ReceiveBufferSize)
                    self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    self.__log("binding to udp port %s" % self.socket.getsockname()[1])
                    self.udp.bind(self.socket.getsockname())
                    pass
                except Exception as e:
                    self.__log("udp bind error");
                    udp = None
                    pass
                return 1
            if timeout == 0: return 0
            raise OpenError("connection timed out (state: %d)" % self.__connect_state)
        except Exception as e:
            self.close()
            raise
        return

    def close(self):
        """
Disconnect from the current server.
        """
        try:
            self.__log("close()")
            # shutdown tcp socket
            if self.__socket_connected: self.socket.shutdown(socket.SHUT_RDWR);
            self.socket.close()
            # shutdown udp socket
            if self.broadcast != None:
                self.broadcast.close()
                self.broadcast = None
                pass
            if self.udp != None:
                self.udp.close()
                self.udp = None
                pass
            pass
        except Exception as e:
            sys.stderr.write(str(e))
            pass
        finally:
            self.__clear()
            self.__socket_connected = False
            pass
        return

    def isOpen(self):
        """
Query whether this Context is connected to a server.
        """
        if not self.socket: return False
        return self.__socket_connected and self.property("opened") == 1;

    def initialize(self, init_options=""):
        """
Set up a streaming session with the server.  Must be called after a successful call to open().  Note, a successful return does not guarantee the system is configured correctly.  The server may send error Events back to the client asynchronously that must be checked with the nextEvent() function.

Arguments:
    init_options = String of space separated list of options.  Each option is an '=' separated key-value pair.

Supported init_options:
    timeout           Number of microseconds to wait for the operation.
                      Default is 5000000.
    frequency         frequency to send capture data to this client at.
    streaming         See streaming().
    slave             0 or 1.  Enables or disables slave mode.
    event.raw         0 or 1.  Enables or disables streaming of raw unfiltered
                      data.  Default is 1.
    event.markers     0 or 1.  Enables or disables streaming of marker data.
                      Default is 1.
    event.rigids      0 or 1.  Enables or disables streaming of rigid body data.
                      Default is 1.
    event.peaks       0 or 1.  Enables or disables streaming of peak data.
                      Default is 0.
    event.planes      Advanced users only
    event.inputs      Advanced users only


Returns:
     >0 on success
     Raises an exception on error

Example:
     context.open("localhost")
     context.initialize("event.markers=1 event.rigids=0")
        """
        try:
            self.__log("initialize(%s)" % init_options)
            if not self.isOpen(): raise InitError("no connection")
            if self.property("initialized") == 1:
                self.__log("    already initialized")
                return 1

            # parse options
            opts = utils.tomap(init_options)
            timeout_usec = 10000000
            if "timeout" in opts: timeout_usec = int(opts["timeout"])
            timeout = timeout_usec * 1E-6

            if not "initializing" in self.__properties:
                name = self.__properties["name"]
                profiles = self.__properties["profiles"]
                defaultprofile = self.__properties["defaultprofile"]
                profiles_json = self.__properties["profiles.json"]
                self.__clear()
                self.__properties["opened"] = 1
                self.__properties["name"] = name
                self.__properties["profiles"] = profiles
                self.__properties["defaultprofile"] = defaultprofile
                self.__properties["profiles.json"] = profiles_json
                self.__properties["initializing"] = 1

                init_options = "event.raw=1 event.markers=1 event.rigids=1"+(" "+init_options if init_options else "")
                self.__log("    uploading options: %s" % init_options)

                # tell server to initialize
                self.__send(Type.BYTE, "initialize", init_options)

                if timeout_usec == 0: return 0
                self.__log("    waiting for response")
                pass

            def wait_func():
                self.__recv(timeout)
                return self.property("initialized") == 1
            # wait for server state
            if not self.__wait(timeout, wait_func) and timeout_usec == 0:
               return 0

            if self.property("initialized") == 0:
                self.__log("    failed")
                if timeout_usec > 0:
                    raise InitError("timed out")
                raise InitError("init failed")

            if self.flag == 1: raise InitError("test")
        except Exception as e:
            if "initializing" in self.__properties: del self.__properties["initializing"]
            self.__properties["initialized"] = 0
            raise
        self.__log("    success")
        return 1

    def done(self, done_options=""):
        """
Inform the server that the streaming session is done.   initialize() can be called again after a session has been terminated.

Arguments:
    done_options = String of space separated list of options.  Each options is an '=' separated key-value pair.

Supported done_options:
    timeout        Number of microseconds to wait before timing out.
                   Default is 1000000.  If timeout is zero,  then done() will
                   send a done packet to the server asynchronously and return
                   before the server responds.  The user can call done() again
                   to poll for the server response.

Returns:
    1 on success, 0 if the timeout option is >0 and server has not responded within timeout.
    -1 is there is no connection to server.
    Raises an exception on error any other error.
        """
        self.__log("done %s" % (self.property("initialized")))
        if not self.isOpen(): return -1

        opts = utils.tomap(done_options)
        timeout = 10000000
        if "timeout" in opts: timeout = int(opts["timeout"])
        timeout *= 1E-6
        if property("initialized") == 0:
            if "flushing" in self.__properties:
                del self.__properties["flushing"]
            return 1
        if not "flushing" in self.__properties:
            self.__properties["flushing"] = 1
            # inform server that this session is done
            self.__send(Type.BYTE, "done", done_options)
            pass
        def wait_func():
            if self.flag == 1: return False
            self.__recv(0)
            return self.property("initialized") == 0
        self.__wait(timeout, wait_func)
        if self.property("initialized") == 1:
            if timeout > 0: return -1
            return 0
        if "flushing" in self.__properties:
            del self.__properties["flushing"]
        return 1

    def streaming(self, enable=None):
        """
Get or set the streaming property of this context.
The streaming property instructs the server to start sending data to the client.
This is an asynchronous operation when called with arguments.

Arguments:
    enable    None to obtain the current value
              0 to disable streaming
              1 to enable tcp streaming
              2 to enable udp streaming
              3 to enable udp broadcast

Returns:
    Returns the current streaming value if called with no parameters
    No return value if called with a parameter.
    Raises an exception on error.

Example:
    if not owl.streaming():
       context.streaming(1)
        """
        return self.__getset_prop(Type.INT, "streaming", enable)

    def frequency(self, freq=None):
        """
Get or set the frequency property of this context.
The frequency property controls the frequency of the data sent to this client.
This is an asynchronous operation when called with arguments.

Arguments:
    freq = A floating point number to set the frequency, or None to query the current frequency.

Returns:
    The current frequency if called with no parameters.
    No return value if called with a parameter.
    Raises an exception on error.

Example:
    context.frequency(960)
    while context.frequency() != 960:
        time.sleep(0)
        """
        if freq != None: freq = float(freq)
        return self.__getset_prop(Type.FLOAT, "frequency", freq)

    def timeBase(self, num=None, denom=None):
        """
Get or set the timebase property of this context.
The timebase property controls the time scale of the returned data.
This is an asynchronous operation when called with arguments.

Arguments:
    num = Integer numerator of the timebase, or None
    denom = Integer denominator of the timebase, or None

Returns:
    The current timebase as a list of integers if called with no parameters
    No return value if called with parameters.
    Raises an exception on error.

Example:
    context.timeBase(1, 1000000) # set time domain to microseconds
        """
        if num != None and denom != None: value = [int(num), int(denom)]
        else: value = None
        ret = self.__getset_prop(Type.INT, "timebase", value)
        if ret == None: return
        return ret

    def scale(self, scale=None):
        """
Get or set the scale property of this context.
The scale property controls the scale factor applied to incoming data.
This is an asynchronous operation when called with arguments.

Arguments:
    scale = Floating point scale factor, or None.

Returns:
    The current scale if called with no parameters
    No return value if called with parameters.
    Raises an exception on error.

Example:
    context.scale(10.0) # scale incoming data by 10
        """
        if scale != None: scale = float(scale)
        return self.__getset_prop(Type.FLOAT, "scale", scale)

    def pose(self, pose=None):
        """
Get or set the pose property of this context.
The pose property controls the positional and rotational offset applied to incoming marker and rigid body data.
This is an asynchronous operation when called arguments.

Arguments:
    pose = List of floats representing a vector and quaternion with the format [x, y, z, w, a, b, c], or None

Returns:
    The current pose if called with no parameters
    No return value if called with parameters.
    Raises an exception on error.

Example:
    # translate incoming markers and rigids 1000 units along the x-axis
    context.pose([1000, 0, 0, 1, 0, 0, 0])
        """
        return self.__getset_prop(Type.FLOAT, "pose", pose)

    def option(self, optname, value=None):
        """
Sets a system option.  Equivalent to options("%s=%s" % (optname, value))
This is an asynchronous operation when called with value != None.

Arguments:
    optname = The name of an option.  See options()
    value = The value to set the option to.  See options()

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.option("event.markers", 1)
        """
        if value == None: return self.__options[optname]
        else: value = "%s=%s" % (optname, value)
        return self.__getset_prop(Type.BYTE, "options", value)

    def options(self, options=None):
        """
Set the current system options.
This is an asynchronous operation when called with arguments.

Arguments:
    options = A space separated string of key-value pairs.  Each key-value pair is separated by an '='.

Supported options:
    event.raw         0 or 1.  Enables or disables streaming of raw unfiltered
                      data.  Default is 1.
    event.markers     0 or 1.  Enables or disables streaming of marker data.
                      Default is 1.
    event.rigids      0 or 1.  Enables or disables streaming of rigid body data.
                      Default is 1.
    event.peaks       0 or 1.  Enables or disables streaming of peak data.
                      Default is 0.
    event.????        0 or 1.  ???? equals a user-defined name for a filter.
                      Default is 0.

Returns:
    No return value
    Raises an exception on error.

Example:
    # disable marker streaming, and enable rigid body streaming
    context.options("event.markers=0 event.rigids=1")
        """
        return self.__getset_prop(Type.BYTE, "options", options)

    def createTracker(self, tracker_id, tracker_type="point", tracker_name=None, tracker_options=""):
        """
        Equivalent to createTrackers(), but allows a single tracker to be specified with direct arguments.
        """
        return self.createTrackers([TrackerInfo(tracker_id, tracker_type, tracker_name, tracker_options)])

    def createTrackers(self, trackerinfo=[]):
        """
Creates trackers on the server from a list of TrackerInfo instances.
This is an asynchronous operation.

A tracker is a construct for organizing data.
There are two main types of trackers:
    "point" = Normal marker data
    "rigid" = Rigid body trackers calculate an 6-DOF orientation from markers that
            are assigned to it. At least four markers are required.

On initialization,  the server usually has a default point tracker.
The client can create additional ones.

Arguments:
    trackerinfo = A list of TrackerInfo instances

Returns:
    No return value.
    Raises an exception on error.

Example:
    See TrackerInfo
        """
        # tracker_info = [ TrackerInfo(), ... ]
        if type(trackerinfo) == str: s = trackerinfo.strip()
        else:
            try: trackerinfo = list(trackerinfo)
            except: trackerinfo = [trackerinfo]
            s = []
            for t in trackerinfo:
                name = t.name if t.name and len(t.name) else t.id
                marker_ids = "mid=" + ",".join([str(m) for m in t.marker_ids]) if len(t.marker_ids) else ""
                s.append(str.format("id={} type={} name={}", t.id, t.type, name))
                if len(marker_ids): s.append(marker_ids)
                if len(t.options): s.append(t.options.strip())
                pass
            s = " ".join(s).strip()
            pass
        self.__log("createtrackers: \"%s\"" % s)
        return 0 < self.__send(Type.BYTE, "createtracker", s)

    def trackerName(self, tracker_id, tracker_name):
        """
Assign a user-defined name to a previously created tracker.
This is an asynchronous operation.

Arguments:
    tracker_id = Numeric ID of the tracker
    tracker_name = Name to be assigned

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.createTracker(1, "point", "myoldtrackername")
    context.trackerName(1, "mynewtrackername")
        """
        s = str.format("id={} name={}", tracker_id, tracker_name)
        self.__log("trackername:", s)
        return 0 < self.__send(Type.BYTE, "trackername", s)

    def trackerOptions(self, tracker_id, tracker_options):
        """
Internal use only.
        """
        s = str.format("id={} {}", tracker_id, tracker_options)
        self.__log("trackeroptions:", s)
        return 0 < self.__send(Type.BYTE, "trackeroptions", s)

    def trackerInfo(self, tracker_id):
        """
Gets a the TrackerInfo of a specified tracker.

Arguments:
    tracker_id = Numeric tracker id.

Returns:
    TrackerInfo instance, or None if the tracker is not defined.

Example:
    context.trackerInfo(1)
        """
        return self.__trackers[tracker_id] if tracker_id in self.__trackers else None

    def destroyTracker(self, tracker_id):
        """
Destroys a previously defined tracker on the server.
This is an asynchronous operation.

Arguments:
    tracker_id = Numeric tracker id

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.createTracker(1, "point")
    context.destroyTracker(1)
        """
        return self.destroyTrackers([tracker_id])

    def destroyTrackers(self, tracker_ids=[]):
        """
Destroys a list of previously defined trackers on the server.
This is an asynchronous operation.

Arguments:
    tracker_ids = Numeric list of tracker ids

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.createTracker(1, "point")
    context.createTracker(2, "point")
    context.destroyTrackers([1, 2])
        """
        if type(tracker_ids) == str: s = tracker_ids
        elif len(tracker_ids) > 0:
            s = "id=" + ",".join([str(t) for t in tracker_ids])
            pass
        self.__log("destroytrackers:", s)
        return 0 < self.__send(Type.BYTE, "destroytracker", s)

    def assignMarker(self, tracker_id, marker_id, marker_name=None, marker_options=""):
        """
Assign a marker to a tracker.  This is primarily useful for adding markers to rigid bodies.
This is an asynchronous operation.

Arguments:
    tracker_id = Numeric tracker id
    marker_id = Numeric marker id
    marker_name =  Optional user-defined marker name.
    marker_options = See MarkerInfo for list of supported options.

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.createTracker(0, "rigid")
    context.assignMarker(0, 1, "mymarker", "pos=100,100,100")
        """
        return self.assignMarkers([MarkerInfo(marker_id, tracker_id, marker_name, marker_options)])

    def assignMarkers(self, markerinfo=[]):
        """
Assigns markers to a tracker.  This is primarily useful for adding markers to rigid bodies.
This is an asynchronous operation.

Arguments:
    markerinfo = List of MarkerInfo instances

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.createTracker(0, "rigid")
    context.assignMarkers([MarkerInfo(0, 1, "mymarker1", "pos=100,0,0"),
                           MarkerInfo(0, 2, "mymarker2", "pos=0,100,0")])
        """
        if type(markerinfo) == str: s = markerinfo.strip()
        else:
            try: markerinfo = list(markerinfo)
            except: markerinfo = [markerinfo]
            s = []
            for m in markerinfo:
                name = m.name if m.name and len(m.name) else m.id
                s.append(str.format("tid={} mid={} name={}", m.tracker_id, m.id, name))
                if len(m.options): s.append(m.options)
                pass
            s = " ".join(s).strip()
            pass
        self.__log("assignmarkers:", s)
        return 0 < self.__send(Type.BYTE, "assignmarker", s)


    def markerName(self, marker_id, marker_name):
        """
Assigns a user-defined name to the specified marker.
This is an asynchronous operation.

Arguments:
    marker_id = The numeric id of the marker
    marker_name = The name to be assigned

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.createTracker(0, "points")
    context.assignMarker(0, 1, "myoldmarkername")
    context.markerName(1, "mynewmarker")
        """
        s = str.format("mid={} name={}", marker_id, marker_name).strip()
        self.__log("markername:", s)
        return 0 < self.__send(Type.BYTE, "markername", s)

    def markerOptions(self, marker_id, marker_options):
        """
Sets options for a specified marker.  See help(MarkerInfo) for supported options.
This is an asynchronous operation.

Arguments:
    marker_id = the numeric id of the marker
    marker_options = see help(MarkerInfo)

Returns:
    No return value.
    Raises an exception on error.

Example:
    context.createTracker(0, "rigid")
    context.assignMarker(0, 1, "myoldmarkername")
    context.markerOptions(1, "pos=100,0,100")
        """
        s = str.format("mid={} {}", marker_id, marker_options).strip()
        self.__log("markeroptions:", s)
        return 0 < self.__send(Type.BYTE, "markeroptions", s)

    def markerInfo(self, marker_id):
        """
Gets the MarkerInfo for a specified marker id.

Arguments:
    marker_id = Numeric marker id

Returns:
    A MarkerInfo instance or None if the marker_id is invalid

Example:
    context.markerInfo(1)
        """
        return self.__markers[marker_id] if marker_id in self.__markers else None


    def filter(self, period, name, filter_options=""):
        """
Define a filter on the server.
This is an asynchronous operation.

Arguments:
    period = See FilterInfo
    name = See FilterInfo
    filter_options = See FilterInfo

Returns:
    No return value.
    Raises an exception on error.

Example:
    # define filters
    context.filter(60, "mylerpfilter", "type=lerp")
    # enable filters
    context.options("event.mylerpfilter1=1") #enable filters
        """
        return self.filters([FilterInfo(period, name, filter_options)])

    def filters(self, filterinfo=[]):
        """
Define filters on the server.
This is an asynchronous operation.

Arguments:
    filterinfo = List of FilterInfo instances

Returns:
    No return value.
    Raises an exception on error.

Example:
    # define filters
    context.filters([FilterInfo(120, "myfilter1", "type=lerp"),
                     FilterInfo(120, "myfilter2", "type=spline")])
    # enable filters
    context.options("event.myfilter1=1 event.myfilter2=1")
        """
        if type(filterinfo) == str: s = filterinfo.strip()
        else:
            try: filterinfo = list(filterinfo)
            except: filterinfo = [filterinfo]
            s = []
            for f in filterinfo:
                s.append(str.format("filter={} period={}", f.name, f.period))
                if f.options: s.append(f.options.strip())
                pass
            s = " ".join(s).strip()
            pass
        self.__log("filter:", s)
        return 0 < self.__send(Type.BYTE, "filter", s)

    def filterInfo(self, name):
        """
Get the FilterInfo of a filter.

Arguments:
    name = User-defined name of the filter

Returns:
    A FilterInfo instance or None if the named filter is not defined.

Example:
    context.filter(60, "myfilter", "type=lerp")
    print(context.filterInfo("myfilter"))
        """
        return self.__filters[name] if name in self.__filters else None

    def deviceInfo(self, hw_id):
        """
Get the DeviceInfo for a specified device.

Arguments:
    hw_id = The hardware id of the device

Returns:
    A DeviceInfo instance or None if the device is not found.

Example:
    print(context.deviceInfo("0xffffffff"))
        """
        return self.__devices[hw_id] if hw_id in self.__devices else None

    def peekEvent(self, timeout_usec=0):
        """
Get the next Event from the server without removing it from the event queue.  Repeated calls to peekEvent() will return the same event.  Use nextEvent() to flush the event queue normally.

Returns:
    The next Event instance in the event queue.
    None if no event is available.
    Raises an exception on error.
        """
        #TODO test again
        timeout = timeout_usec * 1E-6
        self.__recv(timeout)
        if not self.__events: return None
        return self.__events[0];

    def nextEvent(self, timeout_usec=0):
        """
Get the next Event from the server.  All data and system state is received from the server as Events.

Arguments:
    timeout_usec = Maximum number of microseconds to wait for a new Event.  Default is zero.

Returns:
    The next Event instance in the event queue.
    None if no new event is available.
    Raises an exception on error.

Example:
    while True:
        event = context.nextEvent()
        if event: print(event.type_id, event.name, event.time)

        """
        timeout = timeout_usec * 1E-6
        self.__recv(timeout)
        if not self.__events: return None
        return self.__events.popleft()

    def property(self, name):
        """ """
        # TODO test options
        return self.__properties[name]

    #internal functions
    def __recv_helper(self, sock, p):
        ret = 0
        #iterate multiple times because some frames are comprised of multiple events
        for i in range(6):
            events = p.recv(sock)
            if events == None: break
            for evt in events:
                ret += self.__process_event(evt)
                pass
            pass
        return ret

    def __recv(self, timeout):
        try:
            rsocks = []
            if self.__socket_connected and self.socket: rsocks.append(self.socket)
            if self.udp: rsocks.append(self.udp)
            if self.broadcast: rsocks.append(self.broadcast)
            rsocks,wsocks,esocks = select.select(rsocks, [], [], timeout)
            if len(rsocks) == 0: return
            # tcp
            if self.socket in rsocks:
                self.stats.tcp_packet_count += self.__recv_helper(self.socket, self.__protocol)
                pass
            # udp
            if self.udp in rsocks:
                self.stats.udp_packet_count += self.__recv_helper(self.udp, self.__protocol_udp)
                pass
            # udp broadcast
            if self.broadcast in rsocks:
                self.stats.udp_bcast_packet_count += self.__recv_helper(self.broadcast, self.__protocol_udp)
                pass
            pass
        except Exception:
            self.close()
            raise
        return 1

    def __process_event(self, evt):

        # handle internal events
        if evt.id == 0:
            if evt.type_id == Type.ERROR and "initializing" in self.__properties:
                raise InitError(evt.data)
            if evt.type_id != Type.BYTE:
                sys.stderr.write("warning: unknown event: type=%d id=%d" % (evt.type_id, evt.id))
                return 0
            self.__handle_internal(evt)
            return 0

        # end-of-frame event
        if evt.type_id == Type.FRAME and self.__newFrame != None and  evt.id == self.__newFrame.id and evt.time == self.__newFrame.time:
            evt = self.__newFrame
            self.__newFrame = None
            pass

        # accumulate data into frame if part of one, otherwise pass through
        if evt.id & 0xff00:
            frame_id = evt.id >> 8
            evt.id &= 0xff
            if not evt.type_id in self.__types or not evt.id in self.__names:
                # unknown event, drop it
                self.__log("warning: unknown event: type=%d id=%d" % (evt.type_id, evt.id))
                pass
            else:
                # create a new frame object if necessary
                if self.__newFrame == None or frame_id != self.__newFrame.id or evt.time != self.__newFrame.time:
                    self.__newFrame = Event(Type.FRAME, frame_id, 0, evt.time)
                    pass
                # add data to frame object's dictionary
                name = self.__names[evt.id]["name"]
                self.__newFrame[name] = evt.data
                pass
            return 0

        # set type_names and names
        if not self.__populate_event(evt):
            self.__log("error: name lookup failed")
            return 0

        # process special cases
        if evt.type_id == Type.BYTE: self.__handle_byte(evt)
        elif evt.type_id == Type.FLOAT: self.__handle_float(evt)
        elif evt.type_id == Type.CAMERA: self.__properties["cameras"] = evt.data
        elif evt.type_id == Type.INT: self.__handle_int(evt)
        elif evt.type_id == Type.ERROR:
            if "initializing" in self.__properties:
                raise InitError(evt.data)
            pass
        # send to user
        self.__events.append(evt)
        return 1

    def __populate_event(self, evt):
        if not evt.type_id in self.__types or not evt.id in self.__names:
            return False
        evt.type_name = self.__types[evt.type_id]["name"]
        evt.name = self.__names[evt.id]["name"]
        return True

    # TODO fix this
    @staticmethod
    def __findID(table, name):
        for k,v in table.items():
            if v["name"] == name: return k
            pass
        return None

    def __update_property(self, key, value):
        self.__log("property update: %s=%s" % (key, value))
        table = self.__properties
        if key in table:
            t = type(table[key])
            if t == type(value): table[key] = value
            elif t == int or t == float: table[key] = t(value)
            elif t == list or t == tuple:
                t2 = type(table[key][0])
                table[key] = t([t2(v) for v in value.split(",")])
                pass
            else: table[key] = value
            pass
        else:
            self.__log("warning: unsupported property received from server: " + key)
            table[key] = value
            pass
        return

    def __send(self, _type, name, data):
        if type(name) is int: id = name
        else: id = self.__findID(self.__names, name)
        if id == None: raise OWLError("invalid name");
        if _type == Type.BYTE:
            if type(data) != str: raise OWLError("str type data expected")
            pass
        elif _type == Type.FLOAT or _type == Type.INT:
            if type(data) != list: data = [data,]
            if _type == Type.FLOAT:
                form = "<%df" % len(data)
                t = float
                pass
            elif _type == Type.INT:
                form = "<%di" % len(data)
                t = int
                pass
            data = struct.pack(form, *(t(d) for d in data))
            pass
        else: raise OWLError("unsupported send type")
        try:
            self.__protocol.send(self.socket, id, _type, data)
        except:
            self.close()
            raise
        return True

    def __handle_internal(self, e):
        if e.data == None:
            raise OWLError("null internal data");
        data = e.data.decode("ascii")
        self.__log(data)
        if data.startswith("table=types"):
            self.__parseType(self.__types, data)
            pass
        elif data.startswith("table=names"):
            self.__parseType(self.__names, data)
            pass
        elif data.startswith("table=trackers"):
            self.__parseTrackerInfo(self.__trackers, data)
            self.__properties["trackers"] = map(lambda x: x.id, self.__trackers.values())
            self.__properties["trackerinfo"] = self.__trackers.values()
            id = self.__findID(self.__names, "info")
            if id != None: self.__events.append(Event(Type.TRACKERINFO, id, 0, e.time, self.__types[Type.TRACKERINFO]["name"], self.__names[id]["name"], data=self.__trackers.values()))
            pass
        elif data.startswith("table=markers"):
            self.__parseMarkerInfo(self.__markers, data)
            for t in self.__trackers.values(): t.marker_ids = []
            for m in self.__markers.values():
                try: t = self.__trackers[m.tracker_id]
                except KeyError: continue
                if t: t.marker_ids.append(m.id)
                pass
            self.__properties["markers"] = len(self.__markers)
            self.__properties["markerinfo"] = self.__markers
            self.__properties["trackerinfo"] = self.__trackers.values()
            id = self.__findID(self.__names, "info")
            if id != None:
                self.__events.append(Event(Type.MARKERINFO, id, 0, e.time, self.__types[Type.MARKERINFO]["name"], self.__names[id]["name"], data=self.__markers))
                self.__events.append(Event(Type.TRACKERINFO, id, 0, e.time, self.__types[Type.TRACKERINFO]["name"], self.__names[id]["name"], data=self.__trackers.values()))
                pass
            pass
        elif data.startswith("table=devices") or data.startswith("status=devices"):
            if data.startswith("table=devices"): self.__parseDeviceInfo(self.__devices, data)
            if data.startswith("status=devices"): self.__parseDeviceStatus(self.__devices, data)
            self.__properties["deviceinfo"] = self.__devices.values()
            id = self.__findID(self.__names, "info")
            if id != None: self.__events.append(Event(Type.DEVICEINFO, id, 0, e.time, self.__types[Type.DEVICEINFO]["name"], self.__names[id]["name"], data = self.__devices.values()))
            pass
        elif data.startswith("table=enable"):
            opts = dict(filter(lambda x: x[0].startswith("event."), map(lambda x: x.split('='), data.split())))
            self.__options.update(opts)
            pass
        elif data.startswith("filter="):
            self.__parseFilterInfo(self.__filters, data)
            fi = filter(lambda x: x.period, self.__filters.values())
            self.__properties["filterinfo"] = fi
            self.__properties["filters"] = map(lambda x: x.name, fi)
            id = self.__findID(self.__names, "info")
            if id != None:
                self.__events.append(Event(Type.FILTERINFO, id, 0, e.time, self.__types[Type.FILTERINFO]["name"], self.__names[id]["name"], data=fi))
                pass
            pass
        elif data.startswith("defaultprofile="):
            self.__properties["defaultprofile"] = utils.tomap(data)["defaultprofile"]
            pass
        elif data.startswith("profiles.json"):
            self.__properties["profiles.json"] = urlunquote(data)
            pass
        elif data.startswith("profiles="):
            d = data.split('=')
            if len(d) >= 2: self.__properties["profiles"] = d[1].split(",")
            pass
        else:
            for kv in dict(map(lambda x: x.split('='), data.split())).items():
                self.__update_property(kv[0], kv[1])
                # deprecated
                if "streaming" == kv[0]:
                    self.__toggle_broadcast(self.property("streaming"))
                    pass
                pass
            pass
        return

    def __toggle_broadcast(self, v):
        if v == 3 and self.broadcast == None:
            self.broadcast = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
            self.broadcast.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.broadcast.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.broadcast.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.ReceiveBufferSize)
            self.broadcast.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
            self.broadcast.bind(("<broadcast>", 8500 + self.__port_offset))
            self.broadcast.setblocking(0)
            pass
        elif v != 3 and self.broadcast != None:
            self.broadcast.close()
            self.broadcast = None
            pass
        pass

    def __handle_byte(self, e):
        e.data = e.data.decode("ascii")
        self.__log(e.data)
        if e.name == "options":
            self.__log("options recieved: ", self.options())
            options = self.__options
            options.update(utils.tomap(e.data))
            self.__properties["options"] = " ".join(["%s=%s" % (k, options[k]) for k in options])
            pass
        elif e.name == "initialize" or e.name == "done":
            for kv in utils.tomap(e.data).items():
                self.__update_property(kv[0], kv[1])
                if kv[0] == "streaming":
                    self.__toggle_broadcast(int(kv[1], 10))
                    pass
                if e.name == "initialize" and "initializing" in self.__properties:
                    del self.__properties["initializing"]
                    pass
                pass
            return
        return

    def __handle_float(self, e):
        self.__log(e)
        if e.name == "systempose": self.__properties["systempose"] = e.data[:7]
        elif e.name == "scale" : self.__properties["scale"] = e.data[0]
        elif e.name == "pose": self.__properties["pose"] = e.data[:7]
        elif e.name == "frequency": self.__properties["frequency"] = e.data[0]
        return

    def __handle_int(self, e):
        self.__log(e)
        if e.name == "streaming":
            self.__properties["streaming"] = e.data[0]
            self.__toggle_broadcast(e.data[0])
            pass
        elif e.name == "timebase":
            self.__properties["timebase"] = e.data
            pass
        return

    @staticmethod
    def __parseType(table, options):
        opts = map(lambda x: x.split('='), options.split())
        for n,v in opts:
            try:
                n = int(n, 10)
                if n > 65355: continue
                v = v.split(',')
                name, flags, mode = v[0], 0, 0
                if len(v) > 1 and v[1] != "": flags = int(v[1], 10)
                if len(v) > 2 and v[2] != "": mode = int(v[2], 10)
                table[n] = {"name" : name, "flags" : flags, "mode" : mode}
                pass
            except ValueError:
                pass
            pass
        return

    @staticmethod
    def __parseTrackerInfo(table, options):
        # table=trackers id=n,id,type,name [options]
        opts = options.split("id=")
        n = -1
        for o in opts[1:]:
            try:
                o = o.split()
                v = o[0].split(',')
                if len(v) > 1:
                    n = int(v[0])
                    if n > 65355: continue
                    if len(v) == 4:
                        table[n] = TrackerInfo(int(v[1]), v[2], v[3])
                        pass
                    pass
                if len(o) > 1 and n > -1 and n <= 65536:
                    table[n].options += " ".join(o[1:]).strip()
                    pass
                pass
            except:
                print(("error parsing entry: " + str(o)))
                pass
            pass
        return

    @staticmethod
    def __parseMarkerInfo(table, options):
        # table=markers id=id,tid,name [options]
        opts = options.split("id=")
        id = -1
        for o in opts[1:]:
            try:
                o = o.split()
                v = o[0].split(',')
                if len(v) > 1:
                    id = int(v[0])
                    if id > 65535: continue;
                    if len(v) == 3: #id, tid, name
                        table[id] = MarkerInfo(id, int(v[1]), v[2])
                        pass
                    pass
                if len(o) > 1 and id > -1 and id <= 65536:
                    table[id].options += " ".join(o[1:]).strip()
                    pass
                pass
            except:
                print(("error parsing entry: " + str(o)))
                pass
            pass
        return

    @staticmethod
    def __parseDeviceInfo(table, options):
        opts = options.split("id=")
        hwid = -1
        for o in opts[1:]:
            try:
                o = o.split()
                v = o[0].split(',')
                if len(v) > 1:
                    hwid = utils.str_to_int(v[0])
                    if len(v) == 4: #hwid, id, tid, name
                        table[hwid] = DeviceInfo(hwid, int(v[1]), 0, v[2], v[3])
                        pass
                    pass
                if len(o) > 1 and hwid in table:
                    table[hwid].options += " ".join(o[1:]).strip()
                    pass
                pass
            except:
                print(("error parsing entry: " + str(o)))
                pass
            pass
        return

    @staticmethod
    def __parseDeviceStatus(table, options):
        opts = options.split("id=")
        hwid = -1
        for o in opts[1:]:
            try:
                o = o.split()
                v = o[0].split(',')
                if len(v) > 1:
                    hwid = utils.str_to_int(v[0])
                    if len(v) == 2 and hwid in table: #hwid, time
                        table[hwid].time = int(v[1])
                        table[hwid].status = ""
                        pass
                    pass
                if len(o) > 1 and hwid in table:
                    table[hwid].status += " ".join(o[1:]).strip()
                    pass
                pass
            except:
                print(("error parsing entry: " + str(o)))
                pass
            pass
        return

    @staticmethod
    def __parseFilterInfo(table, options):
        opts=options.split("filter=")
        for o in opts[1:]:
            try:
                v = o.split()
                name = v[0]
                table[name] = FilterInfo(0, name, "")
                fopts = []
                for _v in v[1:]:
                    m = re.findall("period=([0-9]+)", _v)
                    if len(m): table[name].period = int(m[0])
                    else: fopts.append(_v)
                    pass
                table[name].options = " ".join(fopts)
                pass
            except:
                print(("error parsing entry: " + str(o)))
                pass
            pass
        return

    def __getset_prop(self, type_id, name, value):
        """internal helper function"""
        if value == None:
            ret = self.__properties[name]
            self.__log(name, ":", ret)
            return ret
        if type_id == Type.BYTE: s = str(value)
        elif type_id == Type.FLOAT: s = value
        elif type_id == Type.INT: s = value
        else: raise OWLError("unsupported type")
        self.__send(type_id, name, s)
        return

    @staticmethod
    def __wait(timeout, func):
        if timeout == 0: return func()
        t0 = time.time()
        while True:
            if func(): return True
            if time.time() - t0 >= timeout: break
            pass
        return False
    pass


class Scan:
    """
Class for scanning for available servers on the local network.

Example:
    scan = Scan()
    scan.send("myscript")
    servers = scan.listen(5000000)
    print(servers)
    """
    def __init__(self):
        """ """
        self.socket = None
        return

    def send(self, message=b"python"):
        """
Send a UDP broadcast packet to local servers.  Servers will respond with an identification packet (see listen())

Arguments:
    message = User-defined string identifying this client to the server.  This will show up in the server's logs.

Returns:
    The number of bytes sent,  or <0 on error.
    Raises a socket exception on error.
        """
        if not self.socket:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
            self.socket.setblocking(0)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            pass
        message = message + '\0'
        if sys.version_info.major >= 3: message = bytes(message, 'UTF-8')
        return self.socket.sendto(message, ("<broadcast>", 8998))

    def listen(self, timeout=0):
        """
Listen for server UDP response packets after a send() call.

Arguments:
    timeout = Maximum microseconds to wait for a response.  Defaults to 1000000.

Returns:
    A list of strings identifying the available servers.  The format of each
    string is "ip={ip_address} hostname={server_hostname} version={server_version}"
        """
        out = []
        if not self.socket: return out
        timeout = None if not timeout else timeout*1E-6
        t0 = time.time()
        while True:
            try:
                r,w,e = select.select([self.socket], [], [self.socket], timeout)
                if len(e): raise SendError("select fail")
                if len(r) == 0: return out
                ret = self.socket.recvfrom(9*1024)
                out.append(str.format("ip={} {}", ret[1][0], ret[0].decode("ascii")))
                print(ret)
                timeout = 0.01 # don't wait much longer after the first packet arrives
                pass
            except socket.error:
                pass
            pass
        return out
    pass

class utils:
    @staticmethod
    def tomap(s):
        return dict(map(lambda x: x.split('='), s.split()))
    @staticmethod
    def getsocketerror(sock):
        return sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
    @staticmethod
    def str_to_int(s):
        pfx = s[:2]
        if pfx == "0x":
            return int(s[2:], 16)
        elif pfx == "0b":
            return int(s[s:], 2)
        else:
            return int(s, 10)
        pass
    pass

class Protocol:

    __struct_marker = struct.Struct("<IIqffff")
    __struct_rigid = struct.Struct("<IIq28sf")
    __struct_plane = struct.Struct("<IIqHH16sf")
    __struct_plane2 = struct.Struct("<ffff")
    __struct_camera = struct.Struct("<II28sf")
    __struct_peak = struct.Struct("<IIqHHIff")
    __struct_pose = struct.Struct("<fffffff")

    class Header:
        __cksum = struct.Struct("<BBBBBBBB")
        __hdr = struct.Struct("<HBBIq")

        def __init__(self, buf=None):
            if buf: self.id, self.type, self.cksum, self.size, self.time = Protocol.Header.__hdr.unpack_from(buf)
            else:
                self.id = -1
                self.type = -1
                self.cksum = 0
                self.size = 0
                self.time = -1
            pass

        def sum(self):
            s = ctypes.c_uint8(0)
            for b in [self.id, self.type, self.cksum, self.size]:
                while b != 0:
                    s = ctypes.c_uint8(s.value + (0xff & b))
                    b >>= 8
                    pass
                pass
            return s.value

        def valid(self):
            return self.sum() == 0

        @staticmethod
        def pack(buf, id, _type, data):
            if type(data) != bytes: data = bytes(data, "utf-8")
            hdr = Protocol.Header()
            hdr.id, hdr.type, hdr.cksum, hdr.size = id, _type, 0, len(data)
            struct.pack_into("<HBBIq%ds" % len(data), buf, 0, id, _type, 256-hdr.sum(), len(data), 0, data)
            return 16 + len(data)
        pass

    def __init__(self):
        self.inbuffer = bytearray(4 * 1024 * 1024)
        self.outbuffer = bytearray(64 * 1024)
        self.imv = memoryview(self.inbuffer)
        self.omv = memoryview(self.outbuffer)
        self.iposition = 0
        self.header = Protocol.Header()
        self.hdr_size = 16
        pass

    def send(self, sock, id, _type, data):
        s = Protocol.Header.pack(self.outbuffer, id, _type, data)
        if sock.send(self.omv[:s]) != s: return False
        return True

    def read_event(self):
        startpos = self.iposition
        endpos = self.iposition + self.hdr_size
        self.header = self.read_header(self.imv[startpos:endpos])
        if not self.header.valid():
            sys.stderr.write("read_event: invalid checksum!\n")
            return None
        evt = Event(self.header.type, self.header.id, 0, self.header.time)
        startpos = endpos
        endpos += self.header.size
        data = self.imv[startpos:endpos]
        if evt.type_id == Type.MARKER: evt.data = self.read_markers(data)
        elif evt.type_id == Type.RIGID: evt.data = self.read_rigids(data)
        elif evt.type_id == Type.PLANE: evt.data = self.read_planes(data)
        elif evt.type_id == Type.INPUT: evt.data = self.read_inputs(data)
        elif evt.type_id == Type.BYTE or evt.type_id == Type.ERROR: evt.data = self.read_string(data)
        elif evt.type_id == Type.INT: evt.data = self.read_ints(data)
        elif evt.type_id == Type.FLOAT: evt.data = self.read_floats(data)
        elif evt.type_id == Type.CAMERA: evt.data = self.read_cameras(data)
        elif evt.type_id == Type.PEAK: evt.data = self.read_peaks(data)
        elif evt.type_id == Type.EVENT:
            pass
        self.iposition = endpos
        return evt

    def recv(self, sock):
        nbytes = self.read_packet(sock)
        if nbytes <= 0: return None
        events = []
        self.iposition = 0
        while self.iposition < nbytes:
            evt = self.read_event()
            if evt == None: break
            events.append(evt)
            pass
        # reset inbuffer to beginning
        self.iposition = 0
        return events

    def read_packet(self, sock):
        rsocks, wsocks, esocks = select.select([sock], [], [sock], 0)
        if len(rsocks) == 0: return 0
        def check_ret(ret):
            if ret == 0: raise RecvError("recv: connection closed")
            elif ret < 0: raise RecvError("recv: error")
            return
        # read packet header
        if self.iposition < self.hdr_size:
            ret = sock.recv_into(self.imv[self.iposition:], self.hdr_size - self.iposition)
            self.iposition += ret
            check_ret(ret)
            if self.iposition == self.hdr_size:
                self.header = self.read_header(self.imv[:self.hdr_size])
                return self.read_packet(sock)
            return 0
        packet_size = self.hdr_size + self.header.size
        if packet_size > len(self.imv): raise RecvError("insufficient buffer size")
        # read packet data
        if self.iposition < packet_size:
            ret = sock.recv_into(self.imv[self.iposition:], packet_size - self.iposition)
            check_ret(ret)
            self.iposition += ret
            pass
        if self.iposition == packet_size:
            return packet_size
        return 0

    def read_header(self, data):
        return Protocol.Header(data)

    def read_cameras(self, data):
        cameras = [Camera(*self.__struct_camera.unpack_from(data, o)) for o in range(0, len(data), 40)]
        for c in cameras: c.pose = list(self.__struct_pose.unpack(c.pose))
        return cameras

    def read_peaks(self, data):
        return [Peak(*self.__struct_peak.unpack_from(data, o)) for o in range(0, len(data), 32)]

    def read_planes(self, data):
        planes = [Plane(*self.__struct_plane.unpack_from(data, o)) for o in range(0, len(data), 40)]
        for p in planes: p.plane = list(self.__struct_plane2.unpack(p.plane))
        return planes

    def read_inputs(self, data):
        # Input: uint32 count, [ uint64 hw_id, uint64 flags, int64 time, uint32 size, uint8 data[], ]
        count = struct.unpack_from("<I", data)
        o, inputs = 4, []
        while o < len(data):
            i = Input(*struct.unpack_from("<QQqI", data, o))
            size = i.data
            o += 28
            i.data = bytearray(data[o:o+size])
            o += size
            inputs.append(i)
            pass
        return inputs

    def read_markers(self, data):
        return [Marker(*self.__struct_marker.unpack_from(data, o)) for o in range(0, len(data), 32)]

    def read_rigids(self, data):
        rigids = [Rigid(*self.__struct_rigid.unpack_from(data, o)) for o in range(0, len(data), 48)]
        for r in rigids: r.pose = list(self.__struct_pose.unpack(r.pose))
        return rigids

    def read_floats(self, data):
        return list(struct.unpack_from("<%df" % (len(data)/4), data, 0))

    def read_ints(self, data):
        return list(struct.unpack_from("<%di" % (len(data)/4), data, 0))

    def read_string(self, data):
        return struct.unpack_from("<%ds" % (len(data)), data, 0)[0]
    pass

class ProtocolUdp(Protocol):
    def read_packet(self, sock):
        if len(select.select([sock], [], [sock], 0)[0]) == 0:
            return 0
        self.iposition = 0
        ret = sock.recv_into(self.imv, len(self.imv))
        if ret <= 0: raise RecvError("recv: error")
        if ret < self.hdr_size: return 0
        self.iposition = ret
        return ret
    pass


if __name__ == "__main__":
    #
    # Example Program
    #
    import argparse
    parser = argparse.ArgumentParser(description="Scan for or connect to an Impulse OWL server")
    parser.add_argument("--scan", dest="scan", action="store_const", const=True, default=False, help="scan for and list local Impulse servers")
    parser.add_argument("--slave", dest="slave", action="store_const", const=1, default=0, help="enable slave mode")
    parser.add_argument("--freq", dest="freq", action="store", default=None, type=float, help="streaming frequency")
    parser.add_argument("--device", dest="device", action="store", default=None, type=str, help="device to connect to")
    parser.add_argument("--timeout", dest="timeout", action="store", default=5000000, type=int, help="maximum microseconds to wait")
    parser.add_argument("--peaks", dest="peaks", action="store_const", const=1, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--planes", dest="planes", action="store_const", const=1, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--inputs", dest="inputs", action="store_const", const=1, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--hub", dest="hub", action="store_const", const=1, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--options", dest="options", action="store", default=None, type=str, help="additional options")
    parser.add_argument("--debug", dest="debug", action="store_const", const=True, default=False, help="enable debugging output")
    args = parser.parse_args()

    if not args.scan and not args.device:
        print("version: " + __version__)
        parser.print_help()

    if args.scan:
        scan = Scan()
        # broadcast query packet to local servers
        scan.send(str.format("owl.py version: {}", __version__))
        # listen for response packets
        servers = scan.listen(0)
        # print response
        for s in servers:
            print(s)

    if args.device:
        try:
            # create a context
            OWL = Context()
            OWL.debug = args.debug

            # open a tcp connection with optional arguments
            open_options = str.format("timeout={timeout}", **args.__dict__)
            OWL.open(args.device, open_options)

            # set the type of events we want to receive
            init_options = str.format("timeout={timeout} slave={slave}", **args.__dict__)
            if args.options:
                init_options += " " + args.options
                pass

            # start a session with optional arguments
            OWL.initialize(init_options)

            ##it is also possible to pass options this way
            #if args.options:
            #    OWL.options(args.options)
            #    pass

            # (optional) set the streaming frequency
            if args.freq: OWL.frequency(args.freq)

            # other options
            if args.peaks: OWL.option("event.peaks", 1)
            if args.planes: OWL.option("event.planes", 1)
            if args.inputs: OWL.option("event.inputs", 1)
            if args.hub: OWL.option("event.hub", 1)

            #
            advanced_options = args.peaks or args.planes or args.inputs or args.hub

            # enable streaming
            OWL.streaming(1)

            # poll for events
            while OWL.isOpen() and OWL.property("initialized"):
                event = OWL.nextEvent(args.timeout)
                if not event: continue
                if event.type_id == Type.FRAME:
                    if "markers" in event:
                        for m in event.markers:
                            print(m)
                    if "rigids" in event:
                        for r in event.rigids:
                            print(r)
                    if advanced_options:
                        if args.peaks and "peaks" in event:
                            for p in event.peaks:
                                print(p)
                        if args.planes and "planes" in event:
                            for p in event.planes:
                                print(p)
                        if args.inputs and "inputs" in event:
                            for inp in event.inputs:
                                print(inp)
                        if args.hub and "hub" in event:
                            for h in event.hub:
                                print(h)
                elif event.type_id == Type.CAMERA:
                    for camera in event.data:
                        print(camera)
                elif event.type_id == Type.BYTE:
                    if event.name == "done":
                        print("done")
                        break
                    elif event.name == "options":
                        if "event.peaks=1" in event.data:
                            args.peaks = True
                            pass
                        if "event.planes=1" in event.data:
                            args.planes = True
                            pass
                        if "event.inputs=1" in event.data:
                            args.inputs = True
                            pass
                        pass
                    pass
                elif event.type_id == Type.ERROR:
                    print("error", event.type_id, event.data)
                    break
                elif event.type_id == Type.INPUT:
                    for i in event.data:
                        print(i)

            # end session
            OWL.done()
            # release connection
            OWL.close()
        except OpenError:
            print("open error")
            raise
        except InitError:
            print("initialize error")
            raise
        except(OWLError, socket.error):
            print("A problem has occurred!")
            raise
