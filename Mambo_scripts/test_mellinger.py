import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

from pyparrot.Minidrone import Mambo
import socket
import numpy as np
import transforms3d
import time
from math import degrees, radians, sin, cos, sqrt
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import generate_spline_peak_speed as gen_spline


################################################
# generate a desired trajectory
# those are all in the controller frame
p_0 = np.array([[0.0], [0.0], [1.0]])
v_0 = np.array([[0.0], [0.0], [0.0]])
a_0 = np.array([[0.0], [0.0], [0.0]])
t_peak = 1.5
t_total = 3.0
dt_traj = 0.10
v_peak = np.array([[0.0], [0.0], [0.0]])

gen_class = gen_spline.generate_spline_by_peak_speed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt_traj)
T, traj_ref = gen_class.do_calculation()


# initialization
time_list = []
yaw_list = []
pitch_list = []
roll_list = []
px_list = []
py_list = []
pz_list = []
vx_list = []
vy_list = []
vz_list = []
yaw_cmd_list = []
pitch_cmd_list = []
roll_cmd_list = []
vz_cmd_list = []

tilt_max = radians(25.0) # in degrees to radians
vz_max = 2.0 # in m/s
yaw_rate_max = 180.0 # in degrees/sec

# some variables
posi_pre = np.array([[0.0], [0.0], [0.0]])
velo_now = np.array([[0.0], [0.0], [0.0]])
dt = dt_traj
t_now = 0.0
mass_drone = 0.1 # [kg] # 100 gramms
#    phasespace    controller
#        X             Y
#        Y             Z
#        Z             X
# gravity vector in new frame
gravity_vector = np.array([[0.0], [0.0], [-9.81]]) # [m/s2]

# gains for PID controller
K_p = np.array([\
    [0.0, 0.0, 0.0], \
    [0.0, 0.0, 0.0], \
    [0.0, 0.0, 0.0]])

K_v = np.array([\
    [0.0, 0.0, 0.0], \
    [0.0, 0.0, 0.0], \
    [0.0, 0.0, 0.0]])


##################################################3
# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect the socket to the port where the server is listening
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 9000        # Port to listen on (non-privileged ports are > 1023)
server_address = (HOST, PORT)
print("connecting to", server_address)
sock.connect(server_address)



# you will need to change this to the address of YOUR mambo
# this is for Mambo_628236
mamboAddr = "D0:3A:93:36:E6:21"
# make my mambo object
# remember to set True/False for the wifi depending on if you are using the wifi or the BLE to connect
mambo = Mambo(mamboAddr, use_wifi=False)
print("trying to connect")
success = mambo.connect(num_retries=3)
print("connected: %s" % success)

if (success):
    # calibrate the Mambo
    mambo.flat_trim()
    print("calibrated the Mambo")

    # set the maximum tilt angle in degrees
    # tilt_max is in radians!!!
    mambo.set_max_tilt(degrees(tilt_max))
    print("set the maximum tilt angle")

    # set the maximum vertical speed in m/s
    mambo.set_max_vertical_speed(vz_max)
    print("set the maximum vertical speed")
    
    # get the state information
    print("sleeping")
    mambo.smart_sleep(2)
    mambo.ask_for_state_update()
    mambo.smart_sleep(2)

    print("taking off!")
    mambo.safe_takeoff(5)

    idx = -10
    while t_now < 1.0:
        t0 = time.time()
        if idx == 0:
            t_start = t0

        msg = sock.recv(4096)

        if msg:
            
            data = np.fromstring(msg, dtype=float)
            data = data[-7:]
            # 2-D numpy array, 3 by 1, [px; py; pz], in meters
            posi_now = np.reshape(data[0:3], (-1, 1))
            # 1-D numpy array to list, [w, x, y, z]
            ori_quat = data[-4:].tolist()
            # compute the velocity
            velo_now = (posi_now - posi_pre) / dt
            posi_pre = posi_now

            if idx < 0:
                # Do nothing
                mambo.smart_sleep(dt_traj)
            else:
                t_now = t0 - t_start
                # load the current and the next desired points
                # the current desired point
                # 2-D numpy array, 9 by 1
                # px, py, pz, vx, vy, vz, ax, ay, az
                point_ref_0 = gen_spline.do_interpolate(t_now, traj_ref, dt_traj)
                point_ref_1 = gen_spline.do_interpolate(t_now + dt_traj, traj_ref, dt_traj)
                
                # the acceleration feedforward, 3 by 1
                a_forward = np.reshape(point_ref_1[6:, 0], (-1, 1))

                e_p = posi_now - np.reshape(point_ref_0[0:3, 0], (-1, 1)) # 3 by 1
                e_v = velo_now - np.reshape(point_ref_0[3:6, 0], (-1, 1)) # 3 by 1

                # remember: in mocap system, euler angles depends on the fixed global axes,
                # but for drone commands, euler angles depends on the current orientation
                # need to stablize yaw very quickly

                # current rotation matrix
                Rot_Mat = transforms3d.quaternions.quat2mat(ori_quat)
                # euler angles only uses for plots, in phasespace frame
                yaw, pitch, roll = transforms3d.euler.quat2euler(ori_quat, axes='syzx')
                yaw = degrees(yaw)
                pitch = degrees(pitch)
                roll = degrees(roll)
                # euler angles in command frame
                yaw = -1.0 * yaw
                pitch = -1.0 * pitch

                # a_ref are the next point
                F_d = -np.dot(K_p,e_p) - np.dot(K_v,e_v) - mass_drone*gravity_vector + mass_drone*a_forward # 3 by 1

                z_B_d = F_d / np.linalg.norm(F_d) # 3 by 1
                z_B = Rot_Mat[:, 2] # 3 by 1

                # don't use it
                #thrust_total = np.dot(F_d.T, z_B) # total thrust, scalar

                # x_C_d = np.array([[cos(yaw_des)], [sin(yaw_des)], [0.0]]) # 3 by 1
                # yaw_des = 0.0 degrees
                x_C_d = np.array([[1.0], [0.0], [0.0]]) # 3 by 1

                y_B_d_dir = np.cross(z_B_d.T, x_C_d.T).T # 3 by 1
                y_B_d = y_B_d_dir / np.linalg.norm(y_B_d_dir) # 3 by 1
                x_B_d = np.cross(y_B_d.T, z_B_d.T).T # 3 by 1
                R_d = np.concatenate((x_B_d, y_B_d, z_B_d), axis=1) # 3 by 3

                # euler angles in phasespace frame (radians)
                yaw_des, pitch_des, roll_des = transforms3d.euler.mat2euler(R_d, axes='syzx') 
                # euler angles in command frame
                yaw_des = -1.0 * yaw_des
                pitch_des = -1.0 * pitch_des

                cmd_roll = roll_des / tilt_max * 100.0
                cmd_pitch = pitch_des / tilt_max * 100.0

                # compute the yaw rate
                cmd_yaw_rate = (degrees(yaw_des) - yaw) / dt_traj / yaw_rate_max * 100.0

                # compute the vertical speed
                # the global vertical speed for the next desired point
                cmd_vz = point_ref_1[5, 0] / vz_max * 100.0


########################3
                px_list.append(px)
                py_list.append(py)
                pz_list.append(pz)
                yaw_list.append(yaw)
                pitch_list.append(pitch)
                roll_list.append(roll)
                time_list.append(t_now)


                mambo.smart_sleep(dt_traj)
                #mambo.fly_direct(0, 100, 0, 0, dt_traj)
                
                

            t1 = time.time()
            dt = t1 - t0
            print("time interval for fly command")
            print(dt)
            idx += 1

        else:
            break
            print("Didn't receive the motion capture message")
            print("Land!")
            mambo.safe_land(5)

    mambo.fly_direct(0, 0, 0, 0, dt_traj)         
    
    print("landing")
    mambo.safe_land(5)

    print("disconnect")
    mambo.disconnect()


    # plot
    fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
    ax1.plot(time_list, yaw_list, color="red", linestyle="-", label="yaw")
    ax1.legend(loc="lower right")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Yaw [degree]")
    ax2.plot(time_list, pitch_list, color="red", linestyle="-", label="pitch")
    ax2.legend(loc="lower right")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Pitch [degree]")
    ax3.plot(time_list, roll_list, color="red", linestyle="-", label="roll")
    ax3.legend(loc="lower right")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Roll [degree]")
    plt.tight_layout()

    plt.show()