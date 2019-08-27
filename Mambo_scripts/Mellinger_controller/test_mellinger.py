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
p_0 = np.array([[0.0], [0.0], [0.6]])
v_0 = np.array([[0.0], [0.0], [0.0]])
a_0 = np.array([[0.0], [0.0], [0.0]])
t_peak = 1.5
t_total = 3.0
dt_traj = 0.10

###############################################
# vx points right, vy points front, always check the length of trajectory
v_peak = np.array([[0.5], [0.0], [0.0]])

gen_class = gen_spline.generate_spline_by_peak_speed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt_traj)
time_ref, traj_ref = gen_class.do_calculation()


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
yaw_des_list = []
pitch_des_list = []
roll_des_list = []
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

    #print("taking off!")
    #mambo.safe_takeoff(5)

    idx = -20
    while t_now < 3.0:
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
                F_d = - np.dot(K_p,e_p) - np.dot(K_v,e_v) - mass_drone*gravity_vector + mass_drone*a_forward # 3 by 1

                z_B_d = F_d / np.linalg.norm(F_d) # 3 by 1
                z_B = Rot_Mat[:, 2] # 3 by 1

                # don't use it
                #thrust_total = np.dot(F_d.T, z_B) # total thrust, scalar

                # x_C_d = np.array([[cos(yaw_des)], [sin(yaw_des)], [0.0]]) # 3 by 1
                # yaw_des = 0.0 degrees
                y_C_d = np.array([[-1.0], [0.0], [0.0]]) # 3 by 1

                x_B_d_dir = np.cross(y_C_d.T, z_B_d.T).T # 3 by 1
                x_B_d = x_B_d_dir / np.linalg.norm(x_B_d_dir) # 3 by 1
                y_B_d = np.cross(z_B_d.T, x_B_d.T).T # 3 by 1

                ####################################3
                R_d = np.concatenate((y_B_d, x_B_d, z_B_d), axis=1) # 3 by 3

                # euler angles in phasespace frame (radians)
                yaw_des, pitch_des, roll_des = transforms3d.euler.mat2euler(R_d, axes='szxy') 
                # euler angles in command frame (radians)
                yaw_des = -1.0 * yaw_des
                pitch_des = -1.0 * pitch_des

                cmd_roll = roll_des / tilt_max * 100.0
                cmd_pitch = pitch_des / tilt_max * 100.0

                # compute the yaw rate
                cmd_yaw_rate = ((degrees(yaw_des)-yaw)/dt_traj)/ yaw_rate_max * 100.0

                # compute the vertical speed
                # the global vertical speed for the next desired point
                cmd_vz = point_ref_1[5, 0] / vz_max * 100.0


                # record
                px_list.append(posi_now[0, 0])
                py_list.append(posi_now[1, 0])
                pz_list.append(posi_now[2, 0])

                vx_list.append(velo_now[0, 0])
                vy_list.append(velo_now[1, 0])
                vz_list.append(velo_now[2, 0])

                yaw_list.append(yaw)
                pitch_list.append(pitch)
                roll_list.append(roll)
                time_list.append(t_now)

                yaw_des_list.append(degrees(yaw_des))
                pitch_des_list.append(degrees(pitch_des))
                roll_des_list.append(degrees(roll_des))

                yaw_cmd_list.append(cmd_yaw_rate)
                pitch_cmd_list.append(cmd_pitch)
                roll_cmd_list.append(cmd_roll)
                vz_cmd_list.append(cmd_vz)


                mambo.smart_sleep(dt_traj)
                #mambo.fly_direct(cmd_roll, cmd_pitch, cmd_yaw_rate, cmd_vz, dt_traj)
                
                

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

    #mambo.fly_direct(0, 0, 0, 0, dt_traj)         
    
    #print("landing")
    #mambo.safe_land(5)

    print("disconnect")
    mambo.disconnect()


    # plot

    # plot 0, actual/desired 3-D trajectory
    fig_0 = plt.figure()
    ax0 = fig_0.gca(projection="3d", title="Trajectory")
    ax0.plot(traj_ref[0, :], traj_ref[1, :], traj_ref[2, :], color="blue", linestyle="-", label="Reference Trajectory")
    ax0.plot([traj_ref[0, 0]], [traj_ref[1, 0]], [traj_ref[2, 0]], marker="D", label="Reference Origin")
    ax0.plot([traj_ref[0, -1]], [traj_ref[1, -1]], [traj_ref[2, -1]], marker="x", label="Reference Destination")
    ax0.plot(px_list, py_list, pz_list, color="red", linestyle="-", label="Actual Trajectory") 
    ax0.plot([px_list[0]], [py_list[0]], [pz_list[0]], marker="s", label="Actual Origin") 
    ax0.plot([px_list[-1]], [py_list[-1]], [pz_list[-1]], marker="v", label="Actual Destination") 
    ax0.set_xlabel("X Label")
    ax0.set_ylabel("Y Label")
    ax0.set_zlabel("Z Label")
    X = np.array(px_list)
    Y = np.array(py_list)
    Z = np.array(pz_list)
    max_range = np.array([X.max()-X.min(), \
        Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
    mid_x = (X.max() + X.min()) * 0.5
    mid_y = (Y.max() + Y.min()) * 0.5
    mid_z = (Z.max() + Z.min()) * 0.5
    ax0.set_xlim(mid_x - max_range, mid_x + max_range)
    ax0.set_ylim(mid_y - max_range, mid_y + max_range)
    ax0.set_zlim(mid_z - max_range, mid_z + max_range)
    plt.legend(loc="upper left", shadow=False)
    plt.legend(loc="upper left", shadow=False)
    plt.draw()

    # plot 1, actual/desired positions
    fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
    ax1.plot(time_list, px_list, color="red", linestyle="-", label="actual px")
    ax1.plot(time_ref.tolist(), traj_ref[0, :].tolist(), color="blue", linestyle="--", label="desired px")
    ax1.legend(loc="upper left")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("px [m]")
    ax2.plot(time_list, py_list, color="red", linestyle="-", label="actual py")
    ax2.plot(time_ref.tolist(), traj_ref[1, :].tolist(), color="blue", linestyle="--", label="desired py")
    ax2.legend(loc="upper left")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("py [m]")
    ax3.plot(time_list, pz_list, color="red", linestyle="-", label="actual pz")
    ax3.plot(time_ref.tolist(), traj_ref[2, :].tolist(), color="blue", linestyle="--", label="desired pz")
    ax3.legend(loc="upper left")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("pz [m]")
    plt.tight_layout()

    # plot 2, actual euler angles
    fig_2, (ax4, ax5, ax6) = plt.subplots(3, 1)
    ax4.plot(time_list, yaw_list, color="red", linestyle="-", label="actual yaw")
    ax4.legend(loc="upper left")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Yaw [degree]")
    ax5.plot(time_list, pitch_list, color="red", linestyle="-", label="actual pitch")
    ax5.legend(loc="upper left")
    ax5.set_xlabel("Time [s]")
    ax5.set_ylabel("Pitch [degree]")
    ax6.plot(time_list, roll_list, color="red", linestyle="-", label="actual roll")
    ax6.legend(loc="upper left")
    ax6.set_xlabel("Time [s]")
    ax6.set_ylabel("Roll [degree]")
    plt.tight_layout()

    # plot 3, desired euler angles
    fig_3, (ax7, ax8, ax9) = plt.subplots(3, 1)
    ax7.plot(time_list, yaw_des_list, color="red", linestyle="-", label="desired yaw angle")
    ax7.legend(loc="upper left")
    ax7.set_xlabel("Time [s]")
    ax7.set_ylabel("desired yaw [degree]")
    ax8.plot(time_list, pitch_des_list, color="red", linestyle="-", label="desired pitch")
    ax8.legend(loc="upper left")
    ax8.set_xlabel("Time [s]")
    ax8.set_ylabel("desired pitch [degree]")
    ax9.plot(time_list, roll_des_list, color="red", linestyle="-", label="desired roll")
    ax9.legend(loc="upper left")
    ax9.set_xlabel("Time [s]")
    ax9.set_ylabel("desired roll [degree]")
    plt.tight_layout()


    # plot 4, inputs which were sent to the Mambo
    fig_4, ((ax10, ax11), (ax12, ax13)) = plt.subplots(2, 2)
    ax10.plot(time_list, yaw_cmd_list, color="red", linestyle="-", label="yaw rate command (unsaturated)")
    ax10.legend(loc="upper left")
    ax10.set_xlabel("Time [s]")
    ax10.set_ylabel("yaw rate command [percentage]")
    ax11.plot(time_list, pitch_cmd_list, color="red", linestyle="-", label="pitch command (unsaturated)")
    ax11.legend(loc="upper left")
    ax11.set_xlabel("Time [s]")
    ax11.set_ylabel("pitch command [percentage]")
    ax12.plot(time_list, roll_cmd_list, color="red", linestyle="-", label="roll command (unsaturated)")
    ax12.legend(loc="upper left")
    ax12.set_xlabel("Time [s]")
    ax12.set_ylabel("roll command [percentage]")
    ax13.plot(time_list, vz_cmd_list, color="red", linestyle="-", label="vertical speed command (unsaturated)")
    ax13.legend(loc="upper left")
    ax13.set_xlabel("Time [s]")
    ax13.set_ylabel("vertical speed command [percentage]")
    plt.tight_layout()

    # plot 5, actual/desired velocities
    fig_5, (ax14, ax15, ax16) = plt.subplots(3, 1)
    ax14.plot(time_list, vx_list, color="red", linestyle="-", label="actual vx")
    ax14.plot(time_ref.tolist(), traj_ref[3, :].tolist(), color="blue", linestyle="--", label="desired vx")
    ax14.legend(loc="upper left")
    ax14.set_xlabel("Time [s]")
    ax14.set_ylabel("vx [m]")
    ax15.plot(time_list, vy_list, color="red", linestyle="-", label="actual vy")
    ax15.plot(time_ref.tolist(), traj_ref[4, :].tolist(), color="blue", linestyle="--", label="desired vy")
    ax15.legend(loc="upper left")
    ax15.set_xlabel("Time [s]")
    ax15.set_ylabel("vy [m]")
    ax16.plot(time_list, vz_list, color="red", linestyle="-", label="actual vz")
    ax16.plot(time_ref.tolist(), traj_ref[5, :].tolist(), color="blue", linestyle="--", label="desired vz")
    ax16.legend(loc="upper left")
    ax16.set_xlabel("Time [s]")
    ax16.set_ylabel("vz [m]")
    plt.tight_layout()

    plt.show()