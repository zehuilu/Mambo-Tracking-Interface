import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')

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
v_peak = np.array([[0.5], [-0.5], [-0.2]])

gen_class = gen_spline.generate_spline_by_peak_speed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt_traj)
time_ref, traj_ref = gen_class.do_calculation()


# initialization
time_list = []
yaw_des_list = []
pitch_des_list = []
roll_des_list = []

# some variables
dt = dt_traj
t_now = 0.0
mass_drone = 0.1 # [kg] # 100 gramms
#    phasespace    controller
#        X             Y
#        Y             Z
#        Z             X
# gravity vector in new frame
gravity_vector = np.array([[0.0], [0.0], [-9.81]]) # [m/s2]

idx = -20
while t_now < 3.0:
    t0 = time.time()
    if idx == 0:
        t_start = t0

    if idx < 0:
        # Do nothing
        time.sleep(dt_traj)
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

        # remember: in mocap system, euler angles depends on the fixed global axes,
        # but for drone commands, euler angles depends on the current orientation
        # need to stablize yaw very quickly

        # a_ref are the next point
        F_d = - mass_drone*gravity_vector + mass_drone*a_forward # 3 by 1

        z_B_d = F_d / np.linalg.norm(F_d) # 3 by 1
        #z_B = Rot_Mat[:, 2] # 3 by 1

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

        # record
        time_list.append(t_now)

        yaw_des_list.append(degrees(yaw_des))
        pitch_des_list.append(degrees(pitch_des))
        roll_des_list.append(degrees(roll_des))

        time.sleep(dt_traj)
        
    t1 = time.time()
    dt = t1 - t0
    print("time interval for fly command")
    print(dt)
    idx += 1


# plot
# plot 0, actual/desired 3-D trajectory
fig_0 = plt.figure()
ax0 = fig_0.gca(projection="3d", title="Trajectory")
ax0.plot(traj_ref[0, :], traj_ref[1, :], traj_ref[2, :], color="blue", linestyle="-", label="Reference Trajectory")
ax0.plot([traj_ref[0, 0]], [traj_ref[1, 0]], [traj_ref[2, 0]], marker="D", label="Reference Origin")
ax0.plot([traj_ref[0, -1]], [traj_ref[1, -1]], [traj_ref[2, -1]], marker="x", label="Reference Destination")
ax0.set_xlabel("X Label")
ax0.set_ylabel("Y Label")
ax0.set_zlabel("Z Label")
X = np.array(traj_ref[0, :])
Y = np.array(traj_ref[1, :])
Z = np.array(traj_ref[2, :])
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

print(yaw_des_list)

plt.show()