#!/usr/bin/env python3
import numpy as np
import scipy.ndimage as nd

class SplineByPeakSpeed(object):
    def __init__(self, p_0, v_0, a_0, v_peak, t_peak, t_total, dt):
        # all numpy array, make sure float
        self.p_0 = p_0.astype(float) # 3*1
        self.v_0 = v_0.astype(float) # 3*1
        self.a_0 = a_0.astype(float) # 3*1
        self.v_peak = v_peak.astype(float) # 3*1
        self.t_peak = float(t_peak)
        self.t_total = float(t_total)
        self.dt = float(dt)


    def do_calculation(self):
        # compute the first part of the spline, up to v_peak
        # time vector, assume t_peak/dt is an integer, 1 * ((t_peak-0)/dt+1)
        T_to_peak = np.arange(0.0, self.t_peak + self.dt, self.dt).reshape(1, int((self.t_peak - 0.0) / self.dt) + 1)
    
        # desired acceleration at peak speed
        a_peak = np.array([[0.0], [0.0], [0.0]])

        # compute change in velocity/accel for each axis
        # Dv and Da are 3*1 vectors
        Dv = self.v_peak - self.v_0 - self.a_0 * self.t_peak
        Da = a_peak - self.a_0

        # compute spline parameters
        (ax, bx, cx) = single_axis_params(Dv[0,0], Da[0,0], self.t_peak)
        (ay, by, cy) = single_axis_params(Dv[1,0], Da[1,0], self.t_peak)
        (az, bz, cz) = single_axis_params(Dv[2,0], Da[2,0], self.t_peak)

        a = np.array([[ax, ay, az]])
        b = np.array([[bx, by, bz]])
        c = np.array([[cx, cy, cz]])

        # compute spline
        Z_to_peak = make_spline(T_to_peak, self.p_0, self.v_0, self.a_0, a, b, c)


        # compute second part of the spline, coming to a stop
        # create time vector for second half
        t_to_stop = self.t_total - self.t_peak
        T_to_stop = np.arange(0.0, t_to_stop + self.dt, self.dt).reshape(1, int((t_to_stop - 0.0) / self.dt) + 1)

        # desired end speed and acceleration
        v_f = np.array([[0.0], [0.0], [0.0]])
        a_f = np.array([[0.0], [0.0], [0.0]])

        # for each axis, compute the change in velocity/acceleration
        Dv = v_f - self.v_peak - a_peak * t_to_stop
        Da = a_f - a_peak

        (ax, bx, cx) = single_axis_params(Dv[0,0], Da[0,0], t_to_stop)
        (ay, by, cy) = single_axis_params(Dv[1,0], Da[1,0], t_to_stop)
        (az, bz, cz) = single_axis_params(Dv[2,0], Da[2,0], t_to_stop)

        a = np.array([[ax, ay, az]])
        b = np.array([[bx, by, bz]])
        c = np.array([[cx, cy, cz]])

        # compute spline
        p_peak = Z_to_peak[0:3,-1].reshape(3,1)
        Z_to_stop = make_spline(T_to_stop, p_peak, self.v_peak, a_peak, a, b, c)

        # connect splines and times
        T = np.concatenate((T_to_peak[0, 0 : -1].reshape(1, int((self.t_peak - 0.0) / self.dt)), T_to_stop + self.t_peak), axis=1)
        Z = np.concatenate((Z_to_peak[:, 0 : -1], Z_to_stop), axis=1)

        # return position, velocity, and acceleration
        traj_ref = Z[0:9, :]

        # T is a 1-D numpy array
        T = T.flatten()

        #self.traj_ref = traj_ref
        #self.T = T

        return T, traj_ref


def do_interpolate(time_queue, traj_ref, dt):
    # time_queue in seconds

    # convert time_queue to the corresponding index
    idx_queue = time_queue / dt
    
    # the rows at which you want the data interpolated -- all rows
    rows_interpolate = np.arange(traj_ref.shape[0])
    # the points where you want to interpolate each row 
    columns_interpolate = idx_queue * np.ones((1, traj_ref.shape[0]), dtype=float).flatten()
    
    # result is a 1-D numpy array
    # result = [px, py, pz, vx, vy, vz, ax, ay, az]
    result = nd.map_coordinates(traj_ref, [rows_interpolate, columns_interpolate], order=3, mode='nearest')

    # result is a 2-D numpy array, 9 by 1
    result = np.reshape(result, (-1, 1))

    return result


def single_axis_params(Dv, Da, T):
    # Dv and Da are scalars
    Dv = float(Dv)
    Da = float(Da)
    T = float(T)
    M = np.array([[0.,0.], [-12.,6*T], [6*T,-2*T**2]])
    # out is 1*3
    out = np.transpose((1./T**3)*np.dot(M,np.array([[Dv],[Da]])))
    a = out[0,0]
    b = out[0,1]
    c = out[0,2]
    return (a, b, c)


def make_spline(T_in, p_0, v_0, a_0, a, b, c):
    p_0 = p_0.astype(float)
    v_0 = v_0.astype(float)
    a_0 = a_0.astype(float)
    # a, b, c are 1*3

    # get spline params
    # make sure they are float
    ax = a[0, 0]
    ay = a[0, 1]
    az = a[0, 2]
    bx = b[0, 0]
    by = b[0, 1]
    bz = b[0, 2]
    cx = c[0, 0]
    cy = c[0, 1]
    cz = c[0, 2]

    # position
    px = (ax/120)*T_in**5 + (bx/24)*T_in**4 + (cx/6)*T_in**3 + \
        (a_0[0,0]/2)*T_in**2 + v_0[0,0]*T_in + p_0[0,0]
    py = (ay/120)*T_in**5 + (by/24)*T_in**4 + (cy/6)*T_in**3 + \
        (a_0[1,0]/2)*T_in**2 + v_0[1,0]*T_in + p_0[1,0]
    pz = (az/120)*T_in**5 + (bz/24)*T_in**4 + (cz/6)*T_in**3 + \
        (a_0[2,0]/2)*T_in**2 + v_0[2,0]*T_in + p_0[2,0]

    # speed
    vx = (ax/24)*T_in**4 + (bx/6)*T_in**3 + (cx/2)*T_in**2 + a_0[0,0]*T_in + v_0[0,0]
    vy = (ay/24)*T_in**4 + (by/6)*T_in**3 + (cy/2)*T_in**2 + a_0[1,0]*T_in + v_0[1,0]
    vz = (az/24)*T_in**4 + (bz/6)*T_in**3 + (cz/2)*T_in**2 + a_0[2,0]*T_in + v_0[2,0]

    # acceleration
    ax = (ax/6)*T_in**3 + (bx/2)*T_in**2 + cx*T_in + a_0[0,0]
    ay = (ay/6)*T_in**3 + (by/2)*T_in**2 + cy*T_in + a_0[1,0]
    az = (az/6)*T_in**3 + (bz/2)*T_in**2 + cz*T_in + a_0[2,0]
    
    # jerk
    jx = (ax/2)*T_in**2 + bx*T_in + cx
    jy = (ay/2)*T_in**2 + by*T_in + cy
    jz = (az/2)*T_in**2 + bz*T_in + cz

    # snap
    sx = ax*T_in + bx
    sy = ay*T_in + by
    sz = az*T_in + bz

    # create output traj
    Z_out = np.concatenate((px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz), axis=0)
    return Z_out


if __name__ == "__main__":
    p_0 = np.array([[1.0], [1.0], [1.0]])
    v_0 = np.array([[0.0], [0.0], [0.0]])
    a_0 = np.array([[0.0], [0.0], [0.0]])
    v_peak = np.array([[0.5], [0.5], [0.5]])
    t_peak = 1.5
    t_total = 3.0
    dt = 0.1

    # initialize the class
    spline_class = SplineByPeakSpeed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt)
    # generate the spline trajectory
    T, traj = spline_class.do_calculation()

    print("This is T.")
    print(T)
    print("This is traj.")
    print(traj)