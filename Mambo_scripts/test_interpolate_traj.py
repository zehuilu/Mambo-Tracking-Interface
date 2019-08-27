import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')
import scipy.ndimage as nd
import numpy as np
import generate_spline_peak_speed as gen_spline



# generate a desired trajectory
p_0 = np.array([[0.0], [0.0], [0.6]])
v_0 = np.array([[0.0], [0.0], [0.0]])
a_0 = np.array([[0.0], [0.0], [0.0]])
t_peak = 0.5
t_total = 3.0
dt_traj = 0.10
v_peak = np.array([[0.0], [0.5], [0.0]])

gen_class = gen_spline.generate_spline_by_peak_speed(p_0, v_0, a_0, v_peak, t_peak, t_total, dt_traj)
T, traj_ref = gen_class.do_calculation()

# index, rather than the value of time
time_queue = 1.5 # in seconds
idx_queue = time_queue / dt_traj

# the rows at which you want the data interpolated -- all rows
rows_interpolate = np.arange(traj_ref.shape[0])
# the points where you want to interpolate each row 
columns_interpolate = idx_queue * np.ones((1, traj_ref.shape[0]), dtype=float).flatten()

result = nd.map_coordinates(traj_ref, [rows_interpolate, columns_interpolate], order=3, mode='nearest')
print(traj_ref)
print(columns_interpolate)
print(rows_interpolate)
print(T)


result_1 = gen_spline.do_interpolate(time_queue, traj_ref, dt_traj)
print(result)
print(result_1)