import sys
sys.path.append('/home/roahmlab/Mambo_scripts/lib')
import scipy.ndimage as nd
import numpy as np
import generate_spline_peak_speed as gen_spline

traj_ref= np.array([[1,2,3,4,5], [10,20,30,40,50], [100,200,300,400,500], [1000,2000,3000,4000,5000]])

t_queue = 1.5

# the rows at which you want the data interpolated -- all rows
r = np.arange(traj_ref.shape[0])
# the points where you want to interpolate each row 
y = t_queue * np.ones((1, traj_ref.shape[0]), dtype=float).flatten()

result = nd.map_coordinates(traj_ref, [r, y], order=1, mode='nearest')
print(result)
print(traj_ref)
print(y)
print(r)
