import numpy as np
from math import factorial
from dextgen.envs.rotations import quat_mul, quat_conjugate, quat_vec_rotate, quat_to_pow, exp_quat, log_quat

class Smoothing():
    """Savitzky Golay Filter"""

    def __init__(self, smooth=True, window_size=21, order=4, deriv=0, rate=1):
        if window_size % 2 != 1 or window_size < 1:
            raise TypeError("window_size size must be a positive odd number")
        if window_size < order + 2:
            raise TypeError("window_size is too small for the polynomials order")
        
        self.smooth = smooth
        self.half_window = (window_size -1) // 2

        order_range = range(order+1)
        b = np.mat([[k**i for i in order_range] for k in range(-self.half_window, self.half_window+1)])
        self.m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
        

    def filter(self, y):
        # pad the signal at the extremes with values taken from the signal itself
        firstvals = y[0] - (y[1:self.half_window+1][::-1] - y[0])
        lastvals = y[-1] - (y[-self.half_window-1:-1][::-1] - y[-1])
        y = np.concatenate((firstvals, y, lastvals))

        return np.convolve(self.m[::-1], y, mode='valid')
    
    def smoothing(self, sim_trajectory_pos, sim_trajectory_rot, sim_object_pos, sim_object_rot, real_object_pos, real_object_rot):
        N = len(sim_trajectory_pos)
        sim_object_to_sim_ee_pos = sim_trajectory_pos[-1] - sim_object_pos
        sim_object_to_or_pos = real_object_pos - sim_object_pos
        sim_object_to_sim_ee_rot = quat_mul(quat_conjugate(sim_object_rot),sim_trajectory_rot[-1])
        sim_object_to_or_rot = quat_mul(quat_conjugate(sim_object_rot),real_object_rot)

        delta_pos = -sim_object_to_sim_ee_pos + sim_object_to_or_pos + quat_vec_rotate(quat_mul(real_object_rot,quat_conjugate(sim_object_rot)),sim_object_to_sim_ee_pos)
        delta_rot = quat_mul(quat_mul(quat_conjugate(sim_object_to_sim_ee_rot),sim_object_to_or_rot),sim_object_to_sim_ee_rot)

        real_trajectory_pos = [sim_trajectory_pos[i] + delta_pos*i/N for i in range(0, N)]
        real_trajectory_rot = [quat_mul(sim_trajectory_rot[i],quat_to_pow(delta_rot,i/N)) for i in range(0, N)]

        if self.smooth:
            real_trajectory_pos_smooth_x = self.filter([arr[0] for arr in real_trajectory_pos])
            real_trajectory_pos_smooth_y = self.filter([arr[1] for arr in real_trajectory_pos])
            real_trajectory_pos_smooth_z = self.filter([arr[2] for arr in real_trajectory_pos])
            real_trajectory_pos_smooth = [np.array([real_trajectory_pos_smooth_x[i],real_trajectory_pos_smooth_y[i],real_trajectory_pos_smooth_z[i]]) for i in range(0, N)]

            real_trajectory_rot_aa = [log_quat(arr)[1:4] for arr in real_trajectory_rot]

            real_trajectory_rot_smooth_x = self.filter([arr[0] for arr in real_trajectory_rot_aa])
            real_trajectory_rot_smooth_y = self.filter([arr[1] for arr in real_trajectory_rot_aa])
            real_trajectory_rot_smooth_z = self.filter([arr[2] for arr in real_trajectory_rot_aa])
            real_trajectory_rot_smooth = [exp_quat(np.concatenate((np.array([0]),np.array([real_trajectory_rot_smooth_x[i],real_trajectory_rot_smooth_y[i],real_trajectory_rot_smooth_z[i]])))) for i in range(0, N)]

            return real_trajectory_pos_smooth, real_trajectory_rot_smooth
        else:
            return real_trajectory_pos, real_trajectory_rot