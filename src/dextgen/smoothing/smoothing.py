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
        

    def filter_signal(self, y):
        # pad the signal at the extremes with values taken from the signal itself
        firstvals = y[0] - (y[1:self.half_window+1][::-1] - y[0])
        lastvals = y[-1] - (y[-self.half_window-1:-1][::-1] - y[-1])
        y = np.concatenate((firstvals, y, lastvals))

        return np.convolve(self.m[::-1], y, mode='valid')
    
    def filter_trajectory(self, trajectory):
        smoothed_x = self.filter_signal([arr[0] for arr in trajectory])
        smoothed_y = self.filter_signal([arr[1] for arr in trajectory])
        smoothed_z = self.filter_signal([arr[2] for arr in trajectory])
        smoothed = [np.array([smoothed_x[i],smoothed_y[i],smoothed_z[i]]) for i in range(0, len(trajectory))]

        return smoothed
    
    def smoothing(self, sim_traj_to_obj_pos, sim_traj_to_obj_rot, sim_traj_to_goal_pos, sim_traj_to_goal_rot, sim_obj_pos, sim_obj_rot, real_obj_pos, real_obj_rot):
        N_to_obj = len(sim_traj_to_obj_pos)
        N_to_goal = len(sim_traj_to_goal_pos)

        sim_obj_to_sim_ee_pos = sim_traj_to_obj_pos[-1] - sim_obj_pos
        sim_obj_to_or_pos = real_obj_pos - sim_obj_pos
        sim_obj_to_sim_ee_rot = quat_mul(quat_conjugate(sim_obj_rot),sim_traj_to_obj_rot[-1])
        sim_obj_to_or_rot = quat_mul(quat_conjugate(sim_obj_rot),real_obj_rot)

        delta_to_obj_pos = -sim_obj_to_sim_ee_pos + sim_obj_to_or_pos + quat_vec_rotate(quat_mul(real_obj_rot,quat_conjugate(sim_obj_rot)),sim_obj_to_sim_ee_pos)
        delta_to_obj_rot = quat_mul(quat_mul(quat_conjugate(sim_obj_to_sim_ee_rot),sim_obj_to_or_rot),sim_obj_to_sim_ee_rot)

        real_traj_to_obj_pos = [sim_traj_to_obj_pos[i] + delta_to_obj_pos*i/(N_to_obj-1) for i in range(0, N_to_obj)]
        real_traj_to_obj_rot = [quat_mul(sim_traj_to_obj_rot[i],quat_to_pow(delta_to_obj_rot,i/(N_to_obj-1))) for i in range(0, N_to_obj)]

        delta_to_goal_pos = real_traj_to_obj_pos[-1] - sim_traj_to_goal_pos[0]
        delta_to_goal_rot = quat_mul(quat_conjugate(sim_traj_to_goal_rot[0]),real_traj_to_obj_rot[-1])

        real_traj_to_goal_pos = [sim_traj_to_goal_pos[i] + delta_to_goal_pos*(N_to_goal-1-i)/(N_to_goal-1) for i in range(0, N_to_goal)]
        real_traj_to_goal_rot = [quat_mul(sim_traj_to_goal_rot[i],quat_to_pow(delta_to_goal_rot,(N_to_goal-1-i)/(N_to_goal-1))) for i in range(0, N_to_goal)]

        if self.smooth:
            real_traj_to_obj_pos_smooth = self.filter_trajectory(real_traj_to_obj_pos)

            real_traj_to_obj_rot_aa = [log_quat(arr)[1:4] for arr in real_traj_to_obj_rot]
            real_traj_to_obj_rot_aa_smooth = self.filter_trajectory(real_traj_to_obj_rot_aa)
            real_traj_to_obj_rot_smooth = [exp_quat(np.concatenate((np.array([0]),real_traj_to_obj_rot_aa_smooth[i]))) for i in range(0, N_to_obj)]

            real_traj_to_goal_pos_smooth = self.filter_trajectory(real_traj_to_goal_pos)

            real_traj_to_goal_rot_aa = [log_quat(arr)[1:4] for arr in real_traj_to_goal_rot]
            real_traj_to_goal_rot_aa_smooth = self.filter_trajectory(real_traj_to_goal_rot_aa)
            real_traj_to_goal_rot_smooth = [exp_quat(np.concatenate((np.array([0]),real_traj_to_goal_rot_aa_smooth[i]))) for i in range(0, N_to_goal)]

            return real_traj_to_obj_pos_smooth, real_traj_to_obj_rot_smooth, real_traj_to_goal_pos_smooth, real_traj_to_goal_rot_smooth
        else:
            return real_traj_to_obj_pos, real_traj_to_obj_rot, real_traj_to_goal_pos, real_traj_to_goal_rot