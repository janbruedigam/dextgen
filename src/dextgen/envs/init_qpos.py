DEFAULT_INITIAL_QPOS_PJ = {
    # Arm
    "panda_joint1": 0,
    "panda_joint2": 0.4,
    "panda_joint3": 0.,
    "panda_joint4": -2.4,
    "panda_joint5": 0,
    "panda_joint6": 2.8,
    "panda_joint7": 3.1415/4,
    # Hand
    "robot0:r_gripper_finger_joint": 0.02,
    "robot0:l_gripper_finger_joint": 0.02,
    # Objects
    "cube:joint": [.1, -.1, .025, 1., 0, 0, 0],
    "cylinder:joint": [-.1, .1, .025, 1., 0, 0, 0],
    "sphere:joint": [.1, .1, .025, 1., 0, 0, 0],
    "mesh:joint": [-.1, -.1, .025, 1., 0, 0, 0],
}

DEFAULT_INITIAL_QPOS_Barrett = {
    # Arm
    "panda_joint1": 0,
    "panda_joint2": 0.4,
    "panda_joint3": 0.,
    "panda_joint4": -2.4,
    "panda_joint5": 0,
    "panda_joint6": 2.8,
    "panda_joint7": 3.1415/4,
    # Hand
    "robot0:finger1_prox_joint": 2,
    "robot0:finger2_prox_joint": 2,
    "robot0:finger1_med_joint": 1.22173,
    "robot0:finger2_med_joint": 1.22173,
    "robot0:finger3_med_joint": 1.22173,
    "robot0:finger1_dist_joint": 0.40724,
    "robot0:finger2_dist_joint": 0.40724,
    "robot0:finger3_dist_joint": 0.40724,
    # Objects
    "cube:joint": [.1, -.1, .04, 1., 0, 0, 0],
    "cylinder:joint": [-.1, .1, .04, 1., 0, 0, 0],
    "sphere:joint": [.1, .1, .04, 1., 0, 0, 0],
    "mesh:joint": [-.1, -.1, .04, 1., 0, 0, 0],
}

DEFAULT_INITIAL_QPOS_SH = {
    "cube:joint": [.1, -.1, .025, 1., 0, 0, 0],  # Grasp objects
    "cylinder:joint": [-.1, .1, .025, 1., 0, 0, 0],
    "sphere:joint": [.1, .1, .025, 1., 0, 0, 0],
    "mesh:joint": [-.1, -.1, .025, 1., 0, 0, 0],
    # Arm
    "panda_joint1": 1.5,
    "panda_joint2": 0.4,
    "panda_joint3": -1.5,
    "panda_joint4": -2.,
    "panda_joint5": 0,
    "panda_joint6": 2.8,
    "panda_joint7": 3.1415/4,
    # ShadowHand
    "robot0:WRJ1": -0.1651,
    "robot0:WRJ0": -0.3197,
    "robot0:FFJ3": 0.1434,
    "robot0:FFJ2": 0.3202,
    "robot0:FFJ1": 0.7126,
    "robot0:FFJ0": 0.6705,
    "robot0:MFJ3": 0.0002,
    "robot0:MFJ2": 0.3152,
    "robot0:MFJ1": 0.7659,
    "robot0:MFJ0": 0.7323,
    "robot0:RFJ3": 0.0003,
    "robot0:RFJ2": 0.3674,
    "robot0:RFJ1": 0.7119,
    "robot0:RFJ0": 0.6699,
    "robot0:LFJ4": 0.0525,
    "robot0:LFJ3": -0.1361,
    "robot0:LFJ2": 0.3987,
    "robot0:LFJ1": 0.7415,
    "robot0:LFJ0": 0.7040,
    "robot0:THJ4": 0.0036,
    "robot0:THJ3": 0.5506,
    "robot0:THJ2": -0.0145,
    "robot0:THJ1": -0.0015,
    "robot0:THJ0": -0.7894,
}