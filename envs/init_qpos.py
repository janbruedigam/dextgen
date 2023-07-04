DEFAULT_INITIAL_QPOS_PJ = {
    # Arm
    "panda_joint1": 0,
    "panda_joint2": 0.4,
    "panda_joint3": 0.,
    "panda_joint4": -2.4,
    "panda_joint5": 0,
    "panda_joint6": 2.8,
    "panda_joint7": 0,
    # Hand
    "robot0:r_gripper_finger_joint": 0.2,
    "robot0:l_gripper_finger_joint": 0.2,
    # Objects
    "cube:joint": [.1, -.1, .025, 1., 0, 0, 0],
    "cylinder:joint": [-.1, .1, .025, 1., 0, 0, 0],
    "sphere:joint": [.1, .1, .025, 1., 0, 0, 0],
    "mesh:joint": [-.1, -.1, .025, 1., 0, 0, 0],
}