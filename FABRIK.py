import numpy as np
from numpy import linalg as LA


def fabrik(joint_pos, target, tolerance=0.01):
    """
    :param joint_pos: Joints positions of your arm. List of np.array() points
    :param target: Target point Single np.array() point
    :param tolerance: Tolerance for the result
    :return: list of target joint positions
    """
    n = len(joint_pos)

    dist_joints = []  # d
    for idx in range(n-1):
        dist_joints.append(LA.norm(joint_pos[idx+1]-joint_pos[idx]))

    dist_root_target = LA.norm(joint_pos[0]-target)
    if dist_root_target > sum(dist_joints):
        for idx in range(n-1):
            r = LA.norm(target-joint_pos[idx])
            lambda_ = dist_joints[idx]/r
            joint_pos[idx+1] = (1-lambda_)*joint_pos[idx] + lambda_*target
    else:
        # Target is reachable so set b as the initial pos of the joint p1
        b = joint_pos[0]
        # Check whether the distance between the en effector pn and
        # the target t is greater than a tolerance diffA = |pn-t|
        diffA = LA.norm(joint_pos[n-1]-target)
        while diffA > tolerance:
            # STAGE 1 forward reaching
            # set end effector pn as target t
            joint_pos[n-1] = target # pn = t
            for i in range(n-2, -1, -1):
                r = LA.norm(joint_pos[i+1]-joint_pos[i])
                lambda_ = dist_joints[i]/r
                joint_pos[i] = (1-lambda_)*joint_pos[i+1] + lambda_*joint_pos[i]

            # STAGE 2 backward reaching
            # set root p1 its initial position
            joint_pos[0] = b
            for i in range(n-1):
                # find distance between new joint i pos and joint i+1
                r = LA.norm(joint_pos[i+1]-joint_pos[i])
                lambda_ = dist_joints[i]/r
                joint_pos[i+1] = (1-lambda_)*joint_pos[i] + lambda_*joint_pos[i+1]

            diffA = LA.norm(joint_pos[n-1]-target)
    return joint_pos
