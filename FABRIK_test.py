import numpy as np
from numpy import linalg as LA
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib

import time

# creation of the robotic arm
joint_pos_input = []  # p
for i in range(0,110,10):
    joint_pos_input.append(np.array([i, 0, 0]))
target = np.array([20, 60, 20])


min, max = -10, 100

matplotlib.interactive(True)

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
# ax = fig.gca(projection='3d')
ax = fig.add_subplot(111, projection='3d')
# !!!!


def draw(joints, target):
    plt.cla()
    x_pts = []
    y_pts = []
    z_pts = []
    for i in joints:
        x_pts.append(i[0])
        y_pts.append(i[1])
        z_pts.append(i[2])
    ax.set_xlim3d(min, max)
    ax.set_ylim3d(min, max)
    ax.set_zlim3d(min, max)

    ax.plot(x_pts, y_pts, z_pts, marker='o', markersize=10)
    plt.plot([target[0]], [target[1]], [target[2]], color='r', marker='o', markersize=10)
    plt.draw()
    fig.canvas.flush_events()


def fabrikWithDrawing(joint_pos, target, tolerance=0.01):
    """
    :param joint_pos: Joints positions of your arm. List of np.array() points
    :param target: Target point Single np.array() point
    :param tolerance: Tolerance for the result
    :return: list of target joint positions
    """
    draw(joint_pos, target)

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
        print("showing initial position")
        draw(joint_pos, target)
        draw(joint_pos, target)
        time.sleep(1)
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

            draw(joint_pos, target)

            diffA = LA.norm(joint_pos[n-1]-target)
            print("diferencia target ", diffA)
            time.sleep(0.1)

fabrikWithDrawing(joint_pos_input, target)

