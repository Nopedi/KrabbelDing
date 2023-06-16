import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

offset = 1
l_arm2 = 10
alpha_deg1 = 6

alpha_deg0 = 60
l_arm_unten = 8.5
l_leg = 4

base_a = np.array((0, 0, 0))
base_b = np.array((0, 5, 0))

def func(offset_base, larm, angle_bound):
    ALPHA = np.linspace(-alpha_deg0*np.pi/180, alpha_deg0*np.pi/180, 21)
    BETA = np.linspace(-angle_bound*np.pi/180, angle_bound*np.pi/180, 21)
    scnd_arm_start_pos = np.array((offset_base, 2.5, 3.5))
    def _joints_from_angle(angles_alpha, angles_beta, l_arm):
        joint_a = []
        joint_b = []
        joint_c = []
        for angle in angles_alpha:
            joint_a.append((base_a[0] + np.cos(angle) * l_arm_unten,
                           base_a[1] + np.sin(angle) * l_arm_unten, base_a[2]))
            joint_b.append((base_b[0] + np.cos(angle) * l_arm_unten,
                           base_b[1] + np.sin(angle) * l_arm_unten, base_b[2]))
        for angle in angles_beta:
            joint_c.append((scnd_arm_start_pos[0] + np.cos(angle) * l_arm,
                           scnd_arm_start_pos[1] + np.sin(angle) * l_arm, scnd_arm_start_pos[2]))
        return np.array(joint_a), np.array(joint_b), np.array(joint_c)
    a, b, c = _joints_from_angle(ALPHA, BETA, larm)
    rot_point = (a + b) / 2
    v = rot_point - c
    foot_end_point = c + l_leg * v
    grad = np.gradient(foot_end_point[:, 1])
    return foot_end_point, c, rot_point



if __name__ == "__main__":
    fig = plt.figure()

    ax1 = fig.add_subplot(projection='3d')

    # out = []
    # off = 6
    # for _ in range(100):
    #     off += 0.01
    #     out.append(func(2, off, 85))


    # foot_end_point, c, rot_point = func(1.303, 5.999, 87)
    foot_end_point, c, rot_point = func(-5, 10, 30)


    print(np.std(foot_end_point[:, 0]))

    ax1.plot( foot_end_point[:, 0], foot_end_point[:, 1], foot_end_point[:, 2], zdir='zPos', label='curve in (x, y)')

    ax1.plot( c[:, 0], c[:, 1], c[:, 2], zdir='zPos', label='curve in (x, y)')
    ax1.plot( rot_point[:, 0], rot_point[:, 1], rot_point[:, 2], zdir='zPos', label='curve in (x, y)')

    # # ax1.quiver(c[:, 0], c[:, 1], c[:, 2], v[:, 0], v[:, 1], v[:, 2], length=4)
    ax1.set_xlim3d(-15, 15)
    ax1.set_ylim3d(-15, 15)
    ax1.set_zlim3d(-10, 10)

    ax1.set(xlabel='xPos', ylabel='yPos', zlabel='zPos',
           title='ALPHA - Position')

    ax1.grid()
    #
    # (ax1, ax2) = fig.subplots(2)
    # ax1.plot( foot_end_point[:, 0], label='derivative of foot position')
    # ax2.plot( grad, label='derivative of foot position')
    plt.show()