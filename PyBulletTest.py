# import numpy as np
# import math
import math

import numpy as np
import stable_baselines3
import pybullet as p
import pybullet_data
from Robit import Robit

p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setRealTimeSimulation(1)

gorundPlane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
target = p.loadURDF("target.urdf", (5, 0, 1), [0, 0, 0, 1])
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,1])
p.changeDynamics(target, -1, lateralFriction=2)
p.changeDynamics(gorundPlane, -1, lateralFriction=2)
R0ang = p.addUserDebugParameter("0RHub", -45, 90)
R0rot = p.addUserDebugParameter("0RRot", -90, 90)
L0ang = p.addUserDebugParameter("0LHub", -45, 90)
L0rot = p.addUserDebugParameter("0LRot", -90, 90)

R1ang = p.addUserDebugParameter("1RHub", -45, 90)
R1rot = p.addUserDebugParameter("1RRot", -90, 90)
L1ang = p.addUserDebugParameter("1LHub", -45, 90)
L1rot = p.addUserDebugParameter("1LRot", -90, 90)

R2ang = p.addUserDebugParameter("2RHub", -45, 90)
R2rot = p.addUserDebugParameter("2RRot", -90, 90)
L2ang = p.addUserDebugParameter("2LHub", -45, 90)
L2rot = p.addUserDebugParameter("2LRot", -90, 90)


def axisToQ(axisAngles):
    return [axisAngles[0] * math.sin(axisAngles[3]/2*math.pi/180),axisAngles[1] * math.sin(axisAngles[3]/2*math.pi/180),
        axisAngles[2] * math.sin(axisAngles[3]/2*math.pi/180),math.cos(axisAngles[3]/2*math.pi/180)]


def QToAxis(Quaternion):
    ang = 2*math.acos(Quaternion[3])
    conv = 180/math.pi
    try:
        return [(Quaternion[0]/math.sin(ang/2)), (Quaternion[1]/math.sin(ang/2)), (Quaternion[2]/math.sin(ang/2)), ang * conv]
    except ZeroDivisionError:
        return [1, 0, 0, 0]


def rotPerAxis(Axis):
    return Axis[0]*Axis[3], Axis[1]*Axis[3], Axis[2]*Axis[3]


r = Robit(quaternion=axisToQ([0, 0, 0, 1]))
i = 0
direction = 0
qKey = ord('q')
mKey = ord('m')


def updateJoints():
    valR0Hub = p.readUserDebugParameter(R0ang)
    valR0Rot = p.readUserDebugParameter(R0rot)
    r.hub0R(valR0Hub)
    r.rot0R(valR0Rot)
    valL0Hub = p.readUserDebugParameter(L0ang)
    valL0Rot = p.readUserDebugParameter(L0rot)
    r.hub0L(valL0Hub)
    r.rot0L(valL0Rot)
    valR1Hub = p.readUserDebugParameter(R1ang)
    valR1Rot = p.readUserDebugParameter(R1rot)
    r.hub1R(valR1Hub)
    r.rot1R(valR1Rot)
    valL1Hub = p.readUserDebugParameter(L1ang)
    valL1Rot = p.readUserDebugParameter(L1rot)
    r.hub1L(valL1Hub)
    r.rot1L(valL1Rot)
    valR2Hub = p.readUserDebugParameter(R2ang)
    valR2Rot = p.readUserDebugParameter(R2rot)
    r.hub2R(valR2Hub)
    r.rot2R(valR2Rot)
    valL2Hub = p.readUserDebugParameter(L2ang)
    valL2Rot = p.readUserDebugParameter(L2rot)
    r.hub2L(valL2Hub)
    r.rot2L(valL2Rot)


# updateJoints()
# r.wait(2)
# p.setGravity(0, 0, -10)

while p.isConnected():
    keys = p.getKeyboardEvents()
    # updateJoints()
    pos, rot = r.getOrientation()
    r.jointMover(np.full(12, 1))
    # p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=pos)
    # r.plugWalk()

    # r.step0R()
    # r.step1L()
    # r.step2R()
    #
    # r.step0L()
    # r.step1R()
    # r.step2L()
    a = rotPerAxis(QToAxis(rot))
    print(round(a[0], 1), round(a[1], 1), round(a[2], 1), end="\n")

    if qKey in keys:
        print(f"Sim stopped by 'q'({qKey}) press")
        p.disconnect()
        break
    if mKey in keys:
        print(f"mkeey")
        r.jointMover(np.full(18, -1))
        break

