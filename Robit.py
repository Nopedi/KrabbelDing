import pybullet as p
import math
import time
from numpy import interp

maxForce = 500


def axisToQ(axisAngles):
    return [axisAngles[0] * math.sin(axisAngles[3] / 2 * math.pi / 180),
            axisAngles[1] * math.sin(axisAngles[3] / 2 * math.pi / 180),
            axisAngles[2] * math.sin(axisAngles[3] / 2 * math.pi / 180), math.cos(axisAngles[3] / 2 * math.pi / 180)]


def QToAxis(Quaternion):
    ang = 2 * math.acos(Quaternion[3])
    conv = 180 / math.pi
    try:
        return [(Quaternion[0] / math.sin(ang / 2)), (Quaternion[1] / math.sin(ang / 2)),
                (Quaternion[2] / math.sin(ang / 2)), ang * conv]
    except ZeroDivisionError:
        return [1, 0, 0, 0]


def rotPerAxis(Axis):
    return Axis[0] * Axis[3], Axis[1] * Axis[3], Axis[2] * Axis[3]


class Robit():
    def __init__(self, quaternion=None):
        if quaternion is None:
            quaternion = [0, 0, 0, 1]
        self.selfIdRobit = p.loadURDF("robit.urdf", [0, 0, 1.5], quaternion)
        p.changeDynamics(self.selfIdRobit, -1, lateralFriction=1)
        self.speed = 50
        self.maxVal = 1
        self.queternion = quaternion

    def start(self):
        self.hub0R(0)
        self.rot0R(0)
        self.hub1R(0)
        self.rot1R(0)
        self.hub2R(0)
        self.rot2R(0)

        self.hub0L(0)
        self.rot0L(0)
        self.hub1L(0)
        self.rot1L(0)
        self.hub2L(0)
        self.rot2L(0)

    def resetRobit(self):
        # self.start()
        p.resetBasePositionAndOrientation(self.selfIdRobit, [0, 0, 1.5], self.queternion)

    def getOrientation(self):
        return p.getBasePositionAndOrientation(self.selfIdRobit)

    def moveJoint(self, joint, angle):
        p.setJointMotorControl2(bodyUniqueId=self.selfIdRobit,
                                jointIndex=joint,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=angle * math.pi / 180,
                                maxVelocity=self.maxVal,
                                force=maxForce)

    def jointMover(self, angles):
        maxAngl = 50
        moveTimeOut = 0
        anglRad = interp(angles[:], (-1, 1), (-maxAngl, maxAngl))

        self.rot0R(angle=anglRad[0])
        self.hub0R(angle=anglRad[1])
        self.rot0L(angle=anglRad[2])
        self.hub0L(angle=anglRad[3])
        self.rot1R(angle=anglRad[4])
        self.hub1R(angle=anglRad[5])
        self.rot1L(angle=anglRad[6])
        self.hub1L(angle=anglRad[7])
        self.rot2R(angle=anglRad[8])
        self.hub2R(angle=anglRad[9])
        self.rot2L(angle=anglRad[10])
        self.hub2L(angle=anglRad[11])

        def isreached():
            if abs(p.getJointState(self.selfIdRobit, 0)[0] - anglRad[0]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[1] - anglRad[1]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[3] - anglRad[3]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[4] - anglRad[4]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[6] - anglRad[6]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[7] - anglRad[7]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[9] - anglRad[9]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[10] - anglRad[10]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[12] - anglRad[12]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[13] - anglRad[13]) > 0.1:
                return False
            if abs(p.getJointState(self.selfIdRobit, 0)[15] - anglRad[15]) > 0.1:
                return False    
            if abs(p.getJointState(self.selfIdRobit, 0)[16] - anglRad[16]) > 0.1:
                return False
            return True
        
        while not isreached() and moveTimeOut < 2000:
            moveTimeOut += 1
            p.stepSimulation()


    def moveRots(self, angle):
        self.moveJoint(0, angle=angle)
        self.moveJoint(3, angle=-angle)
        self.moveJoint(6, angle=angle)
        self.moveJoint(9, angle=-angle)
        self.moveJoint(12, angle=angle)
        self.moveJoint(15, angle=-angle)

    def moveArms(self, angle):
        self.moveJoint(1, angle=angle)
        self.moveJoint(4, angle=-angle)
        self.moveJoint(7, angle=angle)
        self.moveJoint(10, angle=-angle)
        self.moveJoint(13, angle=angle)
        self.moveJoint(16, angle=-angle)

    def moveLegs(self, angle):
        self.moveJoint(2, angle)
        self.moveJoint(5, -angle)
        self.moveJoint(8, angle)
        self.moveJoint(11, -angle)
        self.moveJoint(14, angle)
        self.moveJoint(17, -angle)

    def rot0R(self, angle):
        self.moveJoint(0, angle=angle)

    def rot0L(self, angle):
        self.moveJoint(3, angle=-angle)

    def rot1R(self, angle):
        self.moveJoint(6, angle=angle)

    def rot1L(self, angle):
        self.moveJoint(9, angle=-angle)

    def rot2R(self, angle):
        self.moveJoint(12, angle=angle)

    def rot2L(self, angle):
        self.moveJoint(15, angle=-angle)

    def hub0R(self, angle):
        self.moveJoint(1, angle=-angle)
        self.moveJoint(2, angle=180 - 90 + angle)

    def hub0L(self, angle):
        self.moveJoint(4, angle=angle)
        self.moveJoint(5, angle=-180 + 90 - angle)

    def hub1R(self, angle):
        self.moveJoint(7, angle=-angle)
        self.moveJoint(8, angle=180 - 90 + angle)

    def hub1L(self, angle):
        self.moveJoint(10, angle=angle)
        self.moveJoint(11, angle=-180 + 90 - angle)

    def hub2R(self, angle):
        self.moveJoint(13, angle=-angle)
        self.moveJoint(14, angle=180 - 90 + angle)

    def hub2L(self, angle):
        self.moveJoint(16, angle=angle)
        self.moveJoint(17, angle=-180 + 90 - angle)

    def wait(self, delaytime):
        startTime = time.time()
        while time.time() < startTime + delaytime:
            pass

    def step0R(self):
        self.hub0R(60)
        self.wait(self.speed)
        self.rot0R(-30)
        self.wait(self.speed)
        self.hub0R(-20)
        self.wait(self.speed)
        self.rot0R(30)
        self.wait(self.speed)
        self.hub0R(60)
        self.wait(self.speed)

    def step0L(self):
        self.hub0L(60)
        time.sleep(self.speed)
        self.rot0L(-30)
        time.sleep(self.speed)
        self.hub0L(-20)
        time.sleep(self.speed)
        self.rot0L(30)
        time.sleep(self.speed)
        self.hub0L(60)
        time.sleep(self.speed)

    def step1R(self):
        self.hub1R(60)
        time.sleep(self.speed)
        self.rot1R(-30)
        time.sleep(self.speed)
        self.hub1R(-20)
        time.sleep(self.speed)
        self.rot1R(30)
        time.sleep(self.speed)
        self.hub1R(60)
        time.sleep(self.speed)

    def step1L(self):
        self.hub1L(60)
        time.sleep(self.speed)
        self.rot1L(-30)
        time.sleep(self.speed)
        self.hub1L(-20)
        time.sleep(self.speed)
        self.rot1L(30)
        time.sleep(self.speed)
        self.hub1L(60)
        time.sleep(self.speed)

    def step2R(self):
        self.hub2R(60)
        time.sleep(self.speed)
        self.rot2R(-30)
        time.sleep(self.speed)
        self.hub2R(-20)
        time.sleep(self.speed)
        self.rot2R(30)
        time.sleep(self.speed)
        self.hub2R(60)
        time.sleep(self.speed)

    def step2L(self):
        self.hub2L(60)
        time.sleep(self.speed)
        self.rot2L(-30)
        time.sleep(self.speed)
        self.hub2L(-20)
        time.sleep(self.speed)
        self.rot2L(30)
        time.sleep(self.speed)
        self.hub2L(60)
        time.sleep(self.speed)

    def rotLRL(self, angle):
        self.rot0L(angle)
        self.rot1R(angle)
        self.rot2L(angle)

    def hubLRL(self, angle):
        self.hub0L(angle)
        self.hub1R(angle)
        self.hub2L(angle)

    def rotRLR(self, angle):
        self.rot0R(angle)
        self.rot1L(angle)
        self.rot2R(angle)

    def hubRLR(self, angle):
        self.hub0R(angle)
        self.hub1L(angle)
        self.hub2R(angle)

    def plugWalk(self):

        self.wait(self.speed)

        self.hubRLR(30)
        self.hubLRL(0)

        self.wait(self.speed)

        self.rotRLR(-30)
        self.rotLRL(30)

        self.wait(self.speed)

        self.hubRLR(0)
        self.hubLRL(30)

        self.wait(self.speed)

        self.rotRLR(30)
        self.rotLRL(-30)

        self.wait(self.speed)

    def getRotationXYZ(self):
        pos, rot = self.getOrientation()
        return rotPerAxis(QToAxis(rot))

    def getPosition(self):
        return self.getOrientation()[0]



