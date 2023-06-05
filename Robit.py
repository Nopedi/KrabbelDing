import pybullet as p
import math
import time
import numpy as np
from numpy import interp

maxForce = 10000


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
        self.selfIdRobit = p.loadURDF("Robit.urdf", [0, 0, 1.5], quaternion)
        p.changeDynamics(self.selfIdRobit, -1, lateralFriction=10)
        self.speed = 50
        self.maxVal = .3
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
                                targetPosition=angle,
                                maxVelocity=self.maxVal,
                                force=maxForce)

    def jointMoverTwo(self, angles):
        maxAngl = 30 * math.pi/180  # 30 Grad in radians = 0.523599 rad
        moveTimeOut = 0
        anglRad = interp(angles[:], (-1, 1), (-maxAngl, maxAngl))
        
    def stepsim(self, repetitions):
        i = 0
        while i < repetitions:
            p.stepSimulation()
            i += 1 
    
    def jointMover(self, angle_scale):
        maxAngl = 30 * math.pi/180  # 30 Grad in radians = 0.523599 rad
        moveTimeOut = 400
        # anglRad = interp(angles[:], (-1, 1), (-maxAngl, maxAngl))
        scaled_angles = maxAngl * angle_scale
        # print(anglRad)

        # alle beine auf pos 1 setzen
        
        self.hub0R(angle=scaled_angles[1])
        self.hub0L(angle=scaled_angles[3])
        self.hub1R(angle=scaled_angles[5])
        self.hub1L(angle=scaled_angles[7])
        self.hub2R(angle=scaled_angles[9])
        self.hub2L(angle=scaled_angles[11])
        
        self.stepsim(moveTimeOut)
        #wait for all joints to lift
        self.rot0R(angle=scaled_angles[0])
        self.rot0L(angle=scaled_angles[2])
        self.rot1R(angle=scaled_angles[4])
        self.rot1L(angle=scaled_angles[6])
        self.rot2R(angle=scaled_angles[8])
        self.rot2L(angle=scaled_angles[10])
        
        self.stepsim(moveTimeOut)
        #wait for all joints to rotate forwards
        self.hub0R(angle=-scaled_angles[1])
        self.hub0L(angle=-scaled_angles[3])
        self.hub1R(angle=-scaled_angles[5])
        self.hub1L(angle=-scaled_angles[7])
        self.hub2R(angle=-scaled_angles[9])
        self.hub2L(angle=-scaled_angles[11])
        
        self.stepsim(moveTimeOut)
        #wait for all joints to lower
        self.rot0R(angle=-scaled_angles[0])
        self.rot0L(angle=-scaled_angles[2])
        self.rot1R(angle=-scaled_angles[4])
        self.rot1L(angle=-scaled_angles[6])
        self.rot2R(angle=-scaled_angles[8])
        self.rot2L(angle=-scaled_angles[10])
        
        self.stepsim(moveTimeOut)
        #wait for all joints to rotate backwards
        
            
    def get_joint_angles(self):
        joint_angels = (p.getJointState(self.selfIdRobit, 0)[0],
                        p.getJointState(self.selfIdRobit, 1)[0],
                        p.getJointState(self.selfIdRobit, 3)[0],
                        p.getJointState(self.selfIdRobit, 4)[0],
                        p.getJointState(self.selfIdRobit, 6)[0],
                        p.getJointState(self.selfIdRobit, 7)[0],
                        p.getJointState(self.selfIdRobit, 9)[0],
                        p.getJointState(self.selfIdRobit, 10)[0],
                        p.getJointState(self.selfIdRobit, 12)[0],
                        p.getJointState(self.selfIdRobit, 13)[0],
                        p.getJointState(self.selfIdRobit, 15)[0],
                        p.getJointState(self.selfIdRobit, 16)[0]) 
        return joint_angels


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
        self.moveJoint(2, angle=angle)

    def hub0L(self, angle):
        self.moveJoint(4, angle=angle)
        self.moveJoint(5, angle=-angle)

    def hub1R(self, angle):
        self.moveJoint(7, angle=-angle)
        self.moveJoint(8, angle=angle)

    def hub1L(self, angle):
        self.moveJoint(10, angle=angle)
        self.moveJoint(11, angle=-angle)

    def hub2R(self, angle):
        self.moveJoint(13, angle=-angle)
        self.moveJoint(14, angle=angle)

    def hub2L(self, angle):
        self.moveJoint(16, angle=angle)
        self.moveJoint(17, angle=-angle)

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
    
    def getVel(self):
        return p.getBaseVelocity(self.selfIdRobit)



