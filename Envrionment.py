import gym as gym
import numpy as np
from gym import spaces
from gym.envs.registration import register
from Robit import Robit
import pybullet as p
import pybullet_data


# p.setRealTimeSimulation(1)
targetpos = np.array((5, 0, 1))
MAX_EP_LEN = 50

def getDist(array):
    return np.sqrt(sum(array[:]**2))


class Envr(gym.Env):
    metadata = {'render.modes': ['human']}
    with open('log.csv', 'w') as f:
        f.write(f"Got Closer ; Reached dist ; success ; failed\n")

    def __init__(self, render=False):
        super().__init__()
        self.rendering(render)
        self.target = np.array((5, 0, 1))
        self.action_space = spaces.Box(low=np.full(12, -1), high=np.full(12, 1), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.full(6, -180), high=np.full(6, 180),
                                             dtype=np.float64)
        # TODO: Normalise the observation
        self.r = Robit([0, 0, 0, 1])  # initialise the Robot
        self.distOld = 5
        self.reachedDist = 0
        self.success = 0
        self.failed = 0
        self.gotCloser = 0
        self.resetString = ""
        self.timer = 0

    def rendering(self, gui):
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        gorundPlane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        target = p.loadURDF("target.urdf", targetpos, [0, 0, 0, 1])
        p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 1])
        p.changeDynamics(gorundPlane, -1, lateralFriction=1)
        p.setGravity(0, 0, -10)

    def step(self, action):
        print(action)
        self.timer += 1
        self.r.jointMover(action)
        dist = getDist(self.target - self.r.getPosition())
        pos = self.r.getPosition()
        rot = self.r.getRotationXYZ()
        input = (rot[0], rot[1], rot[2], pos[0]-targetpos[0], pos[1]-targetpos[1], pos[2]-targetpos[2])
        observation = np.array(input).astype(np.float64)
        rwd = 0
        done = False
        self.resetString = "Timeout"
        rwd = 1/(dist)
        if pos[0] > self.target[0]:
            rwd = 5
            self.resetString = "Further than Target"
            #print(self.resetString)
            done = True
        if dist < 1:
            self.success += 1
            rwd = 10
            done = True
            self.resetString = "Goal Reached"
            print(self.resetString)
        if -80 < observation[0] > 80 or -80 < observation[1] > 80 or pos[0] > 7:
            rwd = 0
            done = True
            self.resetString = "Crashed"
            #print(self.resetString)
        info = {'rwd': rwd, 'Timer' : self.timer}
        if self.timer >= MAX_EP_LEN:
            rwd = 0
            done = True
        print(info, end="\r")
        
        return observation, rwd, done, info

    def reset(self, *, seed = None, options = None):
        self.log()
        print(f"\nReset: {self.resetString}")
        self.timer = 0
        self.distOld = 5
        self.reachedDist = 0
        self.success = 0
        self.failed = 0
        self.gotCloser = 0
        p.removeBody(self.r.selfIdRobit)
        self.r = Robit([0, 0, 0, 1])
        self.r.resetRobit()
        pos = self.r.getPosition()
        rot = self.r.getRotationXYZ()
        input = (rot[0], rot[1], rot[2], pos[0]-targetpos[0], pos[1]-targetpos[1], pos[2]-targetpos[2])
        observation = np.array(input)
        return observation

    def close(self):
        p.disconnect()

    def log(self):
        with open('log.csv', 'a', newline='') as f:
            f.write(f"{self.gotCloser}; {self.reachedDist}; {self.success}; {self.failed}\n")


if __name__ == "__main__":
    env = Envr(True)
    while 1:
        obs, rwd, done, indo = env.step(env.action_space.sample())
        if done:
            env.reset()
