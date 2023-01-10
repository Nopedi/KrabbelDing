
import gym as gym
import numpy as np
from gym import spaces
from gym.envs.registration import register
from Robit import Robit
import pybullet as p
import pybullet_data

from stable_baselines3 import A2C
from stable_baselines3.common.env_util import make_vec_env

from stable_baselines3.common.env_checker import check_env


# p.setRealTimeSimulation(1)
targetpos = np.array((5, 0, 1))


def getDist(array):
    return np.sqrt(sum(array[:]**2))


class Envr(gym.Env):
    metadata = {'render.modes': ['human']}
    with open('log.csv', 'w') as f:
        f.write(f"Got Closer ; Reached dist ; success ; failed\n")

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    gorundPlane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
    target = p.loadURDF("target.urdf", targetpos, [0, 0, 0, 1])
    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 1])
    p.changeDynamics(gorundPlane, -1, lateralFriction=2)
    p.setGravity(0, 0, -10)

    def __init__(self):
        super().__init__()
        self.target = np.array((5, 0, 1))
        self.action_space = spaces.Box(low=np.full(18, -1), high=np.full(18, 1), dtype=np.float32)
        self.observation_space = spaces.Box(low=np.full(6, -180), high=np.full(6, 180),
                                             dtype=np.float64)
        # TODO: Normalise the observation
        self.r = Robit([0, 0, 0, 1])  # initialise the Robot
        self.distOld = 5
        self.reachedDist = 0
        self.success = 0
        self.failed = 0
        self.gotCloser = 0

    def step(self, action):
        p.stepSimulation()
        self.r.jointMover(action)
        dist = getDist(self.target - self.r.getPosition())
        pos = self.r.getPosition()
        rot = self.r.getRotationXYZ()
        input = (rot[0], rot[1], rot[2], pos[0]-targetpos[0], pos[1]-targetpos[1], pos[2]-targetpos[2])
        observation = np.array(input).astype(np.float64)
        rwd = -1
        done = False
        if pos[0] > self.target[0]:
            self.reachedDist += 1
            rwd = 100
            done = True
        if dist < self.distOld:
            self.distOld = dist
            self.gotCloser += 1
            rwd = 1
            if dist < 2:
                self.success += 1
                rwd = 1000
                done = True
        else:
            rwd = -50
        if -30 < observation[0] > 30 or -30 < observation[1] > 30 or -30 < observation[2] > 30:
            self.failed += 1
            rwd -= 100
        if -60 < observation[0] > 60 or -60 < observation[1] > 60 or pos[0] > 6 or pos[0] > 6:
            rwd = -1000
            done = True
        info = {'info': 'wer das hier liest ist doof'}
        return observation, rwd, done, info

    def reset(self, *, seed: [int] = None, options: [dict] = None):
        self.log()
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
    env = Envr()
    check_env(env)

    model = A2C("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=100)
    model.save("KrabbelModel")
    del model
    p.disconnect()


    p.connect(p.GUI)
    gorundPlane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
    p.loadURDF("target.urdf", targetpos, [0, 0, 0, 1])
    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 1])
    p.changeDynamics(gorundPlane, -1, lateralFriction=2)
    p.setGravity(0, 0, -10)

    env = Envr()
    model = A2C.load("KrabbelModel")
    obs = env.reset()

    qKey = ord('q')
    mKey = ord('m')

    for i in range(1000):
        action, _state = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)



