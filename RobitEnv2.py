import gym as gym
import numpy as np
from gym import spaces
from gym.envs.registration import register
from stable_baselines3.common.env_checker import check_env
from Robit import Robit
import pybullet as p
import pybullet_data

TARGET_POSITION = np.array((5, 0, 0))

class RobitEnvironment(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self):
        super().__init__()
        self.action_space = spaces.Box(low=np.full(12, -1), high=np.full(12, 1), dtype=np.float32)
        self.observation_space = spaces.Dict({"rotation": spaces.Box(low=np.full(3, -180), high=np.full(3, 180),
                                             dtype=np.float64),
                                             "self_position": spaces.Box(low=np.full(3, -10), high=np.full(3, 10),
                                             dtype=np.float64),
                                             "target_position": spaces.Box(low=np.full(3, -10), high=np.full(3, 10),
                                             dtype=np.float64),
                                             "joint_positions": spaces.Box(low=np.full(12, -40), high=np.full(12, 40),
                                             dtype=np.float64)
                                             })
        
        self._setup(gui=True)
        self.robit = Robit([0, 0, 0, 1])  # initialise the Robot
        self.timer = 0
    
    def _setup(self, gui):
        p.connect(p.GUI) if gui else p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        target = p.loadURDF("target.urdf", TARGET_POSITION, [0, 0, 0, 1])
        gorundPlane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 1])
        p.changeDynamics(gorundPlane, -1, lateralFriction=1)
        p.setGravity(0, 0, -10)
    
    def reset(self):
        self.robit.resetRobit()
        self.timer = 0
        return self._get_obs()
    
    def step(self, action):
        done = False
        self.timer += 1
        self.robit.jointMover(action)
        dist = np.sqrt(sum((TARGET_POSITION - self.robit.getPosition())[:]**2))
        rwd = 1/dist
        if dist < 2:
            rwd = 2
            done = True
        rotation = self.robit.getRotationXYZ()
        if abs(rotation[0]) > 60 or abs(rotation[1]) > 60:
            rwd = 0
            done = True
        if self.timer > 50:
            done = True
        # print(dist, end="\r")
        return self._get_obs(), rwd, done, {}
    
    def close(self):
        p.disconnect()
        return super().close()
    
    def _get_obs(self):
        rotation = np.array(self.robit.getRotationXYZ())
        self_position = np.array(self.robit.getPosition())
        target_position = TARGET_POSITION
        joint_positions = np.array(self.robit.get_joint_angles())
        obs = {"rotation": rotation,
               "self_position": self_position,
               "target_position": target_position,
               "joint_positions": joint_positions}
        return obs
    

if __name__ == "__main__":
    env = RobitEnvironment()
    check_env(env)
    while 1:
        obs, rwd, done, indo = env.step(env.action_space.sample())
        if done:
            env.reset()
    