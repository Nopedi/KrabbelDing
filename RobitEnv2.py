import gym as gym
import numpy as np
from gym import spaces
from gym.envs.registration import register
from stable_baselines3.common.env_checker import check_env
from Robit import Robit
import pybullet as p
import pybullet_data

TIMEOUT = 200

class RobitEnvironment(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, gui=False):
        super().__init__()
        self.use_gui = gui
        self.action_space = spaces.Box(low=np.full(12, -1), high=np.full(12, 1), dtype=np.float32)
        self.observation_space = spaces.Dict({"rotation": spaces.Box(low=np.full(3, -180), high=np.full(3, 180),
                                             dtype=np.float64),
                                             "delta_position": spaces.Box(low=np.full(3, -10), high=np.full(3, 10),
                                             dtype=np.float64),
                                             "joint_positions": spaces.Box(low=np.full(12, -40), high=np.full(12, 40),
                                             dtype=np.float64),
                                             "linvelocity": spaces.Box(low=np.array((-10,-10,-10)), high=np.array((10,10,10)),
                                                                       dtype=np.float64),
                                             "angvelocity": spaces.Box(low=np.array((-10,-10,-10)), high=np.array((10,10,10)),
                                                                       dtype=np.float64),
                                             "dist" : spaces.Box(low=0, high=10, shape=(1,), dtype=np.float64)
                                             })
        p.connect(p.GUI) if self.use_gui else p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0, 0, 1])
        self._setup(gui=self.use_gui)
        self.timer = 0
    
    def _setup(self, gui):
        gorundPlane = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        self.robit = Robit()  # initialise the Robot
        p.changeDynamics(gorundPlane, -1, lateralFriction=1)
        p.setGravity(0, 0, -10)
    
    def _get_new_rdm_target_pos(self):
        self.DISTANCE = np.random.randint(8, 15)
        angl = np.random.randint(-60, 60)
        self.TARGET_POSITION = np.array((np.cos(angl*np.pi/180) * self.DISTANCE, np.sin(angl*np.pi/180) * self.DISTANCE, 1))
    
    def update_target_position(self, dist, angle):
        pos = self.robit.getPosition()
        rot = self.robit.getRotationXYZ()[2]
        
        tp = pos + np.array((np.cos((rot+angle)*np.pi/180) * dist, np.sin((rot+angle)*np.pi/180) * dist, 0))
        self.TARGET_POSITION = np.array(tp)
        
        #try:
        p.removeAllUserDebugItems()
        #except AttributeError:
        #    pass
    
        self.line = p.addUserDebugLine(pos, self.TARGET_POSITION, (0,0,1))
        
    def _get_dist(self):
        return np.sqrt(sum((self.TARGET_POSITION - self.robit.getPosition())[:]**2))
    
    def reset(self):
        # try:
        p.resetSimulation()
        self._get_new_rdm_target_pos()
        # self.TARGET_POSITION = np.array((0,0,0.5))
        self._setup(self.use_gui)
        self.robit.resetRobit()
        self.timer = 0
        return self._get_obs()
    
    def step(self, action):
        # print(action)
        done = False
        fall = False
        goal_reached = False
        timeout = False
        self.timer += 1
        self.robit.jointMover(action)
        dist = self._get_dist()
        rwd = (1/(dist)-1/self.DISTANCE) # *0.99**self.timer
        # if dist != 0:
        #     rwd = -dist * .5
        # else:
        #     rwd = 0
        rotation = self.robit.getRotationXYZ()
        if abs(rotation[0]) > 60 or abs(rotation[1]) > 60:
            rwd = -50
            fall = True
            done = True
        if dist < 1.5:
            rwd = 100
            # print("goal reached")
            goal_reached = True
            done = True
        if self.timer >= TIMEOUT:
            done = True
            timeout = True
 
        # print(rwd)
        info = {"time": self.timer, "done":done, "timeout": timeout , "fall": fall, "goal_reached": goal_reached, "dist": dist, "rwd": rwd}
        return self._get_obs(), rwd, done, info
    
    def close(self):
        p.disconnect()
        return super().close()
    
    def _get_obs(self):
        rotation = np.array(self.robit.getRotationXYZ())
        delta_position = self.TARGET_POSITION - np.array(self.robit.getPosition())
        # target_position = self.TARGET_POSITION
        joint_positions = np.array(self.robit.get_joint_angles())
        linVel, angVel = self.robit.getVel()
        obs = {"rotation": rotation,
               "delta_position": delta_position,
               #"target_position": target_position,
               "joint_positions": joint_positions,
               "linvelocity": np.array(linVel),
               "angvelocity": np.array(angVel),
               "dist" : np.array([self._get_dist()], dtype=np.float64)
               }
        # print(obs)
        return obs
    

if __name__ == "__main__":
    env = RobitEnvironment(True)
    # print(env.observation_space.sample())
    # check_env(env)
    env.reset()
    step1 = np.array((1,-1,-1,1,-1,1,1,-1,1,-1,-1,1))
    step1 = step1 * 0.5    
    # step2 = np.array((-1,-1,1,1,1,1,-1,-1,-1,-1,1,1))
    # step3 = step1 * -1
    # step4 = step2 * -1
    
    for _ in range(100):    
        obs, rwd, done, info = env.step(step1)
        print(info)
        # obs, rwd, done, info = env.step(step2)
        # print(info)
        # obs, rwd, done, info = env.step(step3)
        # print(info)
        # obs, rwd, done, info = env.step(step4)
        # print(info)

    # while 1:
    #     obs, rwd, done, indo = env.step(env.action_space.sample())
    #     if done:
    #         env.reset()
    