import gym as gym
import numpy as np
from gym import spaces
from gym.envs.registration import register
from Robit import Robit
import pybullet as p
import pybullet_data
from Envrionment import Envr
from stable_baselines3 import A2C
targetpos = (5, 0, 1)

env = Envr()
model = A2C.load("KrabbelModel")
obs = env.reset()

for i in range(10000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)