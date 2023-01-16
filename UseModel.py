from Envrionment import Envr
from stable_baselines3 import A2C
import pybullet as p
targetpos = (5, 0, 1)

env = Envr(True)
model = A2C.load("KrabbelModel")
obs = env.reset()

for i in range(10000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)