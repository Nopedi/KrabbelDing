from RobitEnv2 import RobitEnvironment
from stable_baselines3 import SAC as alg
from stable_baselines3.common.env_util import make_vec_env
import pygame
import numpy as np

pygame.init()
pygame.joystick.init()

joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
joystick = joysticks[0]

env = RobitEnvironment(True)

# MODEL_NAME = "opt_models/SAC/SAC_Model_1.5764523333333333"
MODEL_NAME = "KrabbelTest015"
# MODEL_NAME = "logs/best_model.zip"
model = alg.load(MODEL_NAME)
obs = env.reset()
env.TARGET_POSITION = np.array((0,0,0.5))
i = 0
while 1:
    pygame.event.get()
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, done, info = env.step(action)
    i += rewards
    dist = abs(round(joystick.get_axis(1),2)*7)
    angle = round(joystick.get_axis(0),2)*-30
    if joystick.get_button(0):
        env.reset()
    if joystick.get_button(3):
        env.close()
    # print(dist,angle)
    env.update_target_position(dist, angle)
    #print(env.TARGET_POSITION)

print(i)
