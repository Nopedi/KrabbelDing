from Envrionment import Envr
from stable_baselines3 import TD3 as alg
from stable_baselines3.common.env_util import make_vec_env

env = Envr(True)
MODEL_NAME = "KrabbelDest-v0"
model = alg.load(MODEL_NAME)
obs = env.reset()
i = 0
while 1:
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, done, info = env.step(action)
    i += 1