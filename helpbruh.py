from RobitEnv2 import RobitEnvironment
from stable_baselines3 import SAC as alg
from stable_baselines3.common.env_util import make_vec_env
 
env = RobitEnvironment(True)

MODEL_NAME = "opt_models/SAC/SAC_Model_70.14048633333334"
model = alg.load(MODEL_NAME)
obs = env.reset()
i = 0
while 1:
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, done, info = env.step(action)
    i += rewards
    if done:
        env.close()
        break

print(i)
