# from stable_baselines3.common.env_checker import check_env
# import torch as th
from RobitEnv2 import RobitEnvironment
from stable_baselines3 import TD3 as alg
from stable_baselines3.common.env_util import make_vec_env

# env = make_vec_env(RobitEnvironment, n_envs=1)
MODEL_NAME = "KrabbelDest-v1"
env = RobitEnvironment()
    
model = alg("MultiInputPolicy", env,
            # learning_rate=0.007,
            verbose=2)
model.learn(total_timesteps=10000, progress_bar=True)
model.save(MODEL_NAME)
