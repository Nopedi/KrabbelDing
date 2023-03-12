# from stable_baselines3.common.env_checker import check_env
# import torch as th
from RobitEnv2 import RobitEnvironment
from stable_baselines3 import SAC as alg
from stable_baselines3.common.env_util import make_vec_env
import os

# env = make_vec_env(RobitEnvironment, n_envs=1)
MODEL_NAME = "KrabbelTest005"
LEARING_TIMESTEPS = 100_000
env = RobitEnvironment(gui=False)

logdir = "logs"
if not os.path.exists(logdir):
    os.makedirs(logdir)
    
model = alg("MultiInputPolicy",
            env,
            # learning_rate=0.0015,
            ent_coef="auto_0.1",
            train_freq=(1, 'episode'),
            policy_kwargs = dict(net_arch = [400, 300, 200]),        
            learning_starts=2000,
            verbose=2,
            tensorboard_log=logdir)

model.learn(total_timesteps=LEARING_TIMESTEPS, progress_bar=True)
model.save(MODEL_NAME)
