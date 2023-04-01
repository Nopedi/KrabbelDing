# from stable_baselines3.common.env_checker import check_env
# import torch as th
from RobitEnv2 import RobitEnvironment
from stable_baselines3 import SAC as alg
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnNoModelImprovement
from stable_baselines3.common.noise import NormalActionNoise
import os
import numpy as np

env = RobitEnvironment(gui=True)

stop_train_callback = StopTrainingOnNoModelImprovement(max_no_improvement_evals=30, min_evals=5, verbose=1)
eval_callback = EvalCallback(env, eval_freq=5000, callback_after_eval=stop_train_callback, verbose=1)

MODEL_NAME = "KrabbelTest006"
LEARING_TIMESTEPS = 20_000

logdir = "logs"
if not os.path.exists(logdir):
    os.makedirs(logdir)

n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma= .1 * np.ones(n_actions))
model = alg("MultiInputPolicy",
            env,
            verbose=1,
            tensorboard_log=logdir)

model.learn(total_timesteps=LEARING_TIMESTEPS, callback=eval_callback, progress_bar=True)
model.save(MODEL_NAME)
