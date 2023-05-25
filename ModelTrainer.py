# from stable_baselines3.common.env_checker import check_env
# import torch as th
from RobitEnv2 import RobitEnvironment
from stable_baselines3 import SAC as alg
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnNoModelImprovement
from stable_baselines3.common.noise import NormalActionNoise
import os
import torch as th
import numpy as np

# env = RobitEnvironment(gui=True)

env = make_vec_env(RobitEnvironment, n_envs=4)

stop_train_callback = StopTrainingOnNoModelImprovement(max_no_improvement_evals=5, min_evals=5, verbose=1)
eval_callback = EvalCallback(env, eval_freq=5000, callback_after_eval=stop_train_callback, verbose=1)

MODEL_NAME = "KrabbelTest007"
LEARING_TIMESTEPS = 1_000_000
USE_OLD_MODEL = False
logdir = "logs"
if not os.path.exists(logdir):
    os.makedirs(logdir)

if not USE_OLD_MODEL:
    model = alg("MultiInputPolicy",
                        env,
                        # seed=10,
                        gamma=1-0.000159,
                        learning_rate=0.005,
                        # ent_coef=0.0055,
                        policy_kwargs=dict(
                            activation_fn=th.nn.ReLU,
                            net_arch=[128, 128]),
                        # train_freq=(1, "step"),
                        verbose=1,
                        device="auto",
                        tensorboard_log=logdir)
else:
    print(f"Using old Model {MODEL_NAME}")
    model = alg.load(f"{MODEL_NAME}")
    model.set_env(env)

model.learn(total_timesteps=LEARING_TIMESTEPS, callback=eval_callback, progress_bar=True)
model.save(MODEL_NAME)
