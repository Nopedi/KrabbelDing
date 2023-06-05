# from stable_baselines3.common.env_checker import check_env
# import torch as th
from RobitEnv2 import RobitEnvironment
from stable_baselines3 import SAC as alg
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnNoModelImprovement, StopTrainingOnRewardThreshold
from stable_baselines3.common.noise import NormalActionNoise
import os
import torch as th
import numpy as np

# env = RobitEnvironment(gui=True)

env = make_vec_env(RobitEnvironment, n_envs=1)

# stop_train_callback = StopTrainingOnNoModelImprovement(max_no_improvement_evals=5, min_evals=10, verbose=1)
# eval_callback = EvalCallback(env, eval_freq=10000, callback_after_eval=stop_train_callback, verbose=1)
# callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=103, verbose=1)
# eval_callback = EvalCallback(env, callback_on_new_best=callback_on_best, verbose=1)
eval_callback = EvalCallback(env, best_model_save_path="./logs/",
                             log_path="./logs/", eval_freq=5000,
                             deterministic=True, render=False)

OLD_MODEL_NAME = "KrabbelTest015"
MODEL_NAME = "KrabbelTest018"
LEARING_TIMESTEPS = 100_000
USE_OLD_MODEL = False
logdir = "logs"
if not os.path.exists(logdir):
    os.makedirs(logdir)

if not USE_OLD_MODEL:
    model = alg("MultiInputPolicy",
                        env,
                        # seed=10,
                        learning_starts=5_000,
                        # gamma=1-0.00016,
                        # learning_rate=0.0005,
                        ent_coef=0.0005,
                        # policy_kwargs=dict(
                        #     activation_fn=th.nn.ReLU,
                        #     net_arch=[128, 128]),
                        # train_freq=(1, "step"),
                        use_sde=True,
                        verbose=1,
                        device="auto",
                        tensorboard_log=logdir)
else:
    print(f"Using old Model {OLD_MODEL_NAME}")
    model = alg.load(f"{OLD_MODEL_NAME}")
    model.set_env(env)

model.learn(total_timesteps=LEARING_TIMESTEPS, callback=eval_callback, progress_bar=True)
model.save(MODEL_NAME)
