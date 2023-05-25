from typing import Any
from typing import Dict
import os
import gym
import optuna
from optuna.pruners import MedianPruner
from optuna.samplers import TPESampler
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
import torch
import torch.nn as nn
from stable_baselines3.common.env_util import make_vec_env
from RobitEnv2 import RobitEnvironment
from stable_baselines3.common.evaluation import evaluate_policy


N_TRIALS = 50
N_STARTUP_TRIALS = 5
N_EVALUATIONS = 2
N_TIMESTEPS = int(1e4)
EVAL_FREQ = int(N_TIMESTEPS / N_EVALUATIONS)
N_EVAL_EPISODES = 3
model_dir = f"opt_models/SAC"

ENV_ID = RobitEnvironment(False)

if not os.path.exists(model_dir):
    os.makedirs(model_dir)

DEFAULT_HYPERPARAMS = {
    "policy": "MultiInputPolicy",
    "env": ENV_ID,
}

def sample_SAC_params(trial: optuna.Trial) -> Dict[str, Any]:
    """Sampler for SAC hyperparameters."""
    seed = trial.suggest_int("seed", 1, 10000, log=True)
    gamma = trial.suggest_float("gamma", (1-0.1), (1-0.0001), log=True)
    learning_rate = trial.suggest_float("lr", 1e-5, 0.01, log=True)
    buffer_size = trial.suggest_int("buffer_size", 10000, 1e5, log=True)
    batch_size = trial.suggest_int("batch_size", 64, 2048, log=True)
    # ent_coef = trial.suggest_float("ent_coef", 0.00001, 0.1, log=True)
    net_arch = trial.suggest_categorical("net_arch", ["tiny", "small", "rasonable", "default"])
    activation_fn = trial.suggest_categorical("activation_fn", ["tanh", "relu"])

    # Display true values.
    # trial.set_user_attr("gamma_", gamma)

    #change this
    if net_arch == "tiny":
        net_arch = [32]
    elif net_arch == "small":
         net_arch = [128]
    elif net_arch == "rasonable":
         net_arch = [128, 128]
    elif net_arch == "default":
         net_arch = [400, 300]
    elif net_arch == "large":
         net_arch = [512, 512, 512]

    activation_fn = {"tanh": nn.Tanh, "relu": nn.ReLU}[activation_fn]

    trial.set_user_attr("net_arch_", net_arch)
    trial.set_user_attr("activation_fn_", activation_fn)

    return {
        "gamma": gamma,
        "seed": seed,
        "learning_rate": learning_rate,
        "ent_coef": "auto_0.1",
        "batch_size": batch_size,
        "buffer_size": buffer_size,
        "policy_kwargs": {
            "activation_fn": activation_fn,
            "net_arch": net_arch
        },
        "train_freq": (1, 'step'),
        # "verbose": 1,
    }
    
class TrialEvalCallback(EvalCallback):
    """Callback used for evaluating and reporting a trial."""

    def __init__(
        self,
        eval_env: gym.Env,
        trial: optuna.Trial,
        n_eval_episodes: int = 5,
        eval_freq: int = 10000,
        deterministic: bool = True,
        verbose: int = 0,
    ):
        super().__init__(
            eval_env=eval_env,
            n_eval_episodes=n_eval_episodes,
            eval_freq=eval_freq,
            deterministic=deterministic,
            verbose=verbose,
        )
        self.trial = trial
        self.eval_idx = 0
        self.is_pruned = False

    def _on_step(self) -> bool:
        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            super()._on_step()
            self.eval_idx += 1
            self.trial.report(self.last_mean_reward, self.eval_idx)
            # Prune trial if need.
            if self.trial.should_prune():
                self.is_pruned = True
                return False
        return True
    
    def _on_training_end(self) -> None:
        self.model.save(f"{model_dir}/SAC_Model_{self.last_mean_reward}")
        return super()._on_training_end()

    
def objective(trial: optuna.Trial) -> float:
    kwargs = DEFAULT_HYPERPARAMS.copy()
    # Sample hyperparameters.
    kwargs.update(sample_SAC_params(trial))
    # Create the RL model.
    model = SAC(**kwargs)
    # Create env used for evaluation.
    eval_env = Monitor(ENV_ID)
    # Create the callback that will periodically evaluate and report the performance.
    eval_callback = TrialEvalCallback(
        eval_env, trial, n_eval_episodes=N_EVAL_EPISODES, eval_freq=EVAL_FREQ, deterministic=True
    )

    nan_encountered = False
    try:
        model.learn(N_TIMESTEPS, callback=eval_callback, 
                    progress_bar=True
                    )
    except AssertionError as e:
        # Sometimes, random hyperparams can generate NaN.
        print(e)
        nan_encountered = True
    finally:
        # Free memory.
        model.env.close()
        eval_env.close()

    # Tell the optimizer that the trial failed.
    if nan_encountered:
        return float("nan")

    if eval_callback.is_pruned:
        raise optuna.exceptions.TrialPruned()

    return eval_callback.last_mean_reward

if __name__ == "__main__":
    # Set pytorch num threads to 1 for faster training.
    torch.set_num_threads(4)

    sampler = TPESampler(n_startup_trials=N_STARTUP_TRIALS)
    # Do not prune before 1/3 of the max budget is used.
    pruner = MedianPruner(n_startup_trials=N_STARTUP_TRIALS, n_warmup_steps=N_EVALUATIONS // 3)

    study = optuna.create_study(sampler=sampler, pruner=pruner, direction="maximize")
    try:
        study.optimize(objective, n_trials=N_TRIALS, 
                       # timeout=600
                       )
    except KeyboardInterrupt:
        pass

    print("Number of finished trials: ", len(study.trials))

    print("Best trial:")
    trial = study.best_trial

    print("  Value: ", trial.value)

    print("  Params: ")
    for key, value in trial.params.items():
        print("    {}: {}".format(key, value))

    print("  User attrs:")
    for key, value in trial.user_attrs.items():
        print("    {}: {}".format(key, value))