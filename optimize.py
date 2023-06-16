import Gelenkbestimmung as leg
import optuna
from optuna.samplers import TPESampler
import numpy as np


def get_new_parameters(trial: optuna.Trial)-> dict[str, float]:
    offset = trial.suggest_float("leg_offset", 1, 3, log=True)
    l_arm = trial.suggest_float("l_arm", 2, 6, log=True)
    angle = trial.suggest_float("angle", 30, 100, log=True)

    return {"offset_base": offset,
            "larm": l_arm,
            "angle_bound": angle}

def objective(trial: optuna.Trial) -> float:
    kwargs = get_new_parameters(trial)
    foot_end_point, c, rot_point = leg.func(**kwargs)
    bein_variation = np.std(np.gradient(foot_end_point[:, 0])) / abs(foot_end_point[1][-1]-foot_end_point[1][0])

    return float(bein_variation)

if __name__ == "__main__":
    N_TRIALS = 5000
    N_STARTUP_TRIALS = 5
    sampler = TPESampler(n_startup_trials=N_STARTUP_TRIALS)
    study = optuna.create_study(direction="minimize")
    
    try:
        study.optimize(objective, n_trials=N_TRIALS, 
                       timeout=600
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