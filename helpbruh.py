from RobitEnv2 import RobitEnvironment
from stable_baselines3 import SAC as alg
from stable_baselines3.common.env_util import make_vec_env
 
env = RobitEnvironment(True)

# MODEL_NAME = "opt_models/SAC/SAC_Model_1.5764523333333333"
MODEL_NAME = "KrabbelTest015"
# MODEL_NAME = "logs/best_model.zip"
model = alg.load(MODEL_NAME)
obs = env.reset()
i = 0
while 1:
    action, _states = model.predict(obs, deterministic=True)
    #print(action)
    obs, rewards, done, info = env.step(action)
    i += rewards
    print(info)
    if done:
        env.close()
        break

print(i)
