import os
import gym
import time
from ascento_gym_old import Ascento
import matplotlib.pyplot as plt
# from balance_pend import InvertedPendulumEnv as Ascento
from stable_baselines3 import PPO, DDPG, SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold

env = Ascento()
model = PPO.load("/home/gaurav/final_year_project/Ascetno_RL_Joint_Limited/Training/model_for_demo.zip", env = env)

env = Ascento()
env.reset_model()
for i_episode in range(1):
    observation = env.reset()
    done = None
    while not done:
        # env.render()
        print(env.vx)
        action, _ = model.predict(env._get_obs())
        observation, reward, done, info = env.step(action)
        
# env.close()

# plt.rcParams.update({'font.size': 20})
# plt.subplot_tool()
# plt.tight_layout()
plt.subplot(2, 1, 1)
plt.plot(env.roll_list, label="Roll")
plt.plot(env.pitch_list, label="Pitch")
# plt.ylim(-90, 90)
plt.title("Orientation")
plt.xlabel("Time in mili-seconds")
plt.ylabel("Degrees")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(env.z_list, label="Z Position")
plt.title("Position")
plt.xlabel("Time in mili-seconds")
plt.ylabel("Meter")
plt.legend()

plt.suptitle("PID on Bump Terrain with External Distrubance")
# plt.suptitle("PID on Bump Terrain Terrain with No External Distrubance")

plt.show()