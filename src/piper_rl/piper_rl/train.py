import os
import numpy as np
import gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from piper_rl_node import MyRobotEnv
import time

def train():
    env = MyRobotEnv()
    model = PPO(
        policy="MlpPolicy",
        env=env,
        policy_kwargs=dict(net_arch=[256, 256, 128]),
        learning_rate=1e-3,
        batch_size=128,
        n_steps=1024,
        gamma=0.98,
        verbose=1,
        tensorboard_log="./ppo_logs/"
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=50000,
        save_path="./ppo_models/",
        name_prefix="piper_rl"
    )

    model.learn(
        total_timesteps=200000,
        callback=checkpoint_callback
    )

    model.save("ppo_piper_final")
    print("✅ 模型训练完成，已保存为 ppo_piper_final.zip")

def test():
    env = MyRobotEnv()
    model = PPO.load("ppo_piper_final")
    obs = env.reset()
    for _ in range(1000):
        time.sleep(1)
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)

        print("Reward:", reward)
        if done:
            print("🎉 成功抓取，重新开始")
            obs = env.reset()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true", help="测试训练好的策略")
    args = parser.parse_args()

    if args.test:
        test()
    else:
        train()
