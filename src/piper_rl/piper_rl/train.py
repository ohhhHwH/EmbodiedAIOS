import os
import numpy as np
from gymnasium.wrappers import RecordVideo
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
import time


def decay_schedule(initial_value):
    def func(progress_remaining):
        return initial_value * (progress_remaining)

    return func


def make_env(MyRobotEnv, worker_id):
    def _init():
        env = MyRobotEnv(worker_id=worker_id)
        return env

    return _init


def train():
    if args.gazebo:
        env = GazeboRobotEnv()
    else:
        if args.proc > 1:
            from stable_baselines3.common.vec_env import SubprocVecEnv

            env = SubprocVecEnv(
                [make_env(MujocoRobotEnv, i) for i in range(args.proc)],
                start_method="spawn",
            )
        else:
            # 需要 export MUJOCO_GL=egl
            env = MujocoRobotEnv(render_mode="rgb_array")
            # 多进程仿真不支持录制
            if args.train_record:
                video_dir = "./videos/"
                env = RecordVideo(
                    env,
                    video_folder=video_dir,
                    episode_trigger=lambda e: e % 100 == 0,
                    video_length=5000,
                )
    model = PPO(
        policy="MlpPolicy",
        env=env,
        device="cpu",
        policy_kwargs=dict(
            net_arch=[256, 512, 256, 128],
            log_std_init=-2.0,
            ortho_init=True,
        ),
        # 在已训练好的模型基础上继续训练，需要调小学习率，如3e-5
        learning_rate=decay_schedule(3e-4),
        batch_size=1024,
        n_steps=2048,
        gamma=0.99,
        verbose=1,
        # 越高越鼓励探索，但不能太高，否则std会变大，策略会变得不稳定，一般1e-2以内
        ent_coef=1e-2,
        tensorboard_log="./ppo_logs/",
    )
    # model.set_parameters("ppo_piper_final_best.zip")

    checkpoint_callback = CheckpointCallback(
        save_freq=100000, save_path="./ppo_models/", name_prefix="piper_rl_checkpoint"
    )

    model.learn(total_timesteps=1000 * 1000 * 100, callback=checkpoint_callback)

    model.save("ppo_piper_final")
    print("✅ 模型训练完成，已保存为 ppo_piper_final.zip")


def test():
    if args.gazebo:
        env = GazeboRobotEnv()
    else:
        env = MujocoRobotEnv()
        video_dir = "./videos/"
        env = RecordVideo(
            env,
            video_folder=video_dir,
            episode_trigger=lambda e: e % 100 == 0,
            video_length=5000,
        )
    model = PPO.load("ppo_piper_final_maximize_z")
    obs = env.reset()
    for epoch in range(100000):
        # time.sleep(1)
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)

        # print(f"epoch {epoch}, Reward:", reward)
        if done:
            print("🎉 成功抓取，重新开始")
            obs = env.reset()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true", help="测试训练好的策略")
    parser.add_argument("--gazebo", action="store_true", help="使用mujoco仿真")
    parser.add_argument(
        "--train_record",
        default=True,
        action="store_true",
        help="训练时定时录制训练过程",
    )
    parser.add_argument("--proc", default=64, help="并行仿真进程数")
    args = parser.parse_args()
    if args.gazebo:
        from piper_rl_gazebo_node import GazeboRobotEnv

        print("🚀 使用Gazebo仿真")
    else:
        from piper_rl_mujoco import MujocoRobotEnv

        print("🚀 使用Mujoco仿真")

    if args.test:
        test()
    else:
        train()
