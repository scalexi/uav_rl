# File: px4_rl_project/train_agent.py

import os
import rclpy
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from px4_rl_project.environment.px4_nav_obstacle_env import PX4NavObstacleEnv

def main():
    log_dir = os.path.expanduser("~/ws_ros2/ppo_nav_tensorboard")
    os.makedirs(log_dir, exist_ok=True)

    checkpoint_dir = os.path.expanduser("~/ws_ros2/ppo_nav_checkpoints")
    os.makedirs(checkpoint_dir, exist_ok=True)

    env = PX4NavObstacleEnv()

    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=checkpoint_dir,
        name_prefix="ppo_nav"
    )

    model.learn(total_timesteps=200000, callback=checkpoint_callback)

    model.save(os.path.expanduser("~/ws_ros2/ppo_navigation_final.zip"))

if __name__ == "__main__":
    main()
