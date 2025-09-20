#!/usr/bin/env python3

import rclpy
import numpy as np
import time
import os
import matplotlib.pyplot as plt
from collections import deque

# Import our TD3 implementation and environment
from td3_algorithm import TD3, ReplayBuffer
from environment_interface import SafeDroneEnvironment

class TD3TrainingNode:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        
        # Create environment
        self.env = SafeDroneEnvironment()
        
        # Training parameters
        self.max_episodes = 1000
        self.max_timesteps = 1000
        self.start_timesteps = 1000  # Random actions before training
        self.eval_freq = 50  # Evaluate every N episodes
        self.save_model_freq = 100  # Save model every N episodes
        
        # Environment parameters
        self.state_dim = 15  # [rel_pos(2), vel(2), dist_to_goal(1), att(2), lidar(8)]
        self.action_dim = 2  # [dx, dy]
        self.max_action = 1.0  # Normalized action space
        
        # TD3 parameters
        self.lr = 3e-4
        self.discount = 0.99
        self.tau = 0.005
        self.policy_noise = 0.2
        self.noise_clip = 0.5
        self.policy_freq = 2
        self.batch_size = 256
        self.exploration_noise = 0.1
        
        # Initialize TD3 agent
        self.agent = TD3(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            max_action=self.max_action,
            lr=self.lr,
            discount=self.discount,
            tau=self.tau,
            policy_noise=self.policy_noise,
            noise_clip=self.noise_clip,
            policy_freq=self.policy_freq
        )
        
        # Initialize replay buffer
        self.replay_buffer = ReplayBuffer(max_size=1e6)
        
        # Training tracking
        self.episode_rewards = []
        self.episode_lengths = []
        self.episode_successes = []
        self.evaluation_scores = []
        
        # Create directories for saving
        self.model_dir = "models"
        self.log_dir = "logs"
        os.makedirs(self.model_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
        
        print(f"TD3 Training Node initialized")
        print(f"State dimension: {self.state_dim}")
        print(f"Action dimension: {self.action_dim}")
        print(f"Max action: {self.max_action}")
    
    def train(self):
        """Main training loop"""
        total_timesteps = 0
        evaluations = []
        
        print("Starting training...")
        
        for episode in range(self.max_episodes):
            episode_start_time = time.time()
            
            # Reset environment
            state = self.env.reset()
            
            # Wait for initial state
            while state is None:
                rclpy.spin_once(self.env, timeout_sec=0.1)
                state = self.env.get_state()
            
            episode_reward = 0
            episode_timesteps = 0
            done = False
            
            print(f"\n--- Episode {episode + 1}/{self.max_episodes} ---")
            print(f"Goal position: {self.env.goal_position}")
            
            while not done and episode_timesteps < self.max_timesteps:
                # Select action
                if total_timesteps < self.start_timesteps:
                    # Random actions for initial exploration
                    action = np.random.uniform(-1, 1, self.action_dim)
                else:
                    # TD3 action with exploration noise
                    action = self.agent.select_action(state, noise=self.exploration_noise)
                
                # Execute action
                next_state, reward, done, info = self.env.step(action)
                
                # Process ROS messages
                rclpy.spin_once(self.env, timeout_sec=0.01)
                
                # Wait for next state if not available
                if next_state is None:
                    continue
                
                episode_reward += reward
                episode_timesteps += 1
                total_timesteps += 1
                
                # Store transition in replay buffer
                self.replay_buffer.add(state, action, next_state, reward, float(done))
                
                state = next_state
                
                # Train agent
                if total_timesteps >= self.start_timesteps and len(self.replay_buffer) >= self.batch_size:
                    self.agent.train(self.replay_buffer, self.batch_size)
                
                # Print progress
                if episode_timesteps % 100 == 0:
                    attitude_safe = info.get('attitude_safe', True)
                    print(f"Step {episode_timesteps}: Reward={reward:.2f}, Distance={info.get('distance_to_goal', 0):.2f}m, "
                          f"Attitude Safe={attitude_safe}")
                
                # Check for unsafe conditions
                if not info.get('attitude_safe', True):
                    print("Episode ended: Unsafe attitude detected!")
                    break
                
                if info.get('distance_to_goal', float('inf')) < 1.0:
                    print("Episode ended: Goal reached!")
                    break
            
            # Episode summary
            episode_duration = time.time() - episode_start_time
            success = info.get('distance_to_goal', float('inf')) < 1.0
            
            self.episode_rewards.append(episode_reward)
            self.episode_lengths.append(episode_timesteps)
            self.episode_successes.append(success)
            
            print(f"Episode {episode + 1} Summary:")
            print(f"  Reward: {episode_reward:.2f}")
            print(f"  Length: {episode_timesteps} steps")
            print(f"  Duration: {episode_duration:.1f}s")
            print(f"  Success: {success}")
            print(f"  Final distance to goal: {info.get('distance_to_goal', 0):.2f}m")
            
            # Evaluation
            if (episode + 1) % self.eval_freq == 0:
                eval_score = self.evaluate_agent()
                self.evaluation_scores.append((episode + 1, eval_score))
                print(f"Evaluation after episode {episode + 1}: {eval_score:.2f}")
            
            # Save model
            if (episode + 1) % self.save_model_freq == 0:
                model_path = os.path.join(self.model_dir, f"td3_drone_episode_{episode + 1}.pth")
                self.agent.save(model_path)
                print(f"Model saved to {model_path}")
            
            # Plot training progress
            if (episode + 1) % 50 == 0:
                self.plot_training_progress()
        
        print("Training completed!")
        
        # Final save
        final_model_path = os.path.join(self.model_dir, "td3_drone_final.pth")
        self.agent.save(final_model_path)
        print(f"Final model saved to {final_model_path}")
        
        # Final plots
        self.plot_training_progress(save=True)
        self.save_training_logs()
    
    def evaluate_agent(self, num_episodes=5):
        """Evaluate the current agent performance"""
        eval_rewards = []
        
        print("Running evaluation...")
        
        for eval_ep in range(num_episodes):
            state = self.env.reset()
            
            # Wait for initial state
            while state is None:
                rclpy.spin_once(self.env, timeout_sec=0.1)
                state = self.env.get_state()
            
            eval_reward = 0
            eval_done = False
            eval_steps = 0
            
            while not eval_done and eval_steps < self.max_timesteps:
                # No exploration noise during evaluation
                action = self.agent.select_action(state, noise=0.0)
                next_state, reward, eval_done, info = self.env.step(action)
                
                # Process ROS messages
                rclpy.spin_once(self.env, timeout_sec=0.01)
                
                if next_state is None:
                    continue
                
                eval_reward += reward
                eval_steps += 1
                state = next_state
            
            eval_rewards.append(eval_reward)
            print(f"  Eval episode {eval_ep + 1}: Reward={eval_reward:.2f}, Steps={eval_steps}")
        
        avg_eval_reward = np.mean(eval_rewards)
        return avg_eval_reward
    
    def plot_training_progress(self, save=False):
        """Plot training progress"""
        if len(self.episode_rewards) == 0:
            return
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Episode rewards
        axes[0, 0].plot(self.episode_rewards)
        axes[0, 0].set_title('Episode Rewards')
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Reward')
        axes[0, 0].grid(True)
        
        # Moving average of rewards
        if len(self.episode_rewards) > 10:
            window = min(50, len(self.episode_rewards) // 4)
            moving_avg = np.convolve(self.episode_rewards, np.ones(window)/window, mode='valid')
            axes[0, 1].plot(moving_avg)
            axes[0, 1].set_title(f'Moving Average Rewards (window={window})')
            axes[0, 1].set_xlabel('Episode')
            axes[0, 1].set_ylabel('Average Reward')
            axes[0, 1].grid(True)
        
        # Episode lengths
        axes[1, 0].plot(self.episode_lengths)
        axes[1, 0].set_title('Episode Lengths')
        axes[1, 0].set_xlabel('Episode')
        axes[1, 0].set_ylabel('Steps')
        axes[1, 0].grid(True)
        
        # Success rate
        if len(self.episode_successes) > 0:
            window = min(20, len(self.episode_successes))
            success_rate = np.convolve(
                [1.0 if s else 0.0 for s in self.episode_successes], 
                np.ones(window)/window, 
                mode='valid'
            )
            axes[1, 1].plot(success_rate * 100)
            axes[1, 1].set_title(f'Success Rate (window={window})')
            axes[1, 1].set_xlabel('Episode')
            axes[1, 1].set_ylabel('Success Rate (%)')
            axes[1, 1].grid(True)
            axes[1, 1].set_ylim(0, 100)
        
        plt.tight_layout()
        
        if save:
            plot_path = os.path.join(self.log_dir, 'training_progress.png')
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            print(f"Training progress plot saved to {plot_path}")
        
        plt.show()
    
    def save_training_logs(self):
        """Save training logs to files"""
        # Save rewards
        rewards_path = os.path.join(self.log_dir, 'episode_rewards.txt')
        np.savetxt(rewards_path, self.episode_rewards, fmt='%.4f')
        
        # Save episode lengths
        lengths_path = os.path.join(self.log_dir, 'episode_lengths.txt')
        np.savetxt(lengths_path, self.episode_lengths, fmt='%d')
        
        # Save success indicators
        success_path = os.path.join(self.log_dir, 'episode_successes.txt')
        np.savetxt(success_path, [1 if s else 0 for s in self.episode_successes], fmt='%d')
        
        # Save evaluation scores
        if self.evaluation_scores:
            eval_path = os.path.join(self.log_dir, 'evaluation_scores.txt')
            with open(eval_path, 'w') as f:
                for episode, score in self.evaluation_scores:
                    f.write(f"{episode}\t{score:.4f}\n")
        
        print(f"Training logs saved to {self.log_dir}")
    
    def test_agent(self, model_path, num_episodes=10):
        """Test a trained agent"""
        print(f"Testing agent from {model_path}")
        
        # Load trained model
        self.agent.load(model_path)
        
        test_rewards = []
        test_successes = []
        
        for test_ep in range(num_episodes):
            print(f"\n--- Test Episode {test_ep + 1}/{num_episodes} ---")
            
            state = self.env.reset()
            
            # Wait for initial state
            while state is None:
                rclpy.spin_once(self.env, timeout_sec=0.1)
                state = self.env.get_state()
            
            test_reward = 0
            test_done = False
            test_steps = 0
            
            print(f"Goal position: {self.env.goal_position}")
            
            while not test_done and test_steps < self.max_timesteps:
                # No exploration noise during testing
                action = self.agent.select_action(state, noise=0.0)
                next_state, reward, test_done, info = self.env.step(action)
                
                # Process ROS messages
                rclpy.spin_once(self.env, timeout_sec=0.01)
                
                if next_state is None:
                    continue
                
                test_reward += reward
                test_steps += 1
                state = next_state
                
                # Print progress
                if test_steps % 100 == 0:
                    print(f"Step {test_steps}: Distance to goal={info.get('distance_to_goal', 0):.2f}m")
            
            success = info.get('distance_to_goal', float('inf')) < 1.0
            test_rewards.append(test_reward)
            test_successes.append(success)
            
            print(f"Test episode {test_ep + 1} results:")
            print(f"  Reward: {test_reward:.2f}")
            print(f"  Steps: {test_steps}")
            print(f"  Success: {success}")
            print(f"  Final distance to goal: {info.get('distance_to_goal', 0):.2f}m")
        
        # Test summary
        avg_reward = np.mean(test_rewards)
        success_rate = np.mean(test_successes) * 100
        
        print(f"\n=== Test Summary ===")
        print(f"Average reward: {avg_reward:.2f}")
        print(f"Success rate: {success_rate:.1f}%")
        print(f"Episodes: {num_episodes}")
        
        return avg_reward, success_rate
    
    def shutdown(self):
        """Clean shutdown"""
        self.env.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

def main():
    """Main function to run training or testing"""
    import argparse
    
    parser = argparse.ArgumentParser(description='TD3 Drone Control')
    parser.add_argument('--mode', choices=['train', 'test'], default='train',
                        help='Mode: train or test')
    parser.add_argument('--model', type=str, default=None,
                        help='Path to model file for testing')
    parser.add_argument('--episodes', type=int, default=1000,
                        help='Number of episodes for training')
    parser.add_argument('--test_episodes', type=int, default=10,
                        help='Number of episodes for testing')
    
    args = parser.parse_args()
    
    try:
        trainer = TD3TrainingNode()
        
        if args.mode == 'train':
            trainer.max_episodes = args.episodes
            trainer.train()
        elif args.mode == 'test':
            if args.model is None:
                print("Error: --model path required for testing")
                return
            trainer.test_agent(args.model, args.test_episodes)
        
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        trainer.shutdown()

if __name__ == '__main__':
    main()