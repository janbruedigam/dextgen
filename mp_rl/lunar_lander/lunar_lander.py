from operator import itemgetter
import logging
from pathlib import Path
from tqdm import tqdm
import matplotlib.pyplot as plt
import torch
import gym
from ddpg import DDPG, DDPGActor, DDPGCritic
from replay_buffer import MemoryBuffer
from noise import OrnsteinUhlenbeckNoise
from utils import fill_buffer, running_average


logger = logging.getLogger(__name__)


def train(rank:int, size: int, config):
    logger.debug(f"Process {rank} startup successful.")
    config = config["lunarlander_ddpg"]
    logger.debug(f"Config: {config}")
    # Setup constants, hyperparameters, bookkeeping
    env = gym.make("LunarLanderContinuous-v2")
    n_states = len(env.observation_space.low)
    n_actions = len(env.action_space.low)
    gamma, actor_lr, critic_lr, tau = itemgetter("gamma", "actor_lr", "critic_lr", "tau")(config)
    
    noise_process = OrnsteinUhlenbeckNoise(mu=config["mu"], sigma=config["sigma"], dims=n_actions)
    ddpg = DDPG(DDPGActor(n_states, n_actions), DDPGActor(n_states, n_actions), 
                DDPGCritic(n_states, n_actions), DDPGCritic(n_states, n_actions), 
                actor_lr, critic_lr, tau, gamma, noise_process, action_clip=(-1.,1.), actor_clip=1.,
                critic_clip=1.)
    ddpg.init_ddp(rank)
    buffer = MemoryBuffer(config["buffer_size"])
    fill_buffer(env, buffer, config["buffer_size"])
    
    if rank == 0:
        status_bar = tqdm(total=config["epochs"]*config["cycles"], desc="Training iterations", position=0, leave=False)
        reward_log = tqdm(total=0, position=1, bar_format='{desc}', leave=False)
        episode_reward_list = []

    for epoch in range(config["epochs"]):
        for cycle in range(config["cycles"]):
            for episode in range(config["episodes"]):
                state = env.reset()
                done = False
                ddpg.noise_process.reset()
                ep_reward = 0
                while not done:
                    action = ddpg.action(torch.unsqueeze(torch.to_tensor(state), 0))[0]
                    next_state, reward, done, _ = env.step(action)
                    buffer.append((state, action, reward, next_state, done))
                    ep_reward += reward
                    state = next_state
                if rank == 0:
                    episode_reward_list.append(ep_reward)
            # Training
            for train_episode in range(config["train_episodes"]):
                batch = buffer.sample(config["batch_size"])
                batch = [ddpg.sanitize_array(x) for x in batch]
                ddpg.train_critic(batch)
                ddpg.train_actor(batch)
            ddpg.update_targets()
            if rank == 0:
                av_reward = running_average(episode_reward_list)[-1]
                reward_log.set_description_str("Current running average reward: {:.1f}".format(av_reward))
                status_bar.update()
    if rank == 0 and config["save_policy"]:
        path = Path(__file__).parent
        logger.debug(f"Saving models to {path}")
        ddpg.save(path / "lunarlander_ddpg_actor.pt", path / "lunarlander_ddpg_critic.pt", path / "ddpg.pkl")
        logger.debug(f"Saving complete")

    if rank == 0:    
        fig, ax = plt.subplots()
        ax.plot(episode_reward_list)
        smooth_reward = running_average(episode_reward_list, window=10)
        index = range(len(episode_reward_list)-len(smooth_reward), len(episode_reward_list))
        ax.plot(index, smooth_reward)
        ax.set_xlabel('Episode')
        ax.set_ylabel('Accumulated reward')
        ax.set_title('Agent performance over time')
        ax.legend(["Episode reward", "Running average reward"])
        plt.savefig(Path(__file__).parent / "reward.png")