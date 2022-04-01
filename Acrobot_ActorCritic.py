# -*- coding: utf-8 -*-
import gym
import numpy as np
from ActorCritic import Agent
import matplotlib.pyplot as plt

if __name__ == "__main__":
    env = gym.make("Acrobot-v1")
    agent = Agent(alpha=1e-3, n_actions=env.action_space.n)
    n_games = 5
    
    best_score = env.reward_range[0]
    score_history = []
    load_checkpoint = False
    update_model = True
    time_limit = 500
    
    if load_checkpoint:
        agent.load_models()
    
    for i in range(n_games):
        observation = env.reset()
        score = 0
        for timestep in range(1, time_limit):
            action = agent.choose_action(observation)
            observation_, reward, done, info = env.step(action)
            
            reward = 1 - observation_[0]
            if done and timestep < time_limit - 10:
                reward += 10 * (time_limit - timestep)
            score += reward
            if update_model:
                agent.learn(observation, reward, observation_, done)
            observation = observation_
            if done:
                break
        score_history.append(score)
        avg_score = np.mean(score_history[-10:])
            
        if avg_score > best_score:
            best_score = avg_score
            if update_model:
                agent.save_models()
        print("episode ", i, "score %.1f" % score, "avg_score %.1f" % avg_score)
     
    x = [i+1 for i in range(n_games)]
    plt.plot(x, score_history)
    plt.xlabel("Episode")
    plt.ylabel("Score")
    plt.title("10-relu-10-relu Testing")
    plt.ylim(0, 5000)