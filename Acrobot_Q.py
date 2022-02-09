# -*- coding: utf-8 -*-

import gym
import numpy as np

def mapper(obs):
    stat = 0
    stat += (1 + obs[0]) // 0.5
    stat += ((1 + obs[1]) // 0.5) * 4
    stat += ((1 + obs[2]) // 0.5) * 16
    stat += ((1 + obs[3]) // 0.5) * 64
    stat += (((4 * np.pi) + obs[4]) // (2 * np.pi)) * 256
    stat += (((9 * np.pi) + obs[5]) // (4.5 * np.pi)) * 1024
    return int(stat)

env = gym.make("Acrobot-v1")


Q = np.zeros((4096, 3))

episodeS = 10
max_stepS = 500

learning_rate = 0.9
discount = 0.95
random_prob = 0.2

rewards = []
for episode in range(episodeS):
    observation = env.reset()
    state = mapper(observation)
    
    for _ in range(max_stepS):
        env.render()
        
        if np.random.uniform(0, 1) < random_prob:
            action = env.action_space.sample()
        else:
            action = np.argmax(Q[state, :])
        
        next_observation, non_reward, done, _ = env.step(action)
        next_state = mapper(next_observation)
        #reward = 1 - 0.5 * (next_observation[0] * (1 + next_observation[2]))
        reward = next_observation[5] ** 2
        
        Q[state, action] = Q[state, action] + learning_rate * (reward + discount * np.max(Q[next_state, :]) - Q[state, action]) # This function updates the Q-table
        
        state = next_state
        
        if done:
            rewards.append(reward)
            random_prob -= 0.01
            break

env.close()
print(rewards)
