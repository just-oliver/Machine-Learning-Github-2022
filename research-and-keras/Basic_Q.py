# -*- coding: utf-8 -*-

import gym
import numpy as np
import time
import matplotlib.pyplot as plt


env = gym.make("FrozenLake-v0")
stateS = env.observation_space.n # The number of possible states
actionS = env.action_space.n # The number of possible actions for each state

Q = np.zeros((stateS, actionS)) # Initialising the Q-table

episodeS = 10000 # The number of separate instances that will be generated
max_stepS = 100 # The maximum number of steps in an instance

learning_rate = 0.81 # Learning rate - Higher values cause Q-table to update faster
gamma = 0.96 # Discount factor - Lower values cause Q-table to value current reward compared to future rewards

epsilon = 0.9 # The starting probability of a scenario using a random action instead of the optimal action

rewards = []
for episode in range(episodeS):
    
    state = env.reset() # Reset the environment
    for _ in range(max_stepS): # Loop for every step
        
        #env.render()
        
        if np.random.uniform(0,1) < epsilon:
            action = env.action_space.sample() # Pick a random action
        else:
            action = np.argmax(Q[state, :]) # Pick the best action according to current Q-table
        
        next_state, reward, done, _ = env.step(action) # Progress the simulation
        
        Q[state, action] = Q[state, action] + learning_rate * (reward + gamma * np.max(Q[next_state, :]) - Q[state, action]) # This function updates the Q-table
        
        state = next_state
        
        if done: # Once simulation is complete
            rewards.append(reward) # Add reward to list of rewards
            epsilon -= 0.001 # Decrease chance of random action choice
            break

print(Q)
print(f"average reward: {sum(rewards)/len(rewards)}:")

def get_average(values):
    return sum(values) / len(values)

avg_rewards = []
for i in range(0, len(rewards), 100):
    avg_rewards.append(get_average(rewards[i:i+100]))

plt.plot(avg_rewards)
plt.ylabel("average reward")
plt.xlabel("episodes (100\'s)")
plt.show()
