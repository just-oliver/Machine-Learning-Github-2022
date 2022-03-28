# -*- coding: utf-8 -*-
import gym
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers

model = keras.models.load_model("Acrobot_Model")

env = gym.make("Acrobot-v1")  # Create the environment
max_steps_per_episode = 500

state = env.reset()
for timestep in range(1, max_steps_per_episode):
    env.render()
    state = tf.convert_to_tensor(state)
    state = tf.expand_dims(state, 0)
    
    action_probs, critic_value = model(state)
    action = np.random.choice(3, p=np.squeeze(action_probs))
    state, reward, done, _ = env.step(action)
    if done:
        break

env.close()
