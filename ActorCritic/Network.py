# -*- coding: utf-8 -*-
import os
import tensorflow.keras as keras
from tensorflow.keras.layers import Dense

class ActorCriticNetwork(keras.Model):
    def __init__(self, n_actions, fc1_dims=100, fc2_dims=100,
                 name="actor_critic", chkpt_dir="tmp/100_relu_100_relu"):
        super(ActorCriticNetwork, self).__init__()
        self.fc1_dims = fc1_dims
        self.fc2_dims = fc2_dims
        self.n_actions = n_actions
        self.model_name = name
        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, name+"_ac")
        
        self.fc1 = Dense(self.fc1_dims, activation="tanh")
        self.fc2 = Dense(self.fc2_dims, activation="selu")
        self.v = Dense(1, activation=None)
        self.pi = Dense(n_actions, activation="softmax")
    
    def call(self, state):
        value = self.fc1(state)
        value = self.fc2(value)
        
        v = self.v(value)
        pi = self.pi(value)
        
        return v, pi
