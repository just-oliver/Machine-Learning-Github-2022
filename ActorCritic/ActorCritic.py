# -*- coding: utf-8 -*-
import tensorflow as tf
from tensorflow.keras.optimizers import Adam
import tensorflow_probability as tfp
from Network import ActorCriticNetwork

class Agent:
    def __init__(self, alpha=0.0003, gamma=0.99, n_actions=2):
        self.gamma = gamma
        self.n_actions = n_actions
        self.action = None
        self.action_space = [i for i in range(self.n_actions)]
        
        self.actor_critic = ActorCriticNetwork(n_actions=n_actions)
        
        self.actor_critic.compile(optimizer=Adam(learning_rate=alpha))
    
    def choose_action(self, observation):
        state = tf.convert_to_tensor(observation)
        state = tf.expand_dims(state, 0)
        _, probs = self.actor_critic(state)
        
        action_probabilities = tfp.distributions.Categorical(probs=probs)
        action = action_probabilities.sample()
        #action = tf.expand_dims(tf.convert_to_tensor(np.argmax(probs)),0)
        self.action = action
        
        #return np.argmax(probs)
        return action.numpy()[0]
    
    def save_models(self):
        print("...saving models...")
        self.actor_critic.save_weights(self.actor_critic.checkpoint_file)
    
    def load_models(self):
        print("...loading models...")
        self.actor_critic.load_weights(self.actor_critic.checkpoint_file)
    
    def learn(self, state, reward, state_, done):
        state = tf.convert_to_tensor([state], dtype=tf.float32)
        state_ = tf.convert_to_tensor([state_], dtype=tf.float32)
        reward = tf.convert_to_tensor(reward, dtype=tf.float32)
        
        with tf.GradientTape() as tape:
            state_value, probs = self.actor_critic(state)
            state_value_, _ = self.actor_critic(state_)
            state_value = tf.squeeze(state_value)
            state_value_ = tf.squeeze(state_value_)
            
            action_probs = tfp.distributions.Categorical(probs=probs)
            log_probs = action_probs.log_prob(self.action)
            
            delta = reward + self.gamma*state_value_*(1-int(done)) - state_value
            actor_loss = -log_probs*delta
            critic_loss = delta**2
            total_loss = actor_loss + critic_loss
            
        gradient = tape.gradient(total_loss, self.actor_critic.trainable_variables)
        self.actor_critic.optimizer.apply_gradients(zip(
            gradient, self.actor_critic.trainable_variables))