import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import os
import copy
import collections
import random
import math
import xml.etree.ElementTree as ET
#from sumo_experiments.strategies.rl_util import *
from ..rl_networks import *



# https://github.com/ikostrikov/pytorch-ddpg-naf/blob/master/ddpg.py#L11
def soft_update(target, source, tau):
    """
    Perform DDPG soft update (move target params toward source based on weight
    factor tau)
    Inputs:
        target (torch.nn.Module): Net to copy parameters to
        source (torch.nn.Module): Net whose parameters to copy
        tau (float, 0 < x < 1): Weight factor for update
    """
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

# https://github.com/ikostrikov/pytorch-ddpg-naf/blob/master/ddpg.py#L15
def hard_update(target, source):
    """
    Copy network parameters from source to target
    """
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)




class RecurrentDDPGAgent(nn.Module):
    """DDPG agent variant with recurrent actor and feed-forward critic."""

    def __init__(self, num_in_pol, num_out_pol, num_in_critic, hidden_dim=64, lr=0.01, discrete_action=True):
        super().__init__()
        from sumo_experiments.strategies import maddpg_strategy as maddpg_module

        self.policy = RecurrentPolicyNetwork(num_in_pol, num_out_pol, hidden_dim=hidden_dim)
        self.target_policy = RecurrentPolicyNetwork(num_in_pol, num_out_pol, hidden_dim=hidden_dim)
        self.critic = maddpg_module.DeepNN(num_in_critic, 1, hidden_dim=hidden_dim)
        self.target_critic = maddpg_module.DeepNN(num_in_critic, 1, hidden_dim=hidden_dim)

        maddpg_module.hard_update(self.target_policy, self.policy)
        maddpg_module.hard_update(self.target_critic, self.critic)

        self.policy_optimizer = optim.Adam(self.policy.parameters(), lr=lr / 10)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)

        self.discrete_action = discrete_action
        self.recurrent_policy = True
        self._policy_hidden_state = None

    def step(self, obs, explore=False):
        from sumo_experiments.strategies import maddpg_strategy as maddpg_module

        obs = obs.to(next(self.policy.parameters()).device)
        action, hidden_state = self.policy(obs, hidden_state=self._policy_hidden_state, return_hidden=True)
        self._policy_hidden_state = hidden_state

        if self.discrete_action:
            if explore:
                action = nn.functional.gumbel_softmax(action, hard=True)
            else:
                action = maddpg_module.onehot_from_logits(action)
        return action

    def reset_recurrent_state(self):
        self._policy_hidden_state = None

    def get_params(self):
        return {
            'policy': self.policy.state_dict(),
            'critic': self.critic.state_dict(),
            'target_policy': self.target_policy.state_dict(),
            'target_critic': self.target_critic.state_dict(),
            'policy_optimizer': self.policy_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
        }

    def load_params(self, params):
        self.policy.load_state_dict(params['policy'])
        self.critic.load_state_dict(params['critic'])
        self.target_policy.load_state_dict(params['target_policy'])
        self.target_critic.load_state_dict(params['target_critic'])
        self.policy_optimizer.load_state_dict(params['policy_optimizer'])
        self.critic_optimizer.load_state_dict(params['critic_optimizer'])
        self.reset_recurrent_state()

    def to(self, device):
        self.policy.to(device)
        self.critic.to(device)
        self.target_policy.to(device)
        self.target_critic.to(device)
        self.reset_recurrent_state()
        return self



class DDPGAgent(nn.Module):
    """
    General class for DDPG agents (policy, critic, target policy, target
    critic, exploration noise)
    """
    def __init__(self, num_in_pol, num_out_pol, num_in_critic, hidden_dim=64,
                 lr=0.01, discrete_action=True):
        """
        Inputs:
            num_in_pol (int): number of dimensions for policy input
            num_out_pol (int): number of dimensions for policy output
            num_in_critic (int): number of dimensions for critic input
        """
        super().__init__()
        self.policy = DeepNN(num_in_pol, num_out_pol,
                                 hidden_dim=hidden_dim)
        self.critic = DeepNN(num_in_critic, 1,
                                 hidden_dim=hidden_dim)
        self.target_policy = DeepNN(num_in_pol, num_out_pol,
                                        hidden_dim=hidden_dim)
        self.target_critic = DeepNN(num_in_critic, 1,
                                        hidden_dim=hidden_dim)
        hard_update(self.target_policy, self.policy)
        hard_update(self.target_critic, self.critic)
        self.policy_optimizer = optim.Adam(self.policy.parameters(), lr=lr/10)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)

        self.exploration = 0.3  # epsilon for eps-greedy, unused
        self.discrete_action = discrete_action


    def step(self, obs, explore=False):
        """
        Take a step forward in environment for a minibatch of observations
        Inputs:
            obs (PyTorch Variable): Observations for this agent
            explore (boolean): Whether or not to add exploration noise
        Outputs:
            action (PyTorch Variable): Actions for this agent
        """
        action = self.policy(obs)
        if self.discrete_action:
            if explore:
                # note: for MADDPG step, we should output onehot vectors
                action = nn.functional.gumbel_softmax(action, hard=True) # consider replacing with other estimators from https://github.com/uoe-agents/revisiting-maddpg
            else:
                action = onehot_from_logits(action)
        return action

    def get_params(self):
        return {'policy': self.policy.state_dict(),
                'critic': self.critic.state_dict(),
                'target_policy': self.target_policy.state_dict(),
                'target_critic': self.target_critic.state_dict(),
                'policy_optimizer': self.policy_optimizer.state_dict(),
                'critic_optimizer': self.critic_optimizer.state_dict()}

    def load_params(self, params):
        self.policy.load_state_dict(params['policy'])
        self.critic.load_state_dict(params['critic'])
        self.target_policy.load_state_dict(params['target_policy'])
        self.target_critic.load_state_dict(params['target_critic'])
        self.policy_optimizer.load_state_dict(params['policy_optimizer'])
        self.critic_optimizer.load_state_dict(params['critic_optimizer'])

    def to(self, device):
        self.policy.to(device)
        self.critic.to(device)
        self.target_policy.to(device)
        self.target_critic.to(device)
        return self