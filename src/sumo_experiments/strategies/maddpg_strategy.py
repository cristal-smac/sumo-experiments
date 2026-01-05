#from sumo.tools.emissions.findMinDiffModel import model

from sumo_experiments.strategies import Strategy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
from sumo_experiments.strategies.maxpressure_strategy import MaxPressureStrategy
from sumo_experiments.strategies import IntellilightStrategy
import matplotlib.pyplot as plt
import collections



class MADDPGStrategy(IntellilightStrategy):
    """
    Implements an MADDPG system.

    References:
    - https://github.com/shariqiqbal2810/maddpg-pytorch/blob/master/algorithms/maddpg.py#L143
    - Lowe, R., Wu, Y. I., Tamar, A., Harb, J., Pieter Abbeel, O., & Mordatch, I. (2017). Multi-agent actor-critic for mixed cooperative-competitive environments. Advances in neural information processing systems, 30.
    """

    def __init__(self, network, period=10, episode_duration=300, reward_coeffs=(1, 1, 1, 1), gamma=0.99, buffer_size=10000, batch_size=32, 
                 steps_per_update=10, samples_before_update=1024, 
                 learning_rate=1e-2, tau=0.01, hidden_layer_size=64, yellow_time=3):
        """
        Init of class.
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param period: The duration of a period (in seconds).
        :type period: int or dict
        :param episode_duration: Maximum duration of one episode (in seconds).
        :type episode_duration: int
        :param reward_coeffs: Coefficients for computing the reward (e.g. queue, waiting time, throughput, etc.).
        :type reward_coeffs: tuple
        :param gamma: Gamma parameter for the Bellman equation, to compute Q-values.
        :type gamma: float or dict
        :param buffer_size: Memory buffer size for training the neural network. The network is trained when the memory buffer is full.
        :type buffer_size: int or dict
        :param batch_size: Number of samples per training batch.
        :type batch_size: int
        :param steps_per_update: Number of simulation steps between each network update.
        :type steps_per_update: int
        :param learning_rate: Learning rate for the neural networks. Must be a positive number.
        :type learning_rate: float or dict
        :param tau: Soft update coefficient for target network synchronization.
        :type tau: float
        :param hidden_layer_size: The size of the hidden layers of the neural network.
        :type hidden_layer_size: int or dict
        :param yellow_time: Yellow phases duration for all intersections (in seconds).
        :type yellow_time: int or dict
        """
        # super().__init__()
        self.network = network
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {tl_id: yellow_time for tl_id in network.TLS_DETECTORS}
        self.current_phase = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}
        self.time = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}
        self.current_max_time_index = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}
        self.current_yellow_time = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}
        self.started = False
        self.nb_phases = {}
        self.nb_switch = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}
        self.next_phase = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}
        self.period = period
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {tl_id: yellow_time for tl_id in network.TLS_DETECTORS}
        self.c1, self.c2, self.c3, self.c4 = reward_coeffs

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.rollout_device = torch.device('cpu')

        self.observation_sizes = {}
        self.action_space = {tl_id: list(self.network.TLS_DETECTORS[tl_id].keys()) for tl_id in self.network.TLS_DETECTORS}

        self.gamma = gamma if isinstance(gamma, dict) else {tl_id: gamma for tl_id in network.TLS_DETECTORS}
        self.tau = tau
        self.episode_duration = episode_duration

        self.buffer_size = buffer_size# if isinstance(buffer_size, dict) else {tl_id: buffer_size for tl_id in network.TLS_DETECTORS}
        self.batch_size = batch_size
        self.hidden_layer_size = hidden_layer_size if isinstance(hidden_layer_size, dict) else {tl_id: hidden_layer_size for tl_id in network.TLS_DETECTORS}
        self.steps_per_update = steps_per_update# if isinstance(update_target_frequency, dict) else {tl_id: update_target_frequency for tl_id in network.TLS_DETECTORS}
        self.samples_before_update = samples_before_update
        self.learning_rate = learning_rate if isinstance(learning_rate, dict) else {tl_id: learning_rate for tl_id in network.TLS_DETECTORS}

        self.maddpg = None
        self.agents = {}
        self.loss_history = {tl_id: [] for tl_id in self.network.TLS_DETECTORS}
        self.current_phase = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}

        self.number_of_trainings = {tl_id: 0 for tl_id in self.network.TLS_DETECTORS}
        self.mean_scores = []

        self.val_losses = []
        self.pol_losses = []
        self.rewards = []
        self.scores = []
        self.times = []
        self.phases_occurences = {identifiant: {} for identifiant in network.TLS_DETECTORS}
        self.phases_durations = {identifiant: [] for identifiant in network.TLS_DETECTORS}
        self.current_phase_duration = {identifiant: 0 for identifiant in network.TLS_DETECTORS}


    def run_all_agents(self, traci):
        """
        Process agents to make one action each.
        :param traci: The simulation Traci instance
        :type traci: Traci
        :return: Nothing
        """
        if not self.started:
            self.traci = traci
            for tl_id in self.network.TL_IDS:
                self._start_agent(tl_id)

            if self.maddpg is None: # not loading a saved model
                self.maddpg = MADDPG(agents=list(self.agents.values()), alg_types=['MADDPG' for _ in self.network.TLS_DETECTORS], 
                                    gamma=self.gamma[tl_id],
                                    discrete_action=True,
                                    tau=self.tau)
            self.replay_buffer = ReplayBuffer(max_steps=self.buffer_size, num_agents=self.maddpg.nagents, obs_dims=list(self.observation_sizes.values()),
                                               ac_dims=[len(self.action_space[tl_id]) for tl_id in self.network.TLS_DETECTORS])
            self.started = True

            # self.states = [self.get_state(tl_id) for tl_id in self.network.TL_IDS]
            # self.action_indices = [np.zeros_like(self.action_space[tl_id]) for tl_id in self.network.TL_IDS] # TODO: one hot encode current phase
            self.states = None
            self.action_indices = None
            self.changed_phase = [None for _ in self.network.TLS_DETECTORS]

            self.time_step = 1
        else:
            if self.traci.simulation.getTime() % self.episode_duration == 0:
                self.mean_scores.append(np.mean(self.scores))
                self.scores = []
                # reset states and action
                self.states = None
                self.action_indices = None
                self.changed_phase = [None for _ in self.network.TLS_DETECTORS]
                if self.network.TL_IDS:#[0] == "c":
                    plt.plot(range(len(self.mean_scores)), self.mean_scores)
                    plt.savefig('strategy_debug.png')
            for tl_id in self.network.TL_IDS:
                # assert self.time_step == self.traci.simulation.getTime(), print(self.time_step, self.traci.simulation.getTime())
                if 'y' in self.traci.trafficlight.getRedYellowGreenState(tl_id):
                    if self.current_yellow_time[tl_id] >= self.yellow_time[tl_id]:
                        self.traci.trafficlight.setPhase(tl_id, int(self.next_phase[tl_id]))
                        self.current_phase[tl_id] = self.next_phase[tl_id]
                        self.current_yellow_time[tl_id] = 0
                    else:
                        self.current_yellow_time[tl_id] += 1
                else:
                    current_phase = self.traci.trafficlight.getPhase(tl_id)
                    # Counting phase occurences
                    if current_phase not in self.phases_occurences[tl_id]:
                        self.phases_occurences[tl_id][current_phase] = 1
                    else:
                        self.phases_occurences[tl_id][current_phase] += 1
                    self.time[tl_id] += 1
                    self.current_phase_duration[tl_id] += 1
            if self.time_step % self.period == 0:
                self.switch_next_phase()
            if (len(self.replay_buffer) >= self.samples_before_update) and (self.time_step % self.steps_per_update == 0):
                self.train()
            self.time_step += 1


    def switch_next_phase(self):
        """
        Queue switch of the traffic light id_tls to the next, set up the yellow phase if needed.
        """
        next_phases = self.get_next_phases()
        for tl_id in self.network.TL_IDS:
            current_phase = self.traci.trafficlight.getPhase(tl_id)
            if next_phases[tl_id] != self.current_phase[tl_id]:
                self.nb_switch[tl_id] += 1
                self.next_phase[tl_id] = next_phases[tl_id]
                if self.traci.trafficlight.getPhase(tl_id) == self.nb_phases[tl_id] - 1:
                    self.traci.trafficlight.setPhase(tl_id, 0)
                else:
                    self.traci.trafficlight.setPhase(tl_id, int(self.current_phase[tl_id] + 1))
            self.time[tl_id] = 0
            self.phases_durations[tl_id].append((current_phase, self.current_phase_duration[tl_id]))
            self.current_phase_duration[tl_id] = 0

    def ohe_state(self, tl_id):
        phases = list(self.network.TLS_DETECTORS[tl_id].keys())
        num_phases = len(phases)
        idx = phases.index(self.current_phase[tl_id])
        one_hot = np.zeros(num_phases, dtype=np.bool_)
        one_hot[idx] = 1
        return one_hot
    
    def get_state(self, tl_id):
        detectors = self._detectors(tl_id)
        L = [self.traci.lanearea.getJamLengthVehicle(det) / 10 for det in detectors]
        W = [x / 20 for x in self.compute_waiting_time(detectors)]
        V = [x / 10 for x in self.compute_number_of_vehicles(tl_id)]
        P = self.ohe_state(tl_id).tolist()
        return np.array(L + W + V + P, dtype=np.float32)  # concatenate all values into a single array
    
    def get_next_phases(self, train=True):
        """
        Get the next phase for the controller using MADDPG.
        """
        # Store experience in replay buffer
        next_states = [self.get_state(tl_id) for tl_id in self.network.TLS_DETECTORS]
        rewards = [self.get_reward(tl_id, self.changed_phase[i]) for i, tl_id in enumerate(self.network.TLS_DETECTORS)]
        dones = [(self.traci.simulation.getTime()%self.episode_duration)==0 for _ in self.network.TLS_DETECTORS]
        scores = [self.get_score(tl_id, self.changed_phase[i]) for i, tl_id in enumerate(self.network.TLS_DETECTORS)]
        # if dones[0]:
        #     print(f"Rewards at time {self.traci.simulation.getTime()}: {rewards}")
        # if self.states is not None and self.action_indices is not None:
        #     with np.printoptions(precision=2, suppress=True):
        #         print(self.states[0], self.action_indices[0], rewards[0])

        # Global rewards for coordinating MADDPG
        rewards = np.ones_like(rewards) * np.mean(rewards)

        # Update buffer from previous action step
        if self.states is not None and self.action_indices is not None and not dones[0]:
            self.replay_buffer.push(self.states, self.action_indices, rewards, next_states, dones=dones)
            if "c" in self.network.TL_IDS:  # debugging for single intersection
                assert rewards[0]>-(self.c4*self.DEBUG_REWARD), (rewards, self.changed_phase)
                self.rewards.append(rewards[0])
                self.times.append(self.traci.simulation.getTime())
            self.scores.append(np.mean(scores))
        state_tensor = [torch.tensor(n_s, dtype=torch.float32).unsqueeze(0).to(self.rollout_device) for n_s in next_states]
        with torch.no_grad():
            torch_action_indices = self.maddpg.step(state_tensor, explore=True) # exploration handled by softmax
            action_indices = [ac.data.cpu().numpy() for ac in torch_action_indices]
            # DEBUG: force an action
            # s = 1 if self.traci.simulation.getTime() // 100 % 2 == 0 else -1
            # action_indices = [np.array([1,0])[::s]]

        if self.action_indices is None:
            self.changed_phase = [False for _ in self.network.TLS_DETECTORS]
        else:
            self.changed_phase = [np.argmax(action_indices[i]) != np.argmax(self.action_indices[i]) for i, tl_id in enumerate(self.network.TLS_DETECTORS)]
        self.states = next_states
        self.action_indices = action_indices     

        return {tl_id: self.action_space[tl_id][action_indices[i].argmax()] for i, tl_id in enumerate(self.network.TLS_DETECTORS)}

    def train(self):
        # Train the MADDPG model once buffer reaches minimum size
        self.maddpg.prep_training(device=self.device)
        for tl_id in self.network.TL_IDS:
            self.number_of_trainings[tl_id] += 1
        for a_i in range(self.maddpg.nagents):
            sample = self.replay_buffer.sample(self.batch_size,
                                        device=self.device, norm_rews=False)
            val_loss, pol_loss = self.maddpg.update(sample, a_i)
            self.val_losses.append(val_loss)
            self.pol_losses.append(pol_loss)
        self.maddpg.update_all_targets()
        self.maddpg.prep_rollouts(device=self.rollout_device)

    def get_reward(self, tl_id, change_phase=None):
        pressure = MaxPressureStrategy._compute_pressure(self, self.network.TLS_DETECTORS[tl_id])
        return -np.nanmean(list(pressure.values()))/2000
    
    def _start_agent(self, tl_id):
        """
        Start an agent at the beginning of the simulation.
        :param tl_id: The id of the TL
        :type tl_id: str
        """
        self.nb_phases[tl_id] = len(self.traci.trafficlight.getAllProgramLogics(tl_id)[0].phases)
        tl_logic = self.traci.trafficlight.getAllProgramLogics(tl_id)[0]
        phase_index = 0
        for phase in tl_logic.phases:
            phase.duration = 10000
            phase.maxDur = 10000
            phase.minDur = 10000
            phase_index += 1
        self.traci.trafficlight.setProgramLogic(tl_id, tl_logic)
        self.traci.trafficlight.setPhase(tl_id, 0)
        self.traci.trafficlight.setPhaseDuration(tl_id, 10000)

        # DEFER INITIALIZATION OF DDPG AGENTS TO HERE
        input_dims = {tl_id: len(self.get_state(tl_id)) for tl_id in self.network.TL_IDS}
        critic_dim = sum(input_dims.values())
        critic_dim += sum(len(self.action_space[tl_id]) for tl_id in self.network.TL_IDS)
        self.agents[tl_id] = DDPGAgent(num_out_pol=len(self.action_space[tl_id]), 
                                       num_in_pol=input_dims[tl_id], 
                                       num_in_critic=critic_dim, 
                                       hidden_dim=self.hidden_layer_size[tl_id],
                                       lr=self.learning_rate[tl_id],
                                       discrete_action=True).to(self.rollout_device)
        self.observation_sizes[tl_id] = input_dims[tl_id]

    def save_model(self, filepath):
        torch.save(self.maddpg.agents, filepath)

    def load_model(self, filepath):
        torch.serialization.add_safe_globals([DDPGAgent, DeepNN, nn.Linear, nn.functional.relu, optim.Adam, collections.defaultdict, dict])
        if self.maddpg is None:
            self.maddpg = MADDPG(agents=[], alg_types=['MADDPG' for _ in self.network.TLS_DETECTORS], 
                                 gamma=list(self.gamma.values())[0], # TODO: refactor?
                                 discrete_action=True,
                                 tau=self.tau)
        self.maddpg.agents = torch.load(filepath, weights_only=False)


class DeepNN(nn.Module):
    def __init__(self, input_dim, out_dim, hidden_dim=64, nonlin=nn.functional.relu, recurrent=True):
        super().__init__()
        self.recurrent = recurrent
        self.hidden_dim = hidden_dim
        self.device = None
        if not recurrent:
            self.fc1 = nn.Linear(input_dim, hidden_dim)
        else:
            self.fc1 = nn.LSTM(input_size=input_dim, hidden_size=hidden_dim, batch_first=True)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, out_dim)
        self.nonlin = nonlin
        
        # Persistent hidden state for recurrent networks (maintained across timesteps in rollouts)
        self.h_n = None
        self.c_n = None

    def reset_hidden_state(self, batch_size=1):
        """Reset LSTM hidden state. Call this at episode start or when needed."""
        if self.recurrent:
            device = next(self.parameters()).device
            self.h_n = torch.zeros(1, batch_size, self.hidden_dim, device=device)
            self.c_n = torch.zeros(1, batch_size, self.hidden_dim, device=device)

    def forward(self, X, use_hidden_state=False):
        """
        Forward pass.
        
        Args:
            X: Input tensor, shape (batch_size, input_dim) or (batch_size, seq_len, input_dim)
            use_hidden_state: If True and recurrent, use persistent hidden state (for rollouts).
                             If False, use fresh hidden state (for replay buffer training).
        """
        if self.recurrent:
            # Ensure X has sequence dimension
            if X.dim() == 2:
                X = X.unsqueeze(1)  # (batch_size, input_dim) -> (batch_size, 1, input_dim)
            
            if use_hidden_state and self.h_n is not None:
                # Use persistent hidden state from previous timestep
                lstm_out, (self.h_n, self.c_n) = self.fc1(X, (self.h_n, self.c_n))
            else:
                # Fresh hidden state (for training on replay buffer)
                lstm_out, (self.h_n, self.c_n) = self.fc1(X)
            
            h = self.nonlin(lstm_out[:, -1])  # take last timestep
        else: # not used as of now.
            # Remove sequence dimension if present for linear networks
            if X.dim() == 3:
                X = X.squeeze(1)
            h = self.nonlin(self.fc1(X))
        
        h = self.nonlin(self.fc2(h))
        out = self.fc3(h)
        return out
    

## MADDPG Algorithm
import torch
import torch.distributed as dist
from typing import List
loss_fn = torch.nn.HuberLoss()


class MADDPG(object):
    """
    Wrapper class for DDPG-esque (i.e. also MADDPG) agents in multi-agent task
    """
    def __init__(self, agents, alg_types,
                 gamma=0.95, tau=0.01,
                 discrete_action=False):
        """
        Inputs:
            agent_init_params (list of dict): List of dicts with parameters to
                                              initialize each agent
                num_in_pol (int): Input dimensions to policy
                num_out_pol (int): Output dimensions to policy
                num_in_critic (int): Input dimensions to critic
            alg_types (list of str): Learning algorithm for each agent (DDPG
                                       or MADDPG)
            gamma (float): Discount factor
            tau (float): Target update rate
            lr (float): Learning rate for policy and critic
            hidden_dim (int): Number of hidden dimensions for networks
            discrete_action (bool): Whether or not to use discrete action space
        """
        self.alg_types = alg_types
        self.agents: List[DDPGAgent] = agents
        self.gamma = gamma
        self.tau = tau
        self.discrete_action = discrete_action
        self.pol_dev = 'cpu'  # device for policies
        self.critic_dev = 'cpu'  # device for critics
        self.trgt_pol_dev = 'cpu'  # device for target policies
        self.trgt_critic_dev = 'cpu'  # device for target critics
        self.niter = 0

    @property
    def nagents(self):
        return len(self.agents) #len(alg_types)

    @property
    def policies(self):
        return [a.policy for a in self.agents]

    @property
    def target_policies(self):
        return [a.target_policy for a in self.agents]

    def step(self, observations, explore=False):
        """
        Take a step forward in environment with all agents
        Inputs:
            observations: List of observations for each agent
            explore (boolean): Whether or not to add exploration noise
        Outputs:
            actions: List of actions for each agent
        """
        assert len(observations) == self.nagents, (len(observations), self.nagents)
        return [a.step(obs, explore=explore)[0] for a, obs in zip(self.agents,
                                                                 observations)]

    def update(self, sample, agent_i, logger=None):
        """
        Update parameters of agent model based on sample from replay buffer
        Inputs:
            sample: tuple of (observations, actions, rewards, next
                    observations, and episode end masks) sampled randomly from
                    the replay buffer. Each is a list with entries
                    corresponding to each agent
            agent_i (int): index of agent to update
            logger (SummaryWriter from Tensorboard-Pytorch):
                If passed in, important quantities will be logged
        """
        obs, acs, rews, next_obs, dones = sample
        curr_agent = self.agents[agent_i]
        curr_agent.critic_optimizer.zero_grad()
        if self.discrete_action: # one-hot encode action
            all_trgt_acs = [nn.functional.gumbel_softmax(pi(nobs), hard=True) for pi, nobs in
                            zip(self.target_policies, next_obs)]
        else:
            all_trgt_acs = [pi(nobs) for pi, nobs in zip(self.target_policies,
                                                            next_obs)]
        trgt_vf_in = torch.cat((*next_obs, *all_trgt_acs), dim=1)
        target_value = (rews[agent_i].view(-1, 1) + self.gamma *
                        curr_agent.target_critic(trgt_vf_in) *
                        (1 - dones[agent_i].view(-1, 1)))

        vf_in = torch.cat((*obs, *acs), dim=1)

        actual_value = curr_agent.critic(vf_in)
        vf_loss = loss_fn(actual_value, target_value.detach())
        vf_loss.backward()

        torch.nn.utils.clip_grad_norm_(curr_agent.critic.parameters(), 0.5)
        curr_agent.critic_optimizer.step()

        curr_agent.policy_optimizer.zero_grad()

        if self.discrete_action:
            # Forward pass as if onehot (hard=True) but backprop through a differentiable
            # Gumbel-Softmax sample. The MADDPG paper uses the Gumbel-Softmax trick to backprop
            # through discrete categorical samples, but I'm not sure if that is
            # correct since it removes the assumption of a deterministic policy for
            # DDPG. Regardless, discrete policies don't seem to learn properly without it.
            curr_pol_out = curr_agent.policy(obs[agent_i])
            curr_pol_vf_in = nn.functional.gumbel_softmax(curr_pol_out, hard=False)
        else:
            curr_pol_out = curr_agent.policy(obs[agent_i])
            curr_pol_vf_in = curr_pol_out

        all_pol_acs = []
        for i, pi, ob in zip(range(self.nagents), self.policies, obs):
            if i == agent_i:
                all_pol_acs.append(curr_pol_vf_in)
                assert all_pol_acs[-1].requires_grad == True
            else: #discrete action must be one-hot encoded
                all_pol_acs.append(onehot_from_logits(pi(ob)))
                assert all_pol_acs[-1].requires_grad == False

        vf_in = torch.cat((*obs, *all_pol_acs), dim=1)

        pol_loss = -curr_agent.critic(vf_in).mean()
        pol_loss += (curr_pol_out**2).mean() * 1e-3 # regularization prevents large logits
        pol_loss.backward()

        torch.nn.utils.clip_grad_norm_(curr_agent.policy.parameters(), 0.5)
        curr_agent.policy_optimizer.step()
        if logger is not None:
            logger.add_scalars('agent%i/losses' % agent_i,
                               {'vf_loss': vf_loss,
                                'pol_loss': pol_loss},
                               self.niter)
        return vf_loss.cpu().detach().numpy(), pol_loss.cpu().detach().numpy()

    def update_all_targets(self):
        """
        Update all target networks (called after normal updates have been
        performed for each agent)
        """
        for a in self.agents:
            soft_update(a.target_critic, a.critic, self.tau)
            soft_update(a.target_policy, a.policy, self.tau)
        self.niter += 1

    def prep_training(self, device='gpu'):
        for a in self.agents:
            a.policy.train()
            a.critic.train()
            a.target_policy.train()
            a.target_critic.train()
        fn = lambda x: x.to(device)
        if not self.pol_dev == device:
            for a in self.agents:
                a.policy = fn(a.policy)
            self.pol_dev = device
        if not self.critic_dev == device:
            for a in self.agents:
                a.critic = fn(a.critic)
            self.critic_dev = device
        if not self.trgt_pol_dev == device:
            for a in self.agents:
                a.target_policy = fn(a.target_policy)
            self.trgt_pol_dev = device
        if not self.trgt_critic_dev == device:
            for a in self.agents:
                a.target_critic = fn(a.target_critic)
            self.trgt_critic_dev = device

    def prep_rollouts(self, device='cpu'):
        for a in self.agents:
            a.policy.eval()
        fn = lambda x: x.to(device)
        # only need main policy for rollouts
        if not self.pol_dev == device:
            for a in self.agents:
                a.policy = fn(a.policy)
            self.pol_dev = device

    def save(self, filename):
        """
        Save trained parameters of all agents into one file
        """
        self.prep_training(device='cpu')  # move parameters to CPU before saving
        save_dict = {'init_dict': self.init_dict,
                     'agent_params': [a.get_params() for a in self.agents]}
        torch.save(save_dict, filename)

    @classmethod
    def init_from_save(cls, filename):
        """
        Instantiate instance of this class from file created by 'save' method
        """
        save_dict = torch.load(filename, map_location=torch.device('cpu'))
        instance = cls(**save_dict['init_dict'])
        instance.init_dict = save_dict['init_dict']
        for a, params in zip(instance.agents, save_dict['agent_params']):
            a.load_params(params)
        return instance

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

class ReplayBuffer:
    """
    Replay Buffer for multi-agent RL with parallel rollouts
    """
    def __init__(self, max_steps, num_agents, obs_dims, ac_dims):
        """
        Inputs:
            max_steps (int): Maximum number of timepoints to store in buffer
            num_agents (int): Number of agents in environment
            obs_dims (list of ints): number of obervation dimensions for each
                                     agent
            ac_dims (list of ints): number of action dimensions for each agent
        """
        max_steps = int(max_steps)
        self.max_steps = max_steps
        self.num_agents = num_agents
        self.obs_buffs = []
        self.ac_buffs = []
        self.rew_buffs = []
        self.next_obs_buffs = []
        self.done_buffs = []
        for odim, adim in zip(obs_dims, ac_dims):
            self.obs_buffs.append(np.zeros((max_steps, odim)))
            self.ac_buffs.append(np.zeros((max_steps, adim)))
            self.rew_buffs.append(np.zeros(max_steps))
            self.next_obs_buffs.append(np.zeros((max_steps, odim)))
            self.done_buffs.append(np.zeros(max_steps))


        self.filled_i = 0  # index of first empty location in buffer (last index when full)
        self.curr_i = 0  # current index to write to (ovewrite oldest data)

    def __len__(self):
        return self.filled_i

    def push(self, observations, actions, rewards, next_observations, dones):
        nentries = 1  # single environment at a time
        if self.curr_i + nentries > self.max_steps:
            rollover = self.max_steps - self.curr_i # num of indices to roll over
            for agent_i in range(self.num_agents):
                self.obs_buffs[agent_i] = np.roll(self.obs_buffs[agent_i],
                                                  rollover, axis=0)
                self.ac_buffs[agent_i] = np.roll(self.ac_buffs[agent_i],
                                                 rollover, axis=0)
                self.rew_buffs[agent_i] = np.roll(self.rew_buffs[agent_i],
                                                  rollover)
                self.next_obs_buffs[agent_i] = np.roll(
                    self.next_obs_buffs[agent_i], rollover, axis=0)
                self.done_buffs[agent_i] = np.roll(self.done_buffs[agent_i],
                                                   rollover)
            self.curr_i = 0
            self.filled_i = self.max_steps
        for agent_i in range(self.num_agents):
            self.obs_buffs[agent_i][self.curr_i:self.curr_i + nentries] = observations[agent_i]
            # actions are already batched by agent, so they are indexed differently
            self.ac_buffs[agent_i][self.curr_i:self.curr_i + nentries] = actions[agent_i]
            self.rew_buffs[agent_i][self.curr_i:self.curr_i + nentries] = rewards[agent_i]
            self.next_obs_buffs[agent_i][self.curr_i:self.curr_i + nentries] = next_observations[agent_i]
            self.done_buffs[agent_i][self.curr_i:self.curr_i + nentries] = dones[agent_i]
        self.curr_i += nentries
        if self.filled_i < self.max_steps:
            self.filled_i += nentries
        if self.curr_i == self.max_steps:
            self.curr_i = 0

    def sample(self, N, device='cpu', norm_rews=False):
        inds = np.random.choice(np.arange(self.filled_i), size=N,
                                replace=False)
        cast = lambda x: torch.tensor(x, dtype=torch.float32).to(device)
        if norm_rews:
            ret_rews = [cast((self.rew_buffs[i][inds] -
                              self.rew_buffs[i][:self.filled_i].mean()) /
                             self.rew_buffs[i][:self.filled_i].std())
                        for i in range(self.num_agents)]
        else:
            ret_rews = [cast(self.rew_buffs[i][inds]) for i in range(self.num_agents)]
        return ([cast(self.obs_buffs[i][inds]) for i in range(self.num_agents)],
                [cast(self.ac_buffs[i][inds]) for i in range(self.num_agents)],
                ret_rews,
                [cast(self.next_obs_buffs[i][inds]) for i in range(self.num_agents)],
                [cast(self.done_buffs[i][inds]) for i in range(self.num_agents)])
    
## MISCILLANEOUS UTILITIES

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

def onehot_from_logits(logits):
    """
    Given batch of logits, return one-hot sample using epsilon greedy strategy
    (based on given epsilon)
    """
    # get best (according to current policy) actions in one-hot form
    argmax_acs = (logits == logits.max(1, keepdim=True)[0]).float()
    return argmax_acs