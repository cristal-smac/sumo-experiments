#from sumo.tools.emissions.findMinDiffModel import model
from . import Strategy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
from .maxpressure_strategy import MaxPressureStrategy
from .intellilight_strategy import IntellilightStrategy
import matplotlib.pyplot as plt
import collections

from .rl_networks import *
from .rl_agents import *
from .rl_networks import *
from .rl_util import episode_reward_scale

import torch
import torch.distributed as dist
from typing import List
loss_fn = torch.nn.HuberLoss()


def _move_optimizer_state(optimizer, device):
    """Move tensor values inside an optimizer.state to `device`.

    This is needed when the model parameters are moved to a new device but
    the optimizer (created earlier) still has its state tensors on the old
    device (e.g. CPU). Call after moving modules to ensure optimizer state
    and parameters share the same device.
    """
    if optimizer is None:
        return

    tgt = device
    for state in optimizer.state.values():
        for k, v in list(state.items()):
            if torch.is_tensor(v):
                state[k] = v.to(tgt)



class MADDPGStrategy(IntellilightStrategy):
    """
    Implements an MADDPG system.

    References:
    - https://github.com/shariqiqbal2810/maddpg-pytorch/blob/master/algorithms/maddpg.py#L143
    - Lowe, R., Wu, Y. I., Tamar, A., Harb, J., Pieter Abbeel, O., & Mordatch, I. (2017). Multi-agent actor-critic for mixed cooperative-competitive environments. Advances in neural information processing systems, 30.
    """
    REWARD_REFERENCE_EPISODE_DURATION = 1000.0

    def __init__(self, network, period=10, episode_duration=1800, reward_coeffs=(1, 1, 1, 1), gamma=0.99, buffer_size=10000, batch_size=32, 
                 steps_per_update=10, samples_before_update=1024, 
                 learning_rate=1e-2, tau=0.01, hidden_layer_size=64, yellow_time=3, intelligent_intersections=None, shared_policy=False, recurrent_policy=True,
                 debug=True, simulation_time=500000, measure_energy=True):
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
        :param shared_policy: If True, identical agents share policy/target_policy networks, improving sample efficiency for homogeneous intersections.
        :type shared_policy: bool
        """
        Strategy.__init__(self, measure_energy=measure_energy)
        self.shared_policy = shared_policy
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

        # Try Metal (Apple Silicon), then CUDA, then CPU
        if torch.backends.mps.is_available():
            self.device = torch.device("mps")
        elif torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")
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

        if intelligent_intersections is None:
            self.intelligent_intersections = network.TL_IDS
        else:
            self.intelligent_intersections = intelligent_intersections

        self.debug = debug
        # Preallocate the loss telemetry as fixed numpy arrays sized from the run
        # length, so there is no unbounded growth or reallocation. train() runs at
        # most once every steps_per_update steps; each run records one critic loss
        # per agent, and one actor loss per agent (or one total when the policy is
        # shared). _val_i / _pol_i are write cursors; read the filled prefix via
        # val_losses[:_val_i] (e.g. for plotting).
        n_agents = len(self.intelligent_intersections)
        max_trainings = int(simulation_time) // max(int(steps_per_update), 1) + 1
        self.mean_scores = np.zeros(max_trainings, dtype=np.float32)
        self.val_losses = np.zeros(max_trainings * n_agents, dtype=np.float32)
        self.pol_losses = np.zeros(max_trainings * (1 if shared_policy else n_agents), dtype=np.float32)
        self._val_i = 0
        self._pol_i = 0
        self._mean_score_i = 0
        self.rewards = []
        self.scores = []
        self.times = []
        self.phases_occurences = {identifiant: {} for identifiant in network.TLS_DETECTORS}
        self.phases_durations = {identifiant: [] for identifiant in network.TLS_DETECTORS}
        self.current_phase_duration = {identifiant: 0 for identifiant in network.TLS_DETECTORS}
        self.recurrent_policy = recurrent_policy
        if self.recurrent_policy:
            self.__name__ = "MARDDPGStrategy"
        else:
            self.__name__ = "MADDPGStrategy"

    def reset_recurrent_states(self):
        # Called at every episode boundary (and on load). Finalize the current
        # trajectory into the sequence buffer before clearing the actors' hidden
        # state, so stored episodes align exactly with the rollout's hidden-state
        # reset. No-op for the flat buffer / non-recurrent agents.
        buf = getattr(self, 'replay_buffer', None)
        end_fn = getattr(buf, 'end_episode', None)
        if callable(end_fn):
            end_fn()
        if self.maddpg is None:
            return
        for agent in self.maddpg.agents:
            reset_fn = getattr(agent, 'reset_recurrent_state', None)
            if callable(reset_fn):
                reset_fn()

    def _recurrent_seq_len(self):
        """Max number of action decisions in one episode, so a sampled sequence
        can span a whole episode and be replayed from t=0 with zero hidden."""
        period = self.period
        if isinstance(period, dict):
            period = min(period.values())
        period = max(int(period), 1)
        duration = self.episode_duration
        if isinstance(duration, dict):
            duration = max(duration.values())
        return int(duration // period) + 2


    def run_all_agents(self, traci):
        """
        Process agents to make one action each.
        :param traci: The simulation Traci instance
        :type traci: Traci
        :return: Nothing
        """
        if not self.started:
            self.traci = traci
            for tl_id in self.intelligent_intersections:
                self._start_agent(tl_id)

            if self.maddpg is None: # not loading a saved model
                maddpg_cls = RecurrentMADDPG if self.recurrent_policy else MADDPG
                self.maddpg = maddpg_cls(agents=list(self.agents.values()), alg_types=['MADDPG' for _ in self.intelligent_intersections],
                                    gamma=self.gamma[tl_id],
                                    discrete_action=True,
                                    tau=self.tau)
            if self.shared_policy:
                self._share_policies()
            obs_dims = list(self.observation_sizes.values())
            ac_dims = [len(self.action_space[tl_id]) for tl_id in self.intelligent_intersections]
            if self.recurrent_policy:
                self.replay_buffer = RecurrentReplayBuffer(max_steps=self.buffer_size, num_agents=self.maddpg.nagents,
                                                           obs_dims=obs_dims, ac_dims=ac_dims,
                                                           seq_len=self._recurrent_seq_len())
            else:
                self.replay_buffer = ReplayBuffer(max_steps=self.buffer_size, num_agents=self.maddpg.nagents,
                                                  obs_dims=obs_dims, ac_dims=ac_dims)
            self.started = True

            # self.states = [self.get_state(tl_id) for tl_id in self.network.TL_IDS]
            # self.action_indices = [np.zeros_like(self.action_space[tl_id]) for tl_id in self.network.TL_IDS] # TODO: one hot encode current phase
            self.states = None
            self.action_indices = None
            self.changed_phase = [None for _ in self.intelligent_intersections]

            self.time_step = 1
        else:
            self.zeus_monitor.begin_window("all_agents")
            if self.traci.simulation.getTime() % self.episode_duration == 0:
                if self._mean_score_i < self.mean_scores.size:
                    self.mean_scores[self._mean_score_i] = np.mean(self.scores)
                    self._mean_score_i += 1
                self.scores = []
                # reset states and action
                self.states = None
                self.action_indices = None
                self.changed_phase = [None for _ in self.intelligent_intersections]
                if self.debug and self.network.TL_IDS and (self._mean_score_i % 10 == 0):
                    if not hasattr(self, '_debug_fig'):
                        self._debug_fig, self._debug_ax = plt.subplots()
                        self._debug_line, = self._debug_ax.plot([], [])
                        self._debug_ax.set_xlabel('Episode')
                        self._debug_ax.set_ylabel('Mean Score')
                    self._debug_line.set_data(range(self._mean_score_i), self.mean_scores[:self._mean_score_i])
                    self._debug_ax.relim()
                    self._debug_ax.autoscale_view()
                    self._debug_fig.savefig(f'strategy_debug_{self.__name__}.png')
            for tl_id in self.intelligent_intersections:
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
            results = self.zeus_monitor.end_window("all_agents")
            self.energy_consumption += self.get_energy_consumption(results)


    def switch_next_phase(self):
        """
        Queue switch of the traffic light id_tls to the next, set up the yellow phase if needed.
        """
        next_phases = self.get_next_phases()
        for tl_id in self.intelligent_intersections:
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
        """Base MADDPG action loop with recurrent-state resets at episode boundaries."""
        sim_time = int(self.traci.simulation.getTime())
        next_states = [self.get_state(tl_id) for tl_id in self.intelligent_intersections]
        rewards = [self.get_reward(tl_id, self.changed_phase[i]) for i, tl_id in enumerate(self.intelligent_intersections)]
        dones = [(sim_time % self.episode_duration) == 0 for _ in self.intelligent_intersections]
        scores = [self.get_score(tl_id, self.changed_phase[i]) for i, tl_id in enumerate(self.intelligent_intersections)]

        # Preserve per-agent reward signals (do not replace with the mean)
        # rewards = np.array(rewards, dtype=np.float32)
        rewards = np.ones_like(rewards, dtype=np.float32) * np.mean(rewards)
        
        if self.states is not None and self.action_indices is not None and not dones[0]:
            self.replay_buffer.push(self.states, self.action_indices, rewards, next_states, dones=dones)
            if "c" in self.network.TL_IDS:
                assert rewards[0] > -(self.c4 * self.DEBUG_REWARD), (rewards, self.changed_phase)
                self.rewards.append(rewards[0])
                self.times.append(sim_time)
            self.scores.append(np.mean(scores))

        if dones[0]:
            self.reset_recurrent_states()

        state_tensor = [torch.as_tensor(n_s, dtype=torch.float32, device=self.rollout_device).unsqueeze(0) for n_s in next_states]
        with torch.no_grad():
            torch_action_indices = self.maddpg.step(state_tensor, explore=True)
            action_indices = [ac.data.cpu().numpy() for ac in torch_action_indices]

        if self.action_indices is None:
            self.changed_phase = [False for _ in self.intelligent_intersections]
        else:
            self.changed_phase = [
                np.argmax(action_indices[i]) != np.argmax(self.action_indices[i])
                for i, tl_id in enumerate(self.intelligent_intersections)
            ]
        self.states = next_states
        self.action_indices = action_indices

        return {tl_id: self.action_space[tl_id][action_indices[i].argmax()] for i, tl_id in enumerate(self.intelligent_intersections)}

    def train(self):
        # Train the MADDPG model once buffer reaches minimum size
        # self.replay_buffer.update_reward_statistics() # do once for all 
        self.maddpg.prep_training(device=self.device)
        for tl_id in self.intelligent_intersections:
            self.number_of_trainings[tl_id] += 1
        
        sample = self.replay_buffer.sample(self.batch_size,
                            device=self.device, norm_rews=False)
        if self.shared_policy:
            # Phase 1: Update all critics independently
            for a_i in range(self.maddpg.nagents):

                val_loss = self.maddpg.update_critic(sample, a_i)
                if self._val_i < self.val_losses.size:
                    self.val_losses[self._val_i] = val_loss
                    self._val_i += 1
            # Phase 2: Accumulate policy gradients from all critics, step once
            pol_loss = self.maddpg.update_shared_policy(sample)
            if self._pol_i < self.pol_losses.size:
                self.pol_losses[self._pol_i] = pol_loss
                self._pol_i += 1
        else:
            for a_i in range(self.maddpg.nagents):
                val_loss, pol_loss = self.maddpg.update(sample, a_i)
                if self._val_i < self.val_losses.size:
                    self.val_losses[self._val_i] = val_loss
                    self._val_i += 1
                if self._pol_i < self.pol_losses.size:
                    self.pol_losses[self._pol_i] = pol_loss
                    self._pol_i += 1
        self.maddpg.update_all_targets()
        self.maddpg.prep_rollouts(device=self.rollout_device)

    def get_reward(self, tl_id, change_phase=None):
        pressure = MaxPressureStrategy._compute_pressure(self, self.network.TLS_DETECTORS[tl_id])
        base_reward = -np.nanmean(list(pressure.values())) / 2000
        scale = episode_reward_scale(
            self.episode_duration,
            tl_id=tl_id,
            reference_duration=self.REWARD_REFERENCE_EPISODE_DURATION,
        )
        return base_reward * scale

    def _start_agent(self, tl_id):
        """Create recurrent MADDPG agents while keeping base strategy behavior."""
        #from src.sumo_experiments.strategies import maddpg_strategy as maddpg_module

        self.nb_phases[tl_id] = len(self.traci.trafficlight.getAllProgramLogics(tl_id)[0].phases)
        tl_logic = self.traci.trafficlight.getAllProgramLogics(tl_id)[0]
        for phase in tl_logic.phases:
            phase.duration = 10000
            phase.maxDur = 10000
            phase.minDur = 10000
        self.traci.trafficlight.setProgramLogic(tl_id, tl_logic)
        self.traci.trafficlight.setPhase(tl_id, 0)
        self.traci.trafficlight.setPhaseDuration(tl_id, 10000)

        input_dims = {tls_id: len(self.get_state(tls_id)) for tls_id in self.intelligent_intersections}
        critic_dim = sum(input_dims.values())
        critic_dim += sum(len(self.action_space[tls_id]) for tls_id in self.intelligent_intersections)

        if self.recurrent_policy:
            agent = RecurrentDDPGAgent(
                num_out_pol=len(self.action_space[tl_id]),
                num_in_pol=input_dims[tl_id],
                num_in_critic=critic_dim,
                hidden_dim=self.hidden_layer_size[tl_id],
                lr=self.learning_rate[tl_id],
                discrete_action=True,
            )
        else:
            agent = DDPGAgent(
                num_out_pol=len(self.action_space[tl_id]),
                num_in_pol=input_dims[tl_id],
                num_in_critic=critic_dim,
                hidden_dim=self.hidden_layer_size[tl_id],
                lr=self.learning_rate[tl_id],
                discrete_action=True,
            )

        self.agents[tl_id] = agent.to(self.rollout_device)
        self.observation_sizes[tl_id] = input_dims[tl_id]

    def _share_policies(self):
        """
        Make agents with identical architectures share the same policy,
        target_policy, and policy_optimizer. Each agent keeps its own critic.
        Operates on self.maddpg.agents after MADDPG is initialized.
        """
        agents = self.maddpg.agents
        if len(agents) <= 1:
            return
        primary = agents[0]
        for agent in agents[1:]:
            if (agent.policy.fc1.in_features == primary.policy.fc1.in_features and
                agent.policy.fc3.out_features == primary.policy.fc3.out_features and
                agent.policy.fc1.out_features == primary.policy.fc1.out_features):
                agent.policy = primary.policy
                agent.target_policy = primary.target_policy
                agent.policy_optimizer = primary.policy_optimizer

    def save_model(self, filepath):
        torch.save(self.maddpg.agents, filepath)

    def load_model(self, filepath):
        torch.serialization.add_safe_globals([
            DDPGAgent,
            DeepNN,
            RecurrentDDPGAgent,
            RecurrentPolicyNetwork,
            nn.Linear,
            nn.LSTM,
            nn.functional.relu,
            optim.Adam,
            collections.defaultdict,
            dict,
        ])
        if self.maddpg is None:
            maddpg_cls = RecurrentMADDPG if self.recurrent_policy else MADDPG
            self.maddpg = maddpg_cls(agents=[], alg_types=['MADDPG' for _ in self.intelligent_intersections],
                                 gamma=list(self.gamma.values())[0], # TODO: refactor?
                                 discrete_action=True,
                                 tau=self.tau)
        self.maddpg.agents = torch.load(filepath, map_location=self.rollout_device, weights_only=False)
        self.recurrent_policy = any(getattr(agent, 'recurrent_policy', False) for agent in self.maddpg.agents)
        self.reset_recurrent_states()


    

## MADDPG Algorithm


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

    def update_critic(self, sample, agent_i):
        """
        Update only the critic for agent_i (used with shared policies).
        """
        obs, acs, rews, next_obs, dones = sample
        curr_agent = self.agents[agent_i]
        curr_agent.critic_optimizer.zero_grad()
        if self.discrete_action:
            all_trgt_acs = [nn.functional.gumbel_softmax(pi(nobs), hard=True) for pi, nobs in
                            zip(self.target_policies, next_obs)]
        else:
            all_trgt_acs = [pi(nobs) for pi, nobs in zip(self.target_policies, next_obs)]
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
        return vf_loss.cpu().detach().numpy()

    def update_shared_policy(self, sample):
        """
        Accumulate policy gradients from all agents' critics into the shared
        policy, then step once.  Each critic provides a different gradient
        signal, giving the shared policy diverse learning signal.
        """
        obs, acs, rews, next_obs, dones = sample
        # All agents share the same policy optimizer
        self.agents[0].policy_optimizer.zero_grad()

        total_pol_loss = torch.tensor(0.0, device=obs[0].device)
        for agent_i in range(self.nagents):
            curr_agent = self.agents[agent_i]
            if self.discrete_action:
                curr_pol_out = curr_agent.policy(obs[agent_i])
                curr_pol_vf_in = nn.functional.gumbel_softmax(curr_pol_out, hard=False)
            else:
                curr_pol_out = curr_agent.policy(obs[agent_i])
                curr_pol_vf_in = curr_pol_out

            all_pol_acs = []
            for i, pi, ob in zip(range(self.nagents), self.policies, obs):
                if i == agent_i:
                    all_pol_acs.append(curr_pol_vf_in)
                else:
                    all_pol_acs.append(onehot_from_logits(pi(ob)))

            vf_in = torch.cat((*obs, *all_pol_acs), dim=1)
            pol_loss = -curr_agent.critic(vf_in).mean()
            pol_loss += (curr_pol_out**2).mean() * 1e-3
            total_pol_loss = total_pol_loss + pol_loss

        avg_pol_loss = total_pol_loss / self.nagents
        avg_pol_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.agents[0].policy.parameters(), 0.5)
        self.agents[0].policy_optimizer.step()
        return avg_pol_loss.cpu().detach().numpy()

    def update_all_targets(self):
        """
        Update all target networks (called after normal updates have been
        performed for each agent)
        """
        seen_policies = set()
        for a in self.agents:
            soft_update(a.target_critic, a.critic, self.tau)
            pol_id = id(a.policy)
            if pol_id not in seen_policies:
                soft_update(a.target_policy, a.policy, self.tau)
                seen_policies.add(pol_id)
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
        # Ensure optimizer state tensors are on the same device as the
        # corresponding module parameters. This handles checkpoints loaded on
        # CPU that are later trained on GPU/MPS.
        for a in self.agents:
            _move_optimizer_state(getattr(a, 'policy_optimizer', None), device)
            _move_optimizer_state(getattr(a, 'critic_optimizer', None), device)


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

class ReplayBuffer:
    """
    Optimized Multi-Agent Replay Buffer using a circular pointer and 
    epoch-cached reward statistics to avoid whole-buffer recalculation overhead.
    """
    def __init__(self, max_steps, num_agents, obs_dims, ac_dims):
        self.max_steps = int(max_steps)
        self.num_agents = num_agents
        
        self.obs_buffs = []
        self.ac_buffs = []
        self.rew_buffs = []
        self.next_obs_buffs = []
        self.done_buffs = []

        # Allocate contiguous arrays upfront using specified dtypes
        for odim, adim in zip(obs_dims, ac_dims):
            self.obs_buffs.append(np.zeros((self.max_steps, odim), dtype=np.float32))
            self.ac_buffs.append(np.zeros((self.max_steps, adim), dtype=np.float32))
            self.rew_buffs.append(np.zeros(self.max_steps, dtype=np.float32))
            self.next_obs_buffs.append(np.zeros((self.max_steps, odim), dtype=np.float32))
            self.done_buffs.append(np.zeros(self.max_steps, dtype=np.float32))

        self.filled_i = 0  # Number of valid slots currently in the buffer
        self.curr_i = 0    # Circular pointer tracking where to write next

        # Lazy epoch caching for O(1) batch normalization
        self._cached_means = np.zeros(num_agents, dtype=np.float32)
        self._cached_stds = np.ones(num_agents, dtype=np.float32)

    def __len__(self):
        return self.filled_i

    def push(self, observations, actions, rewards, next_observations, dones):
        """
        Inserts data at the circular pointer in O(1) time without shifting arrays.
        """
        idx = self.curr_i
        
        for agent_i in range(self.num_agents):
            self.obs_buffs[agent_i][idx] = observations[agent_i]
            self.ac_buffs[agent_i][idx] = actions[agent_i]
            self.rew_buffs[agent_i][idx] = rewards[agent_i]
            self.next_obs_buffs[agent_i][idx] = next_observations[agent_i]
            self.done_buffs[agent_i][idx] = dones[agent_i]

        # Advance pointer circularly
        self.curr_i = (idx + 1) % self.max_steps
        if self.filled_i < self.max_steps:
            self.filled_i += 1

    def update_reward_statistics(self):
        """
        Call this ONCE right before your optimization loop/epoch begins. 
        Caches the true population stats across active elements, ensuring O(1) sampling.
        """
        if self.filled_i == 0:
            return
            
        for i in range(self.num_agents):
            valid_rews = self.rew_buffs[i][:self.filled_i]
            self._cached_means[i] = valid_rews.mean()
            # Guard against zero variance on step 1 or flat rewards
            std = valid_rews.std()
            self._cached_stds[i] = std if std > 1e-8 else 1.0

    def sample(self, N, device='cpu', norm_rews=False):
        """
        Fast uniform sampling using zero-copy tensor wrapping and cached metrics.
        """
        if self.filled_i == 0:
            raise ValueError("Cannot sample from an empty replay buffer.")

        # Vectorized generation of unique indices
        inds = np.random.choice(self.filled_i, size=min(N, self.filled_i), replace=False)

        # Zero-copy tensor utility
        def cast(arr):
            return torch.as_tensor(arr, dtype=torch.float32, device=device)

        if norm_rews:
            ret_rews = [
                (cast(self.rew_buffs[i][inds]) - self._cached_means[i]) / self._cached_stds[i]
                for i in range(self.num_agents)
            ]
        else:
            ret_rews = [cast(self.rew_buffs[i][inds]) for i in range(self.num_agents)]
        return ([cast(self.obs_buffs[i][inds]) for i in range(self.num_agents)],
                [cast(self.ac_buffs[i][inds]) for i in range(self.num_agents)],
                ret_rews,
                [cast(self.next_obs_buffs[i][inds]) for i in range(self.num_agents)],
                [cast(self.done_buffs[i][inds]) for i in range(self.num_agents)])


class RecurrentReplayBuffer:
    """Episode/sequence replay buffer for recurrent MADDPG (BPTT).

    Unlike the flat ``ReplayBuffer``, this stores whole episodes so the recurrent
    actor can be trained on ordered sequences with the hidden state threaded
    through time. Sequences are always replayed from the *start* of an episode
    with a zero initial hidden state, which exactly matches the rollout
    convention (hidden state reset at every episode boundary). This eliminates
    the train/rollout hidden-state mismatch that makes a single-transition buffer
    incompatible with a recurrent policy.

    API is a drop-in superset of ``ReplayBuffer``: ``push`` / ``__len__`` /
    ``update_reward_statistics`` / ``sample``, plus ``end_episode`` which the
    strategy calls at every episode boundary to finalize the current trajectory.
    ``sample`` returns a 6-tuple ``(obs, acs, rews, next_obs, dones, mask)`` where
    obs/acs/next_obs are ``(batch, seq_len, dim)``, rews/dones are
    ``(batch, seq_len)`` and ``mask`` is ``(batch, seq_len)`` marking valid steps.
    """

    def __init__(self, max_steps, num_agents, obs_dims, ac_dims, seq_len):
        self.max_steps = int(max_steps)
        self.num_agents = num_agents
        self.obs_dims = list(obs_dims)
        self.ac_dims = list(ac_dims)
        self.seq_len = int(seq_len)

        self.episodes = []   # list of finalized episode dicts
        self._cur = None     # in-progress episode (per-agent python lists)
        self._total = 0      # number of stored steps across finalized episodes

        self._cached_means = np.zeros(num_agents, dtype=np.float32)
        self._cached_stds = np.ones(num_agents, dtype=np.float32)

    def __len__(self):
        # Only finalized (sampleable) steps count, so the strategy's
        # samples_before_update gate waits for complete episodes.
        return self._total

    def _blank_episode(self):
        return {
            'obs': [[] for _ in range(self.num_agents)],
            'acs': [[] for _ in range(self.num_agents)],
            'rews': [[] for _ in range(self.num_agents)],
            'next_obs': [[] for _ in range(self.num_agents)],
            'dones': [[] for _ in range(self.num_agents)],
        }

    def push(self, observations, actions, rewards, next_observations, dones):
        if self._cur is None:
            self._cur = self._blank_episode()
        for i in range(self.num_agents):
            self._cur['obs'][i].append(np.asarray(observations[i], dtype=np.float32).reshape(-1))
            self._cur['acs'][i].append(np.asarray(actions[i], dtype=np.float32).reshape(-1))
            self._cur['rews'][i].append(float(np.asarray(rewards[i]).reshape(-1)[0]))
            self._cur['next_obs'][i].append(np.asarray(next_observations[i], dtype=np.float32).reshape(-1))
            self._cur['dones'][i].append(float(np.asarray(dones[i]).reshape(-1)[0]))

    def end_episode(self):
        """Finalize the in-progress trajectory into a stored episode."""
        if self._cur is None:
            return
        ep_len = len(self._cur['rews'][0]) if self.num_agents else 0
        if ep_len == 0:
            self._cur = None
            return
        episode = {
            'obs': [np.asarray(self._cur['obs'][i], dtype=np.float32) for i in range(self.num_agents)],
            'acs': [np.asarray(self._cur['acs'][i], dtype=np.float32) for i in range(self.num_agents)],
            'rews': [np.asarray(self._cur['rews'][i], dtype=np.float32) for i in range(self.num_agents)],
            'next_obs': [np.asarray(self._cur['next_obs'][i], dtype=np.float32) for i in range(self.num_agents)],
            'dones': [np.asarray(self._cur['dones'][i], dtype=np.float32) for i in range(self.num_agents)],
            'len': ep_len,
        }
        self.episodes.append(episode)
        self._total += ep_len
        self._cur = None
        # Evict oldest episodes once capacity is exceeded (always keep >= 1).
        while self._total > self.max_steps and len(self.episodes) > 1:
            old = self.episodes.pop(0)
            self._total -= old['len']

    def update_reward_statistics(self):
        if not self.episodes:
            return
        for i in range(self.num_agents):
            all_rews = np.concatenate([ep['rews'][i] for ep in self.episodes])
            self._cached_means[i] = all_rews.mean()
            std = all_rews.std()
            self._cached_stds[i] = std if std > 1e-8 else 1.0

    def sample(self, N, device='cpu', norm_rews=False):
        if not self.episodes:
            raise ValueError("Cannot sample from an empty recurrent replay buffer.")
        T = self.seq_len
        B = int(N)
        na = self.num_agents

        obs = [np.zeros((B, T, self.obs_dims[i]), dtype=np.float32) for i in range(na)]
        acs = [np.zeros((B, T, self.ac_dims[i]), dtype=np.float32) for i in range(na)]
        next_obs = [np.zeros((B, T, self.obs_dims[i]), dtype=np.float32) for i in range(na)]
        rews = [np.zeros((B, T), dtype=np.float32) for i in range(na)]
        dones = [np.zeros((B, T), dtype=np.float32) for i in range(na)]
        mask = np.zeros((B, T), dtype=np.float32)

        ep_idx = np.random.randint(0, len(self.episodes), size=B)
        for b, ei in enumerate(ep_idx):
            ep = self.episodes[ei]
            length = min(ep['len'], T)  # replay from episode start; pad tail if shorter
            for i in range(na):
                obs[i][b, :length] = ep['obs'][i][:length]
                acs[i][b, :length] = ep['acs'][i][:length]
                next_obs[i][b, :length] = ep['next_obs'][i][:length]
                if norm_rews:
                    rews[i][b, :length] = (ep['rews'][i][:length] - self._cached_means[i]) / self._cached_stds[i]
                else:
                    rews[i][b, :length] = ep['rews'][i][:length]
                dones[i][b, :length] = ep['dones'][i][:length]
            mask[b, :length] = 1.0

        def cast(a):
            return torch.as_tensor(a, dtype=torch.float32, device=device)

        return ([cast(o) for o in obs], [cast(a) for a in acs],
                [cast(r) for r in rews], [cast(n) for n in next_obs],
                [cast(d) for d in dones], cast(mask))


class RecurrentMADDPG(MADDPG):
    """MADDPG variant that trains recurrent actors with BPTT over stored episode
    sequences sampled from a ``RecurrentReplayBuffer``.

    The actor (``RecurrentPolicyNetwork``) is unrolled over the whole sequence via
    ``forward_sequence`` so gradients flow through the recurrence. The critic is
    treated as feed-forward and evaluated per-timestep on the flattened
    ``(batch * seq_len)`` joint observations/actions. Padded timesteps are removed
    from every loss through the per-step ``mask``.
    """

    @staticmethod
    def _flat(x):
        # (B, T, D) -> (B*T, D); (B, T) must be unsqueezed by the caller first.
        return x.reshape(-1, x.shape[-1])

    def _policy_seq(self, policy, obs_seq):
        """Per-timestep logits (B, T, out) for a (recurrent) policy over a seq."""
        fn = getattr(policy, 'forward_sequence', None)
        if fn is not None:
            return fn(obs_seq)
        return policy(obs_seq)  # fallback: a feed-forward policy handles (B,T,·)

    def _target_actions(self, next_obs):
        if self.discrete_action:
            return [nn.functional.gumbel_softmax(self._flat(self._policy_seq(pi, nobs)), hard=True)
                    for pi, nobs in zip(self.target_policies, next_obs)]
        return [self._flat(self._policy_seq(pi, nobs))
                for pi, nobs in zip(self.target_policies, next_obs)]

    def _critic_target(self, sample, agent_i):
        """Masked Huber critic loss for agent_i (shared by update/update_critic)."""
        obs, acs, rews, next_obs, dones, mask = sample
        curr_agent = self.agents[agent_i]
        mask_flat = mask.reshape(-1, 1)
        denom = mask_flat.sum().clamp(min=1.0)

        all_trgt_acs = self._target_actions(next_obs)
        trgt_vf_in = torch.cat((*[self._flat(n) for n in next_obs], *all_trgt_acs), dim=1)
        target_value = (self._flat(rews[agent_i].unsqueeze(-1)) + self.gamma *
                        curr_agent.target_critic(trgt_vf_in) *
                        (1 - self._flat(dones[agent_i].unsqueeze(-1))))
        vf_in = torch.cat((*[self._flat(o) for o in obs], *[self._flat(a) for a in acs]), dim=1)
        actual_value = curr_agent.critic(vf_in)
        per_step = nn.functional.huber_loss(actual_value, target_value.detach(), reduction='none')
        return (per_step * mask_flat).sum() / denom

    def _policy_objective(self, sample, agent_i):
        """Masked deterministic policy-gradient loss for agent_i (BPTT actor)."""
        obs, acs, rews, next_obs, dones, mask = sample
        curr_agent = self.agents[agent_i]
        mask_flat = mask.reshape(-1, 1)
        denom = mask_flat.sum().clamp(min=1.0)

        curr_pol_out = self._flat(self._policy_seq(curr_agent.policy, obs[agent_i]))
        if self.discrete_action:
            curr_pol_vf_in = nn.functional.gumbel_softmax(curr_pol_out, hard=False)
        else:
            curr_pol_vf_in = curr_pol_out

        all_pol_acs = []
        for i, pi, ob in zip(range(self.nagents), self.policies, obs):
            if i == agent_i:
                all_pol_acs.append(curr_pol_vf_in)
            else:  # other agents' actions are fixed (no grad) one-hot encodings
                all_pol_acs.append(onehot_from_logits(self._flat(self._policy_seq(pi, ob))))

        vf_in = torch.cat((*[self._flat(o) for o in obs], *all_pol_acs), dim=1)
        q = curr_agent.critic(vf_in)
        pol_loss = -(q * mask_flat).sum() / denom
        pol_loss = pol_loss + ((curr_pol_out ** 2) * mask_flat).sum() / denom * 1e-3
        return pol_loss

    def update(self, sample, agent_i, logger=None):
        curr_agent = self.agents[agent_i]

        # --- critic ---
        curr_agent.critic_optimizer.zero_grad()
        vf_loss = self._critic_target(sample, agent_i)
        vf_loss.backward()
        torch.nn.utils.clip_grad_norm_(curr_agent.critic.parameters(), 0.5)
        curr_agent.critic_optimizer.step()

        # --- actor (BPTT through the recurrence) ---
        curr_agent.policy_optimizer.zero_grad()
        pol_loss = self._policy_objective(sample, agent_i)
        pol_loss.backward()
        torch.nn.utils.clip_grad_norm_(curr_agent.policy.parameters(), 0.5)
        curr_agent.policy_optimizer.step()

        if logger is not None:
            logger.add_scalars('agent%i/losses' % agent_i,
                               {'vf_loss': vf_loss, 'pol_loss': pol_loss}, self.niter)
        return vf_loss.cpu().detach().numpy(), pol_loss.cpu().detach().numpy()

    def update_critic(self, sample, agent_i):
        curr_agent = self.agents[agent_i]
        curr_agent.critic_optimizer.zero_grad()
        vf_loss = self._critic_target(sample, agent_i)
        vf_loss.backward()
        torch.nn.utils.clip_grad_norm_(curr_agent.critic.parameters(), 0.5)
        curr_agent.critic_optimizer.step()
        return vf_loss.cpu().detach().numpy()

    def update_shared_policy(self, sample):
        mask = sample[5]
        self.agents[0].policy_optimizer.zero_grad()
        total = torch.zeros((), device=mask.device)
        for agent_i in range(self.nagents):
            total = total + self._policy_objective(sample, agent_i)
        avg = total / self.nagents
        avg.backward()
        torch.nn.utils.clip_grad_norm_(self.agents[0].policy.parameters(), 0.5)
        self.agents[0].policy_optimizer.step()
        return avg.cpu().detach().numpy()


## MISCILLANEOUS UTILITIES

def onehot_from_logits(logits):
    """
    Given batch of logits, return one-hot sample using epsilon greedy strategy
    (based on given epsilon)
    """
    # get best (according to current policy) actions in one-hot form
    argmax_acs = (logits == logits.max(1, keepdim=True)[0]).float()
    return argmax_acs