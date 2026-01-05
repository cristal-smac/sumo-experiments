# from sumo.tools.emissions.findMinDiffModel import model

from sumo_experiments.strategies import Strategy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
from sumo_experiments.strategies.maxpressure_strategy import MaxPressureStrategy
import matplotlib.pyplot as plt

loss_fn = nn.HuberLoss()


class IntellilightStrategy(Strategy):
    """
    Implements an Intellilight system for each intersection.
    Use a Double DQN algorithm to train the agent.

    Wei, H., Zheng, G., Yao, H., & Li, Z. (2018, July). Intellilight: A reinforcement learning approach for intelligent traffic light control. In Proceedings of the 24th ACM SIGKDD international conference on knowledge discovery & data mining (pp. 2496-2505).
    """
    DEBUG_REWARD = 10000  # ridiculously large value for debugging

    def __init__(self, network, period=10, reward_coeffs=(1, 1, 1, 1), gamma=0.99, episode_duration=300, batch_size=64, buffer_size=1000, update_target_frequency=10, learning_rate=1 * 10 ** -2, exploration_prob=1, cooling_rate=10 ** -3, hidden_layer_size=64, yellow_time=3):
        """
        Init of class.
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param period: The duration of a period (in seconds).
        :type period: int or dict
        :param gamma: Gamma parameter for the Bellman equation, to compute Q-Values
        :type gamma: float or dict
        :param episode_duration: The duration of an episode for training, in timestep
        :type episode_duration: int or dict
        :param buffer_size: Maximum memory buffer size for training the neural network.
        :type buffer_size: int or dict
        :param update_target_frequency: Frequency at which the target network is updated, in terms of number of trainings. The target network will be updated every ((buffer_size + len(buffer_size) * yellow_time) * update_target_frequency) timesteps.
        :type update_target_frequency: int or dict
        :param learning_rate: Learning rate for the neural network. Must be a positive number.
        :type learning_rate: float or dict
        :param exploration_prob: Probability of selecting a random action at the beginning of the simulation. Must be a positive number.
        :type exploration_prob: float or dict
        :param cooling_rate: Value used to update the exploration_prob. Each time an action is chosen, the next exploration_prob is (exploration_prob - (exploration_prob * cooling_rate)).
        :type cooling_rate: float or dict
        :param hidden_layer_size: The size of the hidden layers of the neural network.
        :type hidden_layer_size: int or dict
        :param yellow_time: Yellow phases duration for all intersections
        :type yellow_time: int or dict
        """
        super().__init__()
        self.network = network
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {identifiant: yellow_time for identifiant in network.TLS_DETECTORS}
        self.current_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.current_max_time_index = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.current_yellow_time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.started = False
        self.nb_phases = {}
        self.nb_switch = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.next_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        if type(period) is dict:
            self.period = period
        else:
            self.period = {identifiant: period for identifiant in network.TLS_DETECTORS}
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {identifiant: yellow_time for identifiant in network.TLS_DETECTORS}
        self.c1, self.c2, self.c3, self.c4 = reward_coeffs
        # self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.device = torch.device("cpu")
        self.action_space = {identifiant: list(self.network.TLS_DETECTORS[identifiant].keys()) for identifiant in self.network.TLS_DETECTORS}
        if type(gamma) is dict:
            self.gamma = gamma
        else:
            self.gamma = {identifiant: gamma for identifiant in network.TLS_DETECTORS}
        if type(episode_duration) is dict:
            self.episode_duration = episode_duration
        else:
            self.episode_duration = {identifiant: episode_duration for identifiant in network.TLS_DETECTORS}
        if type(batch_size) is dict:
            self.batch_size = batch_size
        else:
            self.batch_size = {identifiant: batch_size for identifiant in network.TLS_DETECTORS}
        if type(buffer_size) is dict:
            self.buffer_size = buffer_size
        else:
            self.buffer_size = {identifiant: buffer_size for identifiant in network.TLS_DETECTORS}
        self.replay_buffer = {identifiant: deque(maxlen=self.buffer_size[identifiant]) for identifiant in self.network.TLS_DETECTORS}
        if type(hidden_layer_size) is dict:
            self.hidden_layer_size = hidden_layer_size
        else:
            self.hidden_layer_size = {identifiant: hidden_layer_size for identifiant in network.TLS_DETECTORS}
        # self.model = {identifiant: QNetwork(action_space=self.action_space[identifiant], input_dim=4, hidden_dim=self.hidden_layer_size[identifiant], output_dim=len(self.action_space[identifiant])).to(self.device) for identifiant in self.network.TLS_DETECTORS}
        # self.target_model = {identifiant: QNetwork(action_space=self.action_space[identifiant], input_dim=4, hidden_dim=self.hidden_layer_size[identifiant], output_dim=len(self.action_space[identifiant])).to(self.device) for identifiant in self.network.TLS_DETECTORS}
        self.model = {identifiant: None for identifiant in self.network.TLS_DETECTORS}
        self.target_model = {identifiant: None for identifiant in self.network.TLS_DETECTORS}

        if type(update_target_frequency) is dict:
            self.update_target_frequency = update_target_frequency
        else:
            self.update_target_frequency = {identifiant: update_target_frequency for identifiant in network.TLS_DETECTORS}
        if type(learning_rate) is dict:
            self.learning_rate = learning_rate
        else:
            self.learning_rate = {identifiant: learning_rate for identifiant in network.TLS_DETECTORS}
        self.optimizer = {identifiant: None for identifiant in self.network.TLS_DETECTORS}
        self.loss_history = {identifiant: [] for identifiant in self.network.TLS_DETECTORS}
        self.current_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        if type(exploration_prob) is dict:
            self.exploration_prob = exploration_prob
        else:
            self.exploration_prob = {identifiant: exploration_prob for identifiant in network.TLS_DETECTORS}
        if type(cooling_rate) is dict:
            self.cooling_rate = cooling_rate
        else:
            self.cooling_rate = {identifiant: cooling_rate for identifiant in network.TLS_DETECTORS}
        self.number_of_trainings = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.last_state = {identifiant: None for identifiant in self.network.TLS_DETECTORS}
        self.last_action = {identifiant: None for identifiant in self.network.TLS_DETECTORS}

        self.mean_rewards = []
        self.mean_scores= []
        self.trainnn = {identifiant: True for identifiant in self.network.TLS_DETECTORS}
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
            self.started = True
        else:
            timestep = self.traci.simulation.getTime()
            tl_id = self.network.TL_IDS[0]
            if timestep % self.episode_duration[tl_id] == 0 and self.trainnn[tl_id]:
                self.mean_scores.append(np.mean(self.scores))
                self.scores = []
            for tl_id in self.network.TL_IDS:
                if timestep % self.episode_duration[tl_id] == 0 and self.trainnn[tl_id]:
                    if tl_id == "c":
                        self.mean_rewards.append(np.mean(self.rewards))
                        self.rewards = []
                        plt.plot(range(len(self.mean_rewards)), self.mean_rewards)
                        plt.xlabel("Episode")
                        plt.ylabel("Mean Reward")
                        plt.savefig('strategy_debug.png')
                    self.exploration_prob[tl_id] = self.exploration_prob[tl_id] - (self.exploration_prob[tl_id] * self.cooling_rate[tl_id])
                    self.last_state[tl_id] = None
                    self.last_action[tl_id] = None
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
                    if self.time[tl_id] > self.period[tl_id]:
                        self.switch_next_phase(tl_id)
                    else:
                        self.time[tl_id] += 1
                    self.current_phase_duration[tl_id] += 1
                if len(self.replay_buffer[tl_id]) >= self.batch_size[tl_id] and (timestep % (self.update_target_frequency[tl_id] * self.period[tl_id]) == 0):
                    self.train(tl_id)

    def get_phase_onehot(self, tl_id):
        keys = list(self.network.TLS_DETECTORS[tl_id].keys())
        one_hot = np.zeros(len(keys), dtype=np.bool)
        one_hot[keys.index(self.current_phase[tl_id])] = 1
        return one_hot

    def one_hot_to_phase(self, tl_id, one_hot):
        keys = list(self.network.TLS_DETECTORS[tl_id].keys())
        return keys[np.argmax(one_hot)]

    def switch_next_phase(self, tl_id):
        """
        Switch the traffic light id_tls to the next
        """
        self.nb_switch[tl_id] += 1
        next_action = self.get_next_action(tl_id)
        if next_action == 1:
            phases = list(self.network.TLS_DETECTORS[tl_id].keys())
            self.next_phase[tl_id] = phases[phases.index(self.traci.trafficlight.getPhase(tl_id)) + 1] if phases.index(self.traci.trafficlight.getPhase(tl_id)) + 1 != len(phases) else phases[0]
            if self.traci.trafficlight.getPhase(tl_id) == self.nb_phases[tl_id] - 1:
                self.traci.trafficlight.setPhase(tl_id, 0)
            else:
                self.traci.trafficlight.setPhase(tl_id, int(self.current_phase[tl_id] + 1))
            current_phase = self.traci.trafficlight.getPhase(tl_id)
            self.phases_durations[tl_id].append((current_phase, self.current_phase_duration[tl_id]))
            self.current_phase_duration[tl_id] = 0
        self.time[tl_id] = 0

    def get_next_action(self, tl_id, train=True):
        """
        Get the next phase for the controller.
        :param tl_id: The id of the traffic light
        :type tl_id: str
        :return: The next phase for the controller
        :rtype: int
        """
        state = self.get_state(tl_id)
        phase = int(state[-1])
        state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0).to(self.device)

        if train and random.random() < self.exploration_prob[tl_id]:
            action = random.choice([0, 1])
        else:
            with torch.no_grad():
                q_values = self.model[tl_id](state_tensor, [phase])
                action = torch.argmax(q_values).item()

        reward = self.get_reward(tl_id, change_phase=self.last_action[tl_id])  # rewards should be based on last action
        done = (self.traci.simulation.getTime() % 1000 == 0)

        if self.last_state[tl_id] is not None and self.last_action[tl_id] is not None and not done:
            #assert reward > -self.c4 * self.DEBUG_REWARD
            self.replay_buffer[tl_id].append((self.last_state[tl_id], self.last_action[tl_id], reward, state, phase, done))
            if tl_id == "c":  # debugging for single intersection
                self.rewards.append(reward)
                self.times.append(self.traci.simulation.getTime())
            self.scores = [self.get_score(tl_id, self.last_action[tl_id]) for i, tl_id in enumerate(self.network.TLS_DETECTORS)]
        self.last_state[tl_id] = state
        self.last_action[tl_id] = action

        return action

    def train(self, tl_id):
        """

        """
        self.model[tl_id].train()
        self.target_model[tl_id].eval()

        batch = self._get_homogeneous_memory(tl_id, min(len(self.replay_buffer[tl_id]), self.batch_size[tl_id]))
        states, actions, rewards, next_states, phases, dones = zip(*batch)

        states = torch.tensor(states, dtype=torch.float32).to(self.device)
        next_states = torch.tensor(next_states, dtype=torch.float32).to(self.device)
        actions = torch.tensor(actions).unsqueeze(1).to(self.device)
        rewards = torch.tensor(rewards, dtype=torch.float32).unsqueeze(1).to(self.device)
        dones = torch.tensor(dones, dtype=torch.float32).unsqueeze(1).to(self.device)

        # Create a mapping from action names to indices
        action_to_index = {action: idx for idx, action in enumerate(self.action_space[tl_id])}

        # Vectorized conversion of actions to indices
        # action_indices = torch.tensor([action_to_index[action[0].item()] for action in actions], dtype=torch.long).unsqueeze(1).to(self.device)
        # action_indices = torch.tensor(actions, dtype=torch.long).unsqueeze(1).to(self.device)
        # print(action_indices.size())

        # Compute target Q-values
        with torch.no_grad():
            # next_q_values = torch.stack([self.target_model[tl_id](next_states, phase) for phase in phases])  # [batch_size, output_dim]
            next_q_values = self.target_model[tl_id](states, phases)  # [batch_size, output_dim]
            max_next_q_values = next_q_values.max(dim=1, keepdim=True)[0]  # [batch_size, 1]
            targets = rewards + self.gamma[tl_id] * max_next_q_values * (1 - dones)  # [batch_size, 1]

        # Compute current Q-values
        # current_q_values = torch.stack([self.model[tl_id](states, phase) for phase in phases])  # [batch_size, output_dim]
        current_q_values = self.model[tl_id](states, phases)  # [batch_size, output_dim]
        current_q_values = current_q_values.gather(1, actions)  # [batch_size, 1]

        # Compute loss
        loss = loss_fn(current_q_values, targets)

        self.optimizer[tl_id].zero_grad()
        loss.backward()
        self.optimizer[tl_id].step()
        self.loss_history[tl_id].append(loss.item())
        self.number_of_trainings[tl_id] += 1

        if self.number_of_trainings[tl_id] == self.update_target_frequency[tl_id]:
            self.number_of_trainings[tl_id] = 0
            self.update_target_model(tl_id)

    def _get_homogeneous_memory(self, tl_id, max_len):
        """
        Using memory palace technique, generate a replay buffer with the same number
        of experiences for each couple (phase, action).
        :param tl_id: The id of the traffic light
        :type tl_id: str
        :param max_len: The maximum length of the returned batch. The batched is filled until one of the memory palace is empty.
        :type max_len: int
        :return: A batch respecting memory palaces constraints
        :rtype: np.Array
        """
        actions = [0, 1]
        phases = list(self.network.TLS_DETECTORS[tl_id].keys())
        memory_palaces = {}
        batch = []
        for i in actions:
            for j in phases:
                memory_palaces[(i, j)] = []
        m_len = max_len // len(memory_palaces)
        for exp in self.replay_buffer[tl_id]:
            action = int(exp[1])
            phase = int(exp[0][-1])
            memory_palaces[(action, phase)].append(exp)
        for key in memory_palaces:
            min_palace = min(len(memory_palaces[key]), self.buffer_size[tl_id]) # Handle case when memory palace is empty
            idx = [i for i in range(len(memory_palaces[key]))]
            rnd_idx = np.random.choice(idx, size=min(min_palace, m_len), replace=False)
            for i in rnd_idx:
                batch.append(memory_palaces[key][i])
        np.random.shuffle(batch)
        return batch

    def get_state(self, tl_id):
        detectors = self._detectors(tl_id)
        L = [self.traci.lanearea.getJamLengthVehicle(det) / 10 for det in detectors]
        W = [x / 20 for x in self.compute_waiting_time(detectors)]
        V = [x / 10 for x in self.compute_number_of_vehicles(tl_id)]
        P = [self.current_phase[tl_id]]
        return np.array(L + W + V + P, dtype=np.float32)  # concatenate all values into a single array

    def get_reward(self, tl_id, change_phase=None):
        detectors = self._detectors(tl_id)
        L = self.c1 * sum([self.traci.lanearea.getJamLengthVehicle(det) for det in detectors])
        D = self.c2 * sum([1 - (self.traci.lanearea.getIntervalMeanSpeed(det) / self.traci.lane.getMaxSpeed(self.traci.lanearea.getLaneID(det))) for det in detectors])
        W = self.c3 * sum(self.compute_waiting_time(detectors))
        A = self.c4 * (self.DEBUG_REWARD if change_phase is None else change_phase)  # if change_phase is None this should not be in replay buffer
        # Number of vehicles that passed intersection have to be implemented
        # Travel time of vehicles that passed the intersection have to be implemented
        reward = -(L + D + W + A)
        return reward

    def get_score(self, tl_id, change_phase=None):
        detectors = self._detectors(tl_id)
        L = self.c1 * sum([self.traci.lanearea.getJamLengthVehicle(det) for det in detectors])
        D = self.c2 * sum([1 - (self.traci.lanearea.getIntervalMeanSpeed(det) / self.traci.lane.getMaxSpeed(self.traci.lanearea.getLaneID(det))) for det in detectors])
        W = self.c3 * sum(self.compute_waiting_time(detectors))
        # Number of vehicles that passed intersection have to be implemented
        # Travel time of vehicles that passed the intersection have to be implemented
        return -(L + D + W)

    def update_target_model(self, tl_id):
        self.target_model[tl_id].load_state_dict(self.model[tl_id].state_dict())

    def compute_waiting_time(self, detectors):
        waiting_times = []
        for det in detectors:
            waiting_time = 0
            veh_ids = self.traci.lanearea.getLastStepVehicleIDs(det)
            for veh_id in veh_ids:
                waiting_time += self.traci.vehicle.getAccumulatedWaitingTime(veh_id)
            waiting_times.append(waiting_time)
        return waiting_times

    def compute_number_of_vehicles(self, tl_id):
        detectors = []
        for phase in self.network.TLS_DETECTORS[tl_id]:
            for det in self.network.TLS_DETECTORS[tl_id][phase]['numerical']:
                if det not in detectors:
                    detectors.append(det)
        return list(self.traci.lanearea.getLastStepVehicleNumber(det) for det in detectors)

    def _detectors(self, tl_id):
        detectors = []
        for phase in self.network.TLS_DETECTORS[tl_id]:
            for det in self.network.TLS_DETECTORS[tl_id][phase]['numerical']:
                if det not in detectors:
                    detectors.append(det)
        return detectors

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
        self.started = True

        input_dim = len(self.get_state(tl_id))
        if self.model[tl_id] is None: # no loaded model
            self.model[tl_id] = QNetwork(action_space=self.action_space[tl_id], input_dim=input_dim, hidden_dim=self.hidden_layer_size[tl_id], output_dim=2).to(self.device)
            self.target_model[tl_id] = QNetwork(action_space=self.action_space[tl_id], input_dim=input_dim, hidden_dim=self.hidden_layer_size[tl_id], output_dim=2).to(self.device)
        self.optimizer[tl_id] = optim.Adam(self.model[tl_id].parameters(), lr=self.learning_rate[tl_id])

    def save_model(self, filepath):
        torch.save(self.model, filepath)

    def load_model(self, filepath):
        torch.serialization.add_safe_globals([QNetwork, nn.Linear, nn.Sequential, nn.ReLU, nn.ModuleList, optim.Adam, dict])
        self.model = torch.load(filepath, weights_only=False)
        self.target_model = torch.load(filepath, weights_only=False)


class QNetwork(nn.Module):
    def __init__(self, action_space, input_dim, hidden_dim, output_dim):
        super().__init__()
        self.action_space = action_space
        self.convert_phase = {
            k: self.action_space.index(k) for k in self.action_space
        }
        self.shared = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU()
        )
        self.heads = nn.ModuleList([
            nn.Sequential(nn.Linear(hidden_dim, hidden_dim), nn.Linear(hidden_dim, output_dim)) for _ in range(len(action_space))
        ])

    def forward(self, x, phases):
        # if 0 <= abstract_phase < len(self.heads):
        try:
            h = self.shared(x)
            batch_size = x.size(0)
            # Collect the output of the appropriate head for each phase in the batch
            q_values = torch.zeros(batch_size, self.heads[0][-1].out_features, device=x.device)  # [batch_size, output_dim]
            for i, phase in enumerate(phases):
                q_values[i] = self.heads[self.convert_phase[phase]](h[i])

            return q_values
        except:
            raise ValueError(f"Invalid phase index: {phase}")
