#from sumo.tools.emissions.findMinDiffModel import model

from sumo_experiments.strategies import Strategy
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random

import matplotlib.pyplot as plt


class IntellilightStrategy(Strategy):
    """
    Implements an Intellilight system for each intersection.
    Use a Double DQN algorithm to train the agent.

    Wei, H., Zheng, G., Yao, H., & Li, Z. (2018, July). Intellilight: A reinforcement learning approach for intelligent traffic light control. In Proceedings of the 24th ACM SIGKDD international conference on knowledge discovery & data mining (pp. 2496-2505).
    """

    def __init__(self, network, periods, gamma=0.99, buffer_size=32, update_target_frequency=10, learning_rate=1*10**-2, exploration_prob=1, cooling_rate=10**-3, hidden_layer_size=64, yellow_time=3):
        """
        Init of class.
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param periods: The duration of a period (in seconds).
        :type periods: dict
        :param gamma: Gamma parameter for the Bellman equation, to compute Q-Values
        :type gamma: float
        :param buffer_size: Memory buffer size for training the neural network. The network is trained when th memory buffer is full.
        :type buffer_size: int
        :param update_target_frequency: Frequency at which the target network is updated, in terms of number of trainings. The target network will be updated every ((buffer_size + len(buffer_size) * yellow_time) * update_target_frequency) timesteps.
        :type update_target_frequency: int
        :param learning_rate: Learning rate for the neural network. Must be a positive number.
        :type learning_rate: float
        :param exploration_prob: Probability of selecting a random action at the beginning of the simulation. Must be a positive number.
        :type exploration_prob: float
        :param cooling_rate: Value used to update the exploration_prob. Each time an action is chosen, the next exploration_prob is (exploration_prob - (exploration_prob * cooling_rate)).
        :type cooling_rate: float
        :param hidden_layer_size: The size of the hidden layers of the neural network.
        :param yellow_time: Yellow phases duration for all intersections
        :type yellow_time: int
        """
        super().__init__()
        self.network = network
        self.yellow_time = yellow_time
        self.current_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.current_max_time_index = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.current_yellow_time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.started = False
        self.nb_phases = {}
        self.nb_switch = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.next_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.period = periods

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.action_space = {identifiant: list(self.network.TLS_DETECTORS[identifiant].keys()) for identifiant in self.network.TLS_DETECTORS}
        self.gamma = {identifiant: gamma for identifiant in self.network.TLS_DETECTORS}
        self.replay_buffer = {identifiant: deque() for identifiant in self.network.TLS_DETECTORS}
        self.buffer_size = {identifiant: buffer_size for identifiant in self.network.TLS_DETECTORS}
        self.hidden_layer_size = {identifiant: hidden_layer_size for identifiant in self.network.TLS_DETECTORS}
        self.model = {identifiant: QNetwork(action_space=self.action_space[identifiant], input_dim=4, hidden_dim=self.hidden_layer_size[identifiant], output_dim=len(self.action_space[identifiant])).to(self.device) for identifiant in self.network.TLS_DETECTORS}
        self.target_model = {identifiant: QNetwork(action_space=self.action_space[identifiant], input_dim=4, hidden_dim=self.hidden_layer_size[identifiant], output_dim=len(self.action_space[identifiant])).to(self.device) for identifiant in self.network.TLS_DETECTORS}
        self.update_target_frequency = {identifiant: update_target_frequency for identifiant in self.network.TLS_DETECTORS}
        self.optimizer = {identifiant: optim.Adam(self.model[identifiant].parameters(), lr=learning_rate) for identifiant in self.network.TLS_DETECTORS}
        self.loss_history = {identifiant: [] for identifiant in self.network.TLS_DETECTORS}
        self.current_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.exploration_prob = {identifiant: exploration_prob for identifiant in self.network.TLS_DETECTORS}  # Initial exploration probability
        self.cooling_rate = {identifiant: cooling_rate for identifiant in self.network.TLS_DETECTORS}  # Cooling rate for exploration probability
        self.number_of_trainings = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}

        self.rewards = []
        self.times = []


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
            for tl_id in self.network.TL_IDS:
                if 'y' in self.traci.trafficlight.getRedYellowGreenState(tl_id):
                    if self.current_yellow_time[tl_id] >= self.yellow_time:
                        self.traci.trafficlight.setPhase(tl_id, int(self.next_phase[tl_id]))
                        self.current_phase[tl_id] = self.next_phase[tl_id]
                        self.current_yellow_time[tl_id] = 0
                    else:
                        self.current_yellow_time[tl_id] += 1
                else:
                    if self.time[tl_id] > self.period[tl_id]:
                        self.switch_next_phase(tl_id)
                    else:
                        self.time[tl_id] += 1


    def switch_next_phase(self, tl_id):
        """
        Switch the traffic light id_tls to the next
        """
        self.nb_switch[tl_id] += 1
        #current_phase = self.traci.trafficlight.getPhase(id_tls)
        next_phase = self.get_next_phase(tl_id)
        if next_phase != self.current_phase[tl_id]:
            self.next_phase[tl_id] = next_phase
            if self.traci.trafficlight.getPhase(tl_id) == self.nb_phases[tl_id] - 1:
                self.traci.trafficlight.setPhase(tl_id, 0)
            else:
                self.traci.trafficlight.setPhase(tl_id, int(self.current_phase[tl_id] + 1))
        self.time[tl_id] = 0


    def get_next_phase(self, tl_id, train=True):
        """
        Get the next phase for the controller.
        :param tl_id: The id of the traffic light
        :type tl_id: str
        :return: The next phase for the controller
        :rtype: int
        """
        state = self.get_state(tl_id)
        phase = int(state[3])
        state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0).to(self.device)

        if train and random.random() < self.exploration_prob[tl_id]:
            action = random.choice(self.action_space[tl_id])
        else:
            with torch.no_grad():
                q_values = self.model[tl_id](state_tensor, phase)
                action = self.action_space[tl_id][torch.argmax(q_values).item()]

        reward = self.get_reward(tl_id)
        # if tl_id == "x1-y2":
        #     self.rewards.append(reward)
        #     self.times.append(self.traci.simulation.getTime())
        next_state = self.get_state(tl_id)
        next_state[3] = self.action_space[tl_id].index(next_state[3])

        # self.replay_buffer[tl_id].append((state, abstract_action, reward, next_state, abstract_phase))
        self.replay_buffer[tl_id].append((state, action, reward, next_state, phase))

        if len(self.replay_buffer[tl_id]) == self.buffer_size[tl_id]:
            self.train(tl_id)
            self.replay_buffer[tl_id] = deque()

        if self.number_of_trainings[tl_id] == self.update_target_frequency[tl_id]:
            self.number_of_trainings[tl_id] = 0
            self.update_target_model(tl_id)

        if train:
            #self.exploration_prob[tl_id] = 0.5 / (1 + self.cooling_rate[tl_id] * self.time[tl_id])
            self.exploration_prob[tl_id] = self.exploration_prob[tl_id] - (self.exploration_prob[tl_id] * self.cooling_rate[tl_id])

        return action


    def train(self, tl_id):
        """

        """
        self.model[tl_id].train()
        self.target_model[tl_id].eval()

        batch = random.sample(self.replay_buffer[tl_id], len(self.replay_buffer[tl_id]))
        states, actions, rewards, next_states, phases = zip(*batch)

        states = torch.tensor(states, dtype=torch.float32).to(self.device)
        next_states = torch.tensor(next_states, dtype=torch.float32).to(self.device)
        actions = torch.tensor(actions).unsqueeze(1).to(self.device)
        rewards = torch.tensor(rewards, dtype=torch.float32).unsqueeze(1).to(self.device)

        loss = 0
        for i in range(self.buffer_size[tl_id]):
            abstract_phase = phases[i]
            with torch.no_grad():
                next_q = self.target_model[tl_id](next_states[i].unsqueeze(0), abstract_phase)
                max_next_q = torch.max(next_q)
                target = rewards[i] + self.gamma[tl_id] * max_next_q

            target = target.detach()
            # Convert the phase to an action for the NN
            good_action = torch.tensor([[list(self.network.TLS_DETECTORS[tl_id].keys()).index(actions[i][0].item())]])
            current_q = self.model[tl_id](states[i].unsqueeze(0), abstract_phase).gather(1, good_action)
            loss += (current_q - target) ** 2

        loss = loss / self.buffer_size[tl_id]
        self.optimizer[tl_id].zero_grad()
        loss.backward()
        self.optimizer[tl_id].step()
        self.loss_history[tl_id].append(loss.item())
        self.number_of_trainings[tl_id] += 1





    def get_state(self, tl_id):
        L = sum(self.traci.lanearea.getLastStepVehicleNumber(det) for det in self._detectors_green_lanes(tl_id))
        W = self.compute_waiting_time(self._detectors_red_lanes(tl_id))
        V = self.compute_number_of_vehicles(tl_id)
        P = self.current_phase[tl_id]
        return np.array([L, W, V, P], dtype=np.float32)

    def get_reward(self, tl_id):
        L = sum(self.traci.lanearea.getLastStepVehicleNumber(det) for det in self._detectors_red_lanes(tl_id))
        W = self.compute_waiting_time(self._detectors_red_lanes(tl_id))
        return -0.5 * L - 0.7 * W

    def update_target_model(self, tl_id):
        self.target_model[tl_id].load_state_dict(self.model[tl_id].state_dict())

    def compute_waiting_time(self, detectors):
        waiting_time = 0
        for det in detectors:
            veh_ids = self.traci.lanearea.getLastStepVehicleIDs(det)
            for veh_id in veh_ids:
                waiting_time += self.traci.vehicle.getWaitingTime(veh_id)
        return waiting_time

    def compute_number_of_vehicles(self, tl_id):
        detectors = []
        for phase in self.network.TLS_DETECTORS[tl_id]:
            for det in self.network.TLS_DETECTORS[tl_id][phase]['numerical']:
                if det not in detectors:
                    detectors.append(det)
        return sum(self.traci.lanearea.getLastStepVehicleNumber(det) for det in detectors)



    def _detectors_red_lanes(self, tl_id):
        """
        Return the detectors related to red lanes for a phase.
        :param tl_id: The id of the TL
        :type tl_id: str
        :return: The list of all concerned detectors
        :rtype: list
        """
        detectors = []
        if self.current_phase[tl_id] in self.network.TLS_DETECTORS[tl_id]:
            detectors_current_phase = self.network.TLS_DETECTORS[tl_id][self.current_phase[tl_id]]['numerical']
            for i in self.network.TLS_DETECTORS[tl_id]:
                if i != self.current_phase[tl_id]:
                    for det in self.network.TLS_DETECTORS[tl_id][i]['numerical']:
                        if det not in detectors_current_phase:
                            detectors.append(det)
        return detectors


    def _detectors_green_lanes(self, tl_id):
        """
        Return the detectors related to green lanes for a phase.
        :param tl_id: The id of the TL
        :type tl_id: str
        :return: The list of all concerned detectors
        :rtype: list
        """
        detectors_current_phase = []
        if self.current_phase[tl_id] in self.network.TLS_DETECTORS[tl_id]:
            detectors_current_phase = self.network.TLS_DETECTORS[tl_id][self.current_phase[tl_id]]['numerical']
        return detectors_current_phase



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
            nn.Sequential(nn.Linear(hidden_dim, output_dim)) for _ in range(len(action_space))
        ])

    def forward(self, x, phase):
        #if 0 <= abstract_phase < len(self.heads):
        try:
            h = self.shared(x)
            return self.heads[self.convert_phase[phase]](h)
        except:
            raise ValueError(f"Invalid phase index: {self.convert_phase[phase]}")

