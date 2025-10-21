from sumo_experiments.strategies import Strategy
import numpy as np


class LongestQueueFirstStrategy(Strategy):
    """
    Implements a longest queue first agent. An intersection managed by this agent must be equipped with numerical detectors
    on the entry lanes, that count how many vehicles are currently present on the watched lane.
    At each period t, the agent selects the phase with the longest waiting queue for the next period.

    Wunderlich, R., Liu, C., Elhanany, I., & Urbanik, T. (2008). A novel signal-scheduling algorithm with quality-of-service provisioning for an isolated intersection. IEEE Transactions on intelligent transportation systems, 9(3), 536-547.
    """

    def __init__(self, network, period=30, yellow_time=3):
        """
        Init of class.
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param period: The duration of a period (in seconds).
        :type period: int or dict
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
                    if self.current_yellow_time[tl_id] >= self.yellow_time[tl_id]:
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
        else:
            phases_with_exclusion = list(self.network.TLS_DETECTORS[tl_id].keys())
            phases_with_exclusion.remove(int(self.get_next_phase(tl_id)))
            self.next_phase[tl_id] = np.random.choice(phases_with_exclusion)
        if self.traci.trafficlight.getPhase(tl_id) == self.nb_phases[tl_id] - 1:
            self.traci.trafficlight.setPhase(tl_id, 0)
        else:
            self.traci.trafficlight.setPhase(tl_id, int(self.current_phase[tl_id] + 1))
        self.time[tl_id] = 0


    def get_next_phase(self, tl_id):
        """
        Get the next phase for the controller.
        :param tl_id: The id of the traffic light
        :type tl_id: str
        :return: The next phase for the controller
        :rtype: int
        """
        phases = list(self.network.TLS_DETECTORS[tl_id].keys())
        queue_lengths = {}
        for phase in phases:
            queue_lengths[phase] = 0
            for det in self.network.TLS_DETECTORS[tl_id][phase]['numerical']:
                queue_lengths[phase] += self.traci.lanearea.getJamLengthVehicle(det)
        longest_queue = sorted(queue_lengths, key=queue_lengths.get, reverse=True)[0]
        return longest_queue



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