from sumo_experiments.strategies import Strategy
import numpy as np


class ActuatedStrategy(Strategy):
    """
    Implements an actuated agent. An intersection managed by this agent must be equipped with boolean detectors, that only
    detect if a vehicle is on its scope or not. If a detector detects a vehicle for a lane, and not for the lane of other
    traffic light phases, then the traffic light is set to a phase that is green for this lane. While detectors detect
    vehicles in lanes where traffic light is green, it remains green until there is no vehicle or the maximum green time
    of the phase is reached.
    """

    def __init__(self, network, max_phases_duration=90, yellow_time=3):
        """
        Init of class.
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param max_phases_duration: Maximum duration of each phases for all intersections
        :type max_phases_duration: int or dict
        :param yellow_time: Yellow phases duration for all intersections
        :type yellow_time: int or dict
        """
        super().__init__()
        self.network = network
        if type(max_phases_duration) is dict:
            self.max_phases_durations = max_phases_duration
        else:
            self.max_phases_durations = {identifiant: max_phases_duration for identifiant in network.TLS_DETECTORS}
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
                red_detectors = self._detectors_red_lanes(tl_id)
                green_detectors = self._detectors_green_lanes(tl_id)
                if 'y' in self.traci.trafficlight.getRedYellowGreenState(tl_id):
                    if self.current_yellow_time[tl_id] >= self.yellow_time[tl_id]:
                        self.traci.trafficlight.setPhase(tl_id, int(self.next_phase[tl_id]))
                        self.current_phase[tl_id] = self.next_phase[tl_id]
                        self.current_yellow_time[tl_id] = 0
                    else:
                        self.current_yellow_time[tl_id] += 1
                else:
                    red_detection = any([self.traci.lanearea.getLastStepVehicleNumber(det) > 0 for det in red_detectors])
                    if red_detection:
                        green_detection = any([self.traci.lanearea.getLastStepVehicleNumber(det) > 0 for det in green_detectors])
                        if not green_detection:
                            self.switch_next_phase(tl_id)
                    elif self.max_phases_durations[tl_id] is not None:
                        if self.time[tl_id] > self.max_phases_durations[tl_id]:#[self.current_max_time_index[tl_id] % len(self.max_phases_durations)]:
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
        while phases[0] != self.traci.trafficlight.getPhase(tl_id):
            phases.append(phases.pop(0))
        for phase in phases:
            for det in self.network.TLS_DETECTORS[tl_id][phase]['boolean']:
                if self.traci.lanearea.getLastStepVehicleNumber(det) > 0:
                    return phase
        return self.current_phase[tl_id]


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
            detectors_current_phase = self.network.TLS_DETECTORS[tl_id][self.current_phase[tl_id]]['boolean']
            for i in self.network.TLS_DETECTORS[tl_id]:
                if i != self.current_phase[tl_id]:
                    for det in self.network.TLS_DETECTORS[tl_id][i]['boolean']:
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
            detectors_current_phase = self.network.TLS_DETECTORS[tl_id][self.current_phase[tl_id]]['boolean']
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






