from sumo_experiments.strategies import Strategy


class FixedTimeStrategy(Strategy):
    """
    Implement a fixed time agent for all intersections of the Bologna network.
    """

    def __init__(self, network, phase_times=None, yellow_time=3):
        """
        Init of class
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param phase_times: The time for each phase of each intersection. If set to None, the intersections will have phases as defined in the net file.
        :type phase_times: dict
        :param yellow_time: Yellow phases duration for all intersections
        :type yellow_time: int or dict
        """
        super().__init__()
        self.phase_times = phase_times
        self.started = False
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {identifiant: yellow_time for identifiant in network.TLS_DETECTORS}
        self.network = network
        self.time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.current_yellow_time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.nb_switch = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.next_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.current_phase = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.nb_phases = {}
        self.phases_occurences = {identifiant: {} for identifiant in network.TLS_DETECTORS}
        self.phases_durations = {identifiant: [] for identifiant in network.TLS_DETECTORS}
        self.current_phase_duration = {identifiant: 0 for identifiant in network.TLS_DETECTORS}

    def run_all_agents(self, traci):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        if not self.started:
            self.traci = traci
            for tl_id in self.network.TL_IDS:
                self._start_agents(tl_id)
        else:
            for tl_id in self.network.TL_IDS:
                current_phase = self.traci.trafficlight.getPhase(tl_id)
                if 'y' in self.traci.trafficlight.getRedYellowGreenState(tl_id):
                    if self.current_yellow_time[tl_id] >= self.yellow_time[tl_id]:
                        self.traci.trafficlight.setPhase(tl_id, self.next_phase[tl_id])
                        self.current_phase[tl_id] = self.next_phase[tl_id]
                        self.current_yellow_time[tl_id] = 0
                    else:
                        self.current_yellow_time[tl_id] += 1
                else:
                    # Counting phase occurences
                    if current_phase not in self.phases_occurences[tl_id]:
                        self.phases_occurences[tl_id][current_phase] = 1
                    else:
                        self.phases_occurences[tl_id][current_phase] += 1
                    index_phase = list(self.network.TLS_DETECTORS[tl_id].keys()).index(current_phase)
                    if self.time[tl_id] > self.phase_times[tl_id][index_phase]:
                        self.switch_next_phase(tl_id)
                    else:
                        self.time[tl_id] += 1
                    self.current_phase_duration[tl_id] += 1


    def _start_agents(self, tl_id):
        """
        Start an agent at the beginning of the simulation.
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

    def switch_next_phase(self, tl_id):
        """
        Switch the traffic light id_tls to the next
        """
        self.nb_switch[tl_id] += 1
        current_phase = self.traci.trafficlight.getPhase(tl_id)
        self.next_phase[tl_id] = self.get_next_phase(tl_id)
        if current_phase == self.nb_phases[tl_id] - 1:
            self.traci.trafficlight.setPhase(tl_id, 0)
        else:
            self.traci.trafficlight.setPhase(tl_id, self.current_phase[tl_id] + 1)
        self.time[tl_id] = 0
        self.phases_durations[tl_id].append((current_phase, self.current_phase_duration[tl_id]))
        self.current_phase_duration[tl_id] = 0


    def get_next_phase(self, tl_id):
        """
        Get the next phase for the controller.
        :param tl_id: The id of the traffic light
        :type tl_id: str
        :return: The next phase for the controller
        :rtype: int
        """
        phases = list(self.network.TLS_DETECTORS[tl_id].keys())
        current_phase = self.traci.trafficlight.getPhase(tl_id)
        if current_phase == phases[-1]:
            return 0
        else:
            phases = phases * 2
            return phases[phases.index(current_phase) + 1]

