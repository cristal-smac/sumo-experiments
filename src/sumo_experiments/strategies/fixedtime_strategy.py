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

    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.network.TL_IDS:
            assert tl in self.phase_times, "The list of intersections given in the phase_times parameter is not exhaustive."
            tl_logic = self.traci.trafficlight.getAllProgramLogics(tl)[0]
            nb_phase = 0
            i = 0
            for phase in tl_logic.phases:
                if 'y' in phase.state:
                    phase.duration = self.yellow_time
                    phase.maxDur = self.yellow_time
                    phase.minDur = self.yellow_time
                if nb_phase in self.network.TLS_DETECTORS[tl]:
                    if nb_phase in self.phase_times:
                        phase.duration = self.phase_times[tl][phase]
                        phase.maxDur = self.phase_times[tl][phase]
                        phase.minDur = self.phase_times[tl][phase]
                        i += 1
                nb_phase += 1
            self.traci.trafficlight.setProgramLogic(tl, tl_logic)
            self.traci.trafficlight.setPhase(tl, 0)
        self.started = True

    def run_all_agents(self, traci):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        if not self.started:
            self.traci = traci
            if self.phase_times is not None:
                self._start_agents()
        else:
            for id_tls in self.network.TL_IDS:
                current_phase = self.traci.trafficlight.getPhase(id_tls)
                current_state = self.traci.trafficlight.getRedYellowGreenState(id_tls)
                if current_phase in self.phase_times[id_tls]:
                    if self.time[id_tls] >= self.phase_times[id_tls][current_phase]:
                        self.traci.trafficlight.setPhase(id_tls, current_phase + 1)
                        self.time[id_tls] = 0
                    else:
                        self.time[id_tls] += 1
                elif 'y' in current_state:
                    if self.current_yellow_time[id_tls] >= self.yellow_time:
                        if current_phase + 1 != len(self.traci.trafficlight.getAllProgramLogics(id_tls)[0].phases):
                            self.traci.trafficlight.setPhase(id_tls, current_phase + 1)
                        else:
                            self.traci.trafficlight.setPhase(id_tls, 0)
                        self.current_yellow_time[id_tls] = 0
                    else:
                        self.current_yellow_time[id_tls] += 1

