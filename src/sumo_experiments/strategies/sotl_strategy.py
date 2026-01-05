from sumo_experiments.strategies import Strategy


class SotlStrategy(Strategy):
    """
    Implement a SOTL agent for all intersections of the Bologna network.

    Gershenson, C. (2004). Self-organizing traffic lights. arXiv preprint nlin/0411066.
    """

    def __init__(self, network, threshold_switch=600, threshold_force=30, min_phase_duration=5, yellow_time=3):
        """
        Init of class
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param threshold_switch: The thresholds of vehicles to reach to switch phase. The number of vehicles is computed by summing the number of waiting vehicles at each time step for red lanes.
        :type threshold_switch: int or dict
        :param threshold_force: The maximum number of vehicles allowed to wait on red lanes. When this number is reached, the intersection switches its phase.
        :type threshold_force: int or dict
        :param min_phase_duration: The minimum phase time for all intersections
        :type min_phase_duration: int or dict
        :param yellow_time: Yellow phases duration for all intersections
        :type yellow_time: int or dict
        """
        super().__init__()
        self.started = False
        self.network = network
        self.countdowns = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.current_yellow_time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        if type(threshold_switch) is dict:
            self.thresholds_switch = threshold_switch
        else:
            self.thresholds_switch = {identifiant: threshold_switch for identifiant in network.TLS_DETECTORS}
        if type(threshold_force) is dict:
            self.thresholds_force = threshold_force
        else:
            self.thresholds_force = {identifiant: threshold_force for identifiant in network.TLS_DETECTORS}
        if type(min_phase_duration) is dict:
            self.min_phase_durations = min_phase_duration
        else:
            self.min_phase_durations = {identifiant: min_phase_duration for identifiant in network.TLS_DETECTORS}
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {identifiant: yellow_time for identifiant in network.TLS_DETECTORS}
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
            self._start_agents()
            return True
        else:
            for id_tls in self.network.TL_IDS:
                sum_vehicles = self.compute_vehicles_red_lanes(id_tls)
                current_phase = self.traci.trafficlight.getPhase(id_tls)
                current_state = self.traci.trafficlight.getRedYellowGreenState(id_tls)
                if 'y' in current_state:
                    if self.current_yellow_time[id_tls] >= self.yellow_time[id_tls]:
                        if current_phase + 1 != len(self.traci.trafficlight.getAllProgramLogics(id_tls)[0].phases):
                            self.traci.trafficlight.setPhase(id_tls, current_phase + 1)
                        else:
                            self.traci.trafficlight.setPhase(id_tls, 0)
                        self.current_yellow_time[id_tls] = 0
                    else:
                        self.current_yellow_time[id_tls] += 1
                elif current_phase in self.network.TLS_DETECTORS[id_tls]:
                    # Counting phase occurences
                    if current_phase not in self.phases_occurences[id_tls]:
                        self.phases_occurences[id_tls][current_phase] = 1
                    else:
                        self.phases_occurences[id_tls][current_phase] += 1
                    # Individual behaviour
                    if self.time[id_tls] >= self.min_phase_durations[id_tls]:
                        if self.countdowns[id_tls] >= self.thresholds_switch[id_tls]:
                            if not self.are_vehicles_passing(id_tls) or self.time[id_tls] >= self.thresholds_force[id_tls]:
                                self.traci.trafficlight.setPhase(id_tls, current_phase + 1)
                                self.countdowns[id_tls] = 0
                                self.time[id_tls] = 0
                                self.phases_durations[id_tls].append((current_phase, self.current_phase_duration[id_tls]))
                                self.current_phase_duration[id_tls] = 0
                            else:
                                self.countdowns[id_tls] += sum_vehicles
                                self.time[id_tls] += 1
                                self.current_phase_duration[id_tls] += 1
                        else:
                            self.countdowns[id_tls] += sum_vehicles
                            self.time[id_tls] += 1
                            self.current_phase_duration[id_tls] += 1
                    else:
                        self.countdowns[id_tls] += sum_vehicles
                        self.time[id_tls] += 1
                        self.current_phase_duration[id_tls] += 1

    def compute_vehicles_red_lanes(self, id_tls):
        """
        Compute the number of vehicles on the red lanes, with the numerical detectors.
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: The number of vehicles approaching the traffic light on the red lanes
        :rtype: int
        """
        current_phase = self.traci.trafficlight.getPhase(id_tls)
        detectors = set()
        for phase in self.network.TLS_DETECTORS[id_tls]:
            if phase != current_phase:
                detectors.update(self.network.TLS_DETECTORS[id_tls][phase]['numerical'])
        sum = 0
        for detector in detectors:
            sum += self.traci.lanearea.getLastStepVehicleNumber(detector)
        return sum

    def are_vehicles_passing(self, id_tls):
        """
        Check if vehicles are still passing the intersection, with the boolean detectors.
        :param id_tls: The id of the traffic light
        :type id_tls: str
        :return: True if vehicles are still passing the intersection, False otherwise
        :rtype: bool
        """
        current_phase = self.traci.trafficlight.getPhase(id_tls)
        for det in self.network.TLS_DETECTORS[id_tls][current_phase]['boolean']:
            if self.traci.lanearea.getLastStepVehicleNumber(det) > 0:
                return True
        return False

    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.network.TL_IDS:
            tl_logic = self.traci.trafficlight.getAllProgramLogics(tl)[0]
            nb_phase = 0
            for phase in tl_logic.phases:
                if nb_phase in self.network.TLS_DETECTORS[tl]:
                    phase.duration = 10000
                    phase.maxDur = 10000
                    phase.minDur = 10000
                nb_phase += 1
            self.traci.trafficlight.setProgramLogic(tl, tl_logic)
            self.traci.trafficlight.setPhase(tl, 0)
            self.traci.trafficlight.setPhaseDuration(tl, 10000)
        self.started = True