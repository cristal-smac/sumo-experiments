from sumo_experiments.strategies import Strategy
import operator


class MaxPressureStrategy(Strategy):
    """
    Implement a max pressure agent for all intersections of the Bologna network.

    Varaiya, P. (2013). Max pressure control of a network of signalized intersections. Transportation Research Part C: Emerging Technologies, 36, 177-195.
    """

    def __init__(self, network, period=30, yellow_time=3):
        """
        Init of class
        :param network: The network to deploy the strategy
        :type network: src.sumo_experiments.Network
        :param period: The duration of a period (in seconds).
        :type period: int or dict
        :param yellow_time: Yellow phases duration for all intersections
        :type yellow_time: int or dict
        """
        super().__init__()
        self.started = False
        if type(period) is dict:
            self.period_times = period
        else:
            self.period_times = {identifiant: period for identifiant in network.TLS_DETECTORS}
        if type(yellow_time) is dict:
            self.yellow_time = yellow_time
        else:
            self.yellow_time = {identifiant: yellow_time for identifiant in network.TLS_DETECTORS}
        self.network = network
        self.countdowns = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
        self.to_switch = {identifiant: None for identifiant in self.network.TLS_DETECTORS}
        self.current_yellow_time = {identifiant: 0 for identifiant in self.network.TLS_DETECTORS}
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
                current_phase = self.traci.trafficlight.getPhase(id_tls)
                current_state = self.traci.trafficlight.getRedYellowGreenState(id_tls)
                if 'y' in current_state:
                    if self.current_yellow_time[id_tls] >= self.yellow_time[id_tls]:
                        if current_phase + 1 != len(self.traci.trafficlight.getAllProgramLogics(id_tls)[0].phases) and current_phase + 1 not in self.network.TLS_DETECTORS[id_tls].keys():
                            self.traci.trafficlight.setPhase(id_tls, current_phase + 1)
                        else:
                            self.traci.trafficlight.setPhase(id_tls, int(self.to_switch[id_tls]))
                            self.to_switch[id_tls] = None
                        self.current_yellow_time[id_tls] = 0
                    else:
                        self.current_yellow_time[id_tls] += 1
                else:
                    # Counting phase occurences
                    if current_phase not in self.phases_occurences[id_tls]:
                        self.phases_occurences[id_tls][current_phase] = 1
                    else:
                        self.phases_occurences[id_tls][current_phase] += 1
                    # Individual behaviour
                    # if current_phase in self.network.TLS_DETECTORS[id_tls] and self.to_switch[id_tls] is not None:
                    #     self.traci.trafficlight.setPhase(id_tls, self.to_switch[id_tls])
                    #     self.to_switch[id_tls] = None
                    #     self.countdowns[id_tls] = 0
                    #     print("ici " + str(id_tls))
                    if self.countdowns[id_tls] >= self.period_times[id_tls]:
                        if current_phase in self.network.TLS_DETECTORS[id_tls]:
                            pressures = self._compute_pressure(self.network.TLS_DETECTORS[id_tls])
                            phase_max_pressure = max(pressures.items(), key=operator.itemgetter(1))[0]
                            if phase_max_pressure != current_phase:
                                self.phases_durations[id_tls].append((current_phase, self.current_phase_duration[id_tls]))
                                self.current_phase_duration[id_tls] = 0
                                self.to_switch[id_tls] = phase_max_pressure
                                self.traci.trafficlight.setPhase(id_tls, current_phase + 1)
                                self.countdowns[id_tls] = 0
                            else:
                                self.countdowns[id_tls] = 1
                                self.current_phase_duration[id_tls] += 1
                        else:
                            self.countdowns[id_tls] += 1
                            self.current_phase_duration[id_tls] += 1
                    else:
                        self.countdowns[id_tls] += 1
                        self.current_phase_duration[id_tls] += 1


    def _compute_pressure(self, detectors):
        """
        Compute the pressure for a phase. The pressure is computed in vehicles.
        :param detectors: The list of detectors of the intersection.
        :type detectors: dict
        :return: The pressures of all phases.
        :rtype: dict
        """
        pressures = {}
        for phase in detectors:
            pressure = 0
            routes = {}
            nb_vehicles = 0
            for detector in detectors[phase]['numerical']:
                pressure += self.traci.lanearea.getLastStepOccupancy(detector) * self.traci.lanearea.getLastStepVehicleNumber(detector)
                vehicles = self.traci.lanearea.getLastStepVehicleIDs(detector)
                for vehicle in vehicles:
                    direction = self.traci.vehicle.getRoute(vehicle)[0]
                    if direction in routes:
                        routes[direction] += 1
                    else:
                        routes[direction] = 1
                    nb_vehicles += 1
            for key in routes:
                routes[key] = routes[key] / nb_vehicles
            for detector in detectors[phase]['exit']:
                edge = self.traci.lanearea.getLaneID(detector).split('_')[0]
                k = routes[edge] if edge in routes else 0
                pressure -= self.traci.lanearea.getLastStepOccupancy(detector) * self.traci.lanearea.getLastStepVehicleNumber(detector) * k
            pressures[phase] = pressure
        return pressures

    def _start_agents(self):
        """
        Start an agent at the beginning of the simulation.
        """
        for tl in self.network.TL_IDS:
            tl_logic = self.traci.trafficlight.getAllProgramLogics(tl)[0]
            nb_phase = 0
            for phase in tl_logic.phases:
                #if nb_phase in self.TLS_DETECTORS[tl]:
                phase.duration = 1000000
                phase.maxDur = 1000000
                phase.minDur = 1000000
                nb_phase += 1
            self.traci.trafficlight.setProgramLogic(tl, tl_logic)
            self.traci.trafficlight.setPhase(tl, 0)
            self.traci.trafficlight.setPhaseDuration(tl, 100000)
        self.started = True