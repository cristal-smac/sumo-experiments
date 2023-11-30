from sumo_experiments.agents import Agent
import traci


class NumericalAgent(Agent):
    """
    Implements a numerical agent. An intersection managed by this agent must be equipped with numerical detectors, that
    count how many vehicles are currently on the watched lane. If a numerical detector counts more vehicle on a red lane
    than the defined threshold, then it will directly switch to green for this lane. The phases last at least a defined
    minimum time and at maximum a defined maximum time. It's possible not to define maximum phases time. Then, phases
    will last until the threshold on other lane is passed.
    """

    def __init__(self,
                 id_intersection,
                 id_tls_program,
                 intersection_relations,
                 min_phases_durations,
                 threshold,
                 counted_vehicles='all',
                 max_phases_durations=None,
                 yellow_time=None):
        """
        Init of class.
        :param id_intersection: The id of the intersection the agent will control
        :type id_intersection: str
        :param id_tls_program: The id of the traffic light program related to the intersection
        :type id_tls_program: str
        :param min_phases_durations: The minimum durations of each traffic light phase (except yellow phases). Can't be None.
        :type min_phases_durations: dict
        :param threshold: The threshold of vehicles that will release a phase switch
        :type threshold: int
        :param counted_vehicles: The vehicles that will be count. 'all' means that all vehicles detected by the numerical
        detector will be counted. 'stopped' means that only stopped vehicles detected will be counted.
        :type counted_vehicles: str
        :param max_phases_durations: The maximum durations of each traffic light phase (except yellow phases). If None,
        traffic lights will not switch to another phase until they are car detected on red lanes.
        :type max_phases_durations: dict
        :param yellow_time: The duration of yellow phases. If None, yellow phase will remain as declared in the network definition.
        :type yellow_time: dict
        :param intersection_relations: The relations for this intersection.
        :type intersection_relations: dict
        """
        super().__init__()
        self.started = False
        self.id_intersection = id_intersection
        self.id_tls_program = id_tls_program
        self.min_phases_durations = min_phases_durations
        self.max_phases_durations = max_phases_durations
        self.yellow_time = yellow_time
        self.current_phase = 0
        self.countdown = 0
        self.relations = intersection_relations
        self.current_max_time_index = 0
        self.threshold = threshold
        if counted_vehicles == 'all':
            self.count_function = traci.lanearea.getLastStepVehicleNumber
        elif counted_vehicles == 'stopped':
            self.count_function = traci.lanearea.getJamLengthVehicle
        else:
            raise ValueError('counted_vehicles argument is not valid.')

    def choose_action(self):
        """
        If the threshold is passed for a defined lane and the minimum time is exceeded, switch to a phase that is green
        for this lane.
        :return: True if the agent switched to another phase, False otherwise
        :rtype: bool
        """
        if not self.started:
            self._start_agent()
            return True
        if 'y' not in traci.trafficlight.getRedYellowGreenState(self.id_tls_program):
            # Check if minimum time is exceeded
            if self.countdown > self.min_phases_durations[self.phases_index[self.current_phase % self.nb_phases]]:
                red_detectors = self._detectors_red_lanes()
                detectors_trigerred = [self.count_function(det.id) >= self.threshold for det in red_detectors]
                if any(detectors_trigerred):
                    edges = []
                    for i in range(len(red_detectors)):
                        if detectors_trigerred[i]:
                            edges.append(red_detectors[i].edge)
                    edges = set(edges)
                    self.current_phase += 1
                    traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                    green_detectors = set(self._detectors_green_lanes())
                    cpt = 0
                    while not edges.issubset(green_detectors) and cpt < self.nb_phases:
                        self.current_phase += 1
                        traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                        green_detectors = set(self._detectors_green_lanes())
                        cpt += 1
                    if cpt >= self.nb_phases:
                        self.current_phase += 1
                        traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                    self.countdown = 0
                    return True
            else:
                self.countdown += 1
                return False

    def _detectors_red_lanes(self):
        """
        Return the detectors related to red lanes for a phase.
        :return: The list of all concerned detectors
        :rtype: list
        """
        detectors = []
        current_phase = traci.trafficlight.getRedYellowGreenState(self.id_tls_program)
        phases = traci.trafficlight.getControlledLinks(self.id_tls_program)
        for i in range(len(current_phase)):
            link = current_phase[i]
            if link == 'r':
                link_infos = phases[i]
                for info in link_infos:
                    lane = info[0]
                    lane_number = int(lane.split('_')[-1])
                    edge = lane[:-2]
                    edge_index = self.relations['related_edges'].index(edge)
                    detector = self.relations['related_numerical_detectors'][edge_index][lane_number]
                    if detector not in detectors:
                        detectors.append(detector)
        return detectors

    def _detectors_green_lanes(self):
        """
        Return the detectors related to green lanes for a phase.
        :return: The list of all concerned detectors
        :rtype: list
        """
        detectors = []
        current_phase = traci.trafficlight.getRedYellowGreenState(self.id_tls_program)
        phases = traci.trafficlight.getControlledLinks(self.id_tls_program)
        for i in range(len(current_phase)):
            link = current_phase[i]
            if link == 'g' or link == 'G':
                link_infos = phases[i]
                for info in link_infos:
                    lane = info[0]
                    lane_number = int(lane.split('_')[-1])
                    edge = lane[:-2]
                    edge_index = self.relations['related_edges'].index(edge)
                    detector = self.relations['related_numerical_detectors'][edge_index][lane_number]
                    if detector not in detectors:
                        detectors.append(detector)
        return detectors

    def _start_agent(self):
        """
        Start an agent at the beginning of the simulation.
        """
        self.nb_phases = len(traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0].phases)
        self.phases_index = {}
        tl_logic = traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0]
        phase_index = 0
        nb_phase = 0
        for phase in tl_logic.phases:
            if 'y' in phase.state:
                if self.yellow_time is not None:
                    phase.duration = self.yellow_time
                    phase.minDur = self.yellow_time
                    phase.maxDur = self.yellow_time
            else:
                if self.max_phases_durations is not None:
                    phase.duration = self.max_phases_durations[phase_index] + 5
                    phase.maxDur = self.max_phases_durations[phase_index] + 5
                    phase.minDur = self.max_phases_durations[phase_index] + 5
                else:
                    phase.duration = 10000
                    phase.maxDur = 10000
                    phase.minDur = 10000
                self.phases_index[nb_phase] = phase_index
                phase_index += 1
            nb_phase += 1
        traci.trafficlight.setProgramLogic(self.id_tls_program, tl_logic)
        traci.trafficlight.setPhase(self.id_tls_program, 0)
        if self.max_phases_durations is not None:
            traci.trafficlight.setPhaseDuration(self.id_tls_program, self.max_phases_durations[0])
        else:
            traci.trafficlight.setPhaseDuration(self.id_tls_program, 10000)
        self.started = True
