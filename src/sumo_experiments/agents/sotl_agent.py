from sumo_experiments.agents import Agent
import traci


class SOTLAgent(Agent):
    """
    Implements the SOTL system from [1] to control an intersection. An intersection managed by this agent must be
    equipped with numerical detectors and boolean detectors.
    Each agent is directed by three rules :
    1) Each second, the system increments a variable with the number of vehicles incoming or waiting on red lanes. If this
    number is superior to a defined threshold x, then the traffic light switch to another phase.
    2) This rule 1 is not effective if the boolean detectors of green lanes detect vehicles that are currently crossing the
    intersection, so groups of vehicles won't be cut.
    3) The rule 2 is not effective if the numerical detectors on red lanes count a number of incoming vehicles superior
    to another threshold y.

    [1] Cools, S. B., Gershenson, C., & D’Hooghe, B. (2013). Self-organizing traffic lights: A realistic simulation.
    Advances in applied self-organizing systems, 45-55.
    """

    def __init__(self,
                 id_intersection,
                 id_tls_program,
                 intersection_relations,
                 threshold_switch,
                 threshold_force,
                 min_phase_duration,
                 yellow_time=None):
        """
        Init of class.
        :param id_intersection: The id of the intersection the agent will control
        :type id_intersection: str
        :param id_tls_program: The id of the traffic light program related to the intersection
        :type id_tls_program: str
        :param threshold_switch: The threshold of vehicles over time that will release a phase switch
        :type threshold_switch: int
        :param threshold_force: The threshold of vehicles for one step that will force a phase switch, even if vehicles
        are still crossing the intersection on green lanes
        :type threshold_force: int
        :param min_phase_duration: The minimum duration for a green phase before switching.
        :type min_phase_duration: int
        :param yellow_time: The duration of yellow phases. If None, yellow phase will remain as declared in the network definition.
        :type yellow_time: dict
        :param intersection_relations: The relations for this intersection.
        :type intersection_relations: dict
        """
        super().__init__()
        self.started = False
        self.id_intersection = id_intersection
        self.id_tls_program = id_tls_program
        self.yellow_time = yellow_time
        self.current_phase = 0
        self.countdown = 0
        self.time_countdown = 0
        self.relations = intersection_relations
        self.current_max_time_index = 0
        self.threshold_switch = threshold_switch
        self.threshold_force = threshold_force
        self.min_phase_duration = min_phase_duration
        self.count_function = traci.lanearea.getLastStepVehicleNumber

    def choose_action(self):
        """
        Choose an action for the next step, following the three rules of SOTL system.
        For intersections with more than 2 phases (except yellows), if all conditions are met, the agent will select
        the phase with the highest count of incoming vehicles.
        :return: True if the agent switched to another phase, False otherwise
        :rtype: bool
        """
        if not self.started:
            self._start_agent()
            return True
        if 'y' not in traci.trafficlight.getRedYellowGreenState(self.id_tls_program):
            if self.countdown == 0:
                traci.trafficlight.setPhase(self.id_tls_program, self.current_phase)
            # Check if minimum time is exceeded
            if self.countdown > self.threshold_switch and self.time_countdown >= self.min_phase_duration:
                green_detectors = self.boolean_detectors_green_lanes()
                detectors_trigerred = [self.count_function(det.id) > 0 for det in green_detectors]
                red_detectors = self.numerical_detectors_red_lanes()
                phase_count = sum([self.count_function(det.id) for det in red_detectors])
                if not any(detectors_trigerred) or phase_count >= self.threshold_force:
                    next_phase = self.current_phase
                    max_count = 0
                    for i in range(self.nb_phases):
                        if i != self.current_phase:
                            traci.trafficlight.setPhase(self.id_tls_program, i)
                            if 'y' not in traci.trafficlight.getRedYellowGreenState(self.id_tls_program):
                                red_detectors = self.numerical_detectors_red_lanes()
                                phase_count = sum([self.count_function(det.id) for det in red_detectors])
                                if phase_count > max_count:
                                    next_phase = i
                                    max_count = phase_count
                    traci.trafficlight.setPhase(self.id_tls_program, self.current_phase + 1)
                    self.current_phase = next_phase
                    self.countdown = 0
                    self.time_countdown = 0
                    return True
            else:
                red_detectors = self.numerical_detectors_red_lanes()
                add = 1
                for detector in red_detectors:
                    add += self.count_function(detector.id)
                self.countdown += add
                self.time_countdown += 1
                return False

    def numerical_detectors_red_lanes(self):
        """
        Return the numerical detectors related to red lanes for a phase.
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

    def boolean_detectors_green_lanes(self):
        """
        Return the boolean detectors related to green lanes for a phase.
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
                    detector = self.relations['related_boolean_detectors'][edge_index][lane_number]
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
                phase.duration = 10000
                phase.maxDur = 10000
                phase.minDur = 10000
                self.phases_index[nb_phase] = phase_index
                phase_index += 1
            nb_phase += 1
        traci.trafficlight.setProgramLogic(self.id_tls_program, tl_logic)
        traci.trafficlight.setPhase(self.id_tls_program, 0)
        traci.trafficlight.setPhaseDuration(self.id_tls_program, 10000)
        self.started = True