from sumo_experiments.agents import Agent
import traci


class BooleanAgent(Agent):
    """
    Implements a boolean agent. An intersection managed by this agent must be equipped with boolean detectors, that only
    detect if a vehicle is on its scope or not. If a detector detects a vehicle for a lane, and not for the lane of other
    traffic light phases, then the traffic light is set to a phase that is green for this lane. While detectors detect
    vehicles in lanes where traffic light is green, it remains green until there is no vehicle or the maximum green time
    of the phase is reached.
    """

    def __init__(self, id_intersection, id_tls_program, intersection_relations, max_phases_durations=None, yellow_time=None):
        """
        Init of class.
        :param id_intersection: The id of the intersection the agent will control
        :type id_intersection: str
        :param id_tls_program: The id of the traffic light program related to the intersection
        :type id_tls_program: str
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
        self.max_phases_durations = max_phases_durations
        self.yellow_time = yellow_time
        self.current_phase = 0
        self.countdown = 0
        self.relations = intersection_relations
        self.current_max_time_index = 0

    def choose_action(self):
        """
        Switch to the next phase when countdown is equal to the current phase duration.
        :return: True if the agent switched to another phase, False otherwise
        :rtype: bool
        """
        if not self.started:
            self._start_agent()
            return True
        red_detectors = self._detectors_red_lanes()
        green_detectors = self._detectors_green_lanes()
        if 'y' not in traci.trafficlight.getRedYellowGreenState(self.id_tls_program):
            red_detection = any([traci.lanearea.getLastStepVehicleNumber(det.id) > 0 for det in red_detectors])
            if red_detection:
                green_detection = any([traci.lanearea.getLastStepVehicleNumber(det.id) > 0 for det in green_detectors])
                if not green_detection:
                    self.current_phase += 1
                    traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                    return True
            elif self.max_phases_durations is not None:
                if self.countdown > self.max_phases_durations[self.current_max_time_index % len(self.max_phases_durations)]:
                    self.current_phase += 1
                    traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
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
                    detector = self.relations['related_boolean_detectors'][edge_index][lane_number]
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
                    detector = self.relations['related_boolean_detectors'][edge_index][lane_number]
                    if detector not in detectors:
                        detectors.append(detector)
        return detectors

    def _start_agent(self):
        """
        Start an agent at the beginning of the simulation.
        """
        self.nb_phases = len(traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0].phases)
        tl_logic = traci.trafficlight.getAllProgramLogics(self.id_tls_program)[0]
        phase_index = 0
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
                phase_index += 1
        traci.trafficlight.setProgramLogic(self.id_tls_program, tl_logic)
        traci.trafficlight.setPhase(self.id_tls_program, 0)
        if self.max_phases_durations is not None:
            traci.trafficlight.setPhaseDuration(self.id_tls_program, self.max_phases_durations[0])
        else:
            traci.trafficlight.setPhaseDuration(self.id_tls_program, 10000)
        self.started = True
