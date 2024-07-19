from sumo_experiments.agents import Agent
import traci
from copy import deepcopy

class AcolightAgent(Agent):
    """
    ACoLight agent : Adaptative et COoperative traffic Lights
    The ACoLight agent works like an actuated controller. The agent has an ordered set of phases, and passes to the
    next one when a stopping condition is triggered. Each phase has a maximum duration time, increased by delta seconds
    everytime that no stopping condition has been triggered. Some stopping condition can lower the maximum time by delta
    seconds when triggered. At the beginning, the maximum duration time is set to the minimum duration time.
    ACoLight agents can cooperate with each other under certain condition, but the implementation of this behaviour will be
    done at the strategy level.
    """

    def __init__(self,
                 id_intersection,
                 id_tls_program,
                 intersection_relations,
                 min_phases_durations,
                 max_phases_durations,
                 yellow_time=None,
                 activate_asymetric_saturation=True):
        """
        Init of class.
        :param id_intersection: The id of the intersection the agent will control
        :type id_intersection: str
        :param id_tls_program: The id of the traffic light program related to the intersection
        :type id_tls_program: str
        :param min_phases_durations: The minimum durations of each traffic light phase (except yellow phases). Can't be None.
        :type min_phases_durations: dict
        :param max_phases_durations: The maximum durations of each traffic light phase (except yellow phases).
        :type max_phases_durations: dict
        :param yellow_time: The duration of yellow phases. If None, yellow phase will remain as declared in the network definition.
        :type yellow_time: dict
        :param activate_asymetric_saturation: True to activate the asymetric saturation behaviour
        :type: bool
        :param intersection_relations: The relations for this intersection.
        :type intersection_relations: dict
        """
        super().__init__()
        self.started = False
        self.id_intersection = id_intersection
        self.id_tls_program = id_tls_program
        self.min_phases_durations = deepcopy(min_phases_durations)
        self.max_phases_durations = max_phases_durations
        self.current_max = deepcopy(min_phases_durations)
        self.yellow_time = yellow_time
        self.current_phase = 0
        self.countdown = 0
        self.relations = intersection_relations
        self.current_max_time_index = 0
        self.current_operation = "ACTUATED"
        self.count_operation = 0
        self.activate_asymetric_saturation = activate_asymetric_saturation

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
            # Asymetric detection
            if self.activate_asymetric_saturation:
                if self.countdown == 0:
                    if self.current_operation == 'ACTUATED':
                        if self._asymetric_saturation():
                            self.current_operation = "FIXED"
                            self.count_operation = 0
                    else:
                        if not self._asymetric_saturation():
                            self.count_operation += 1
                            if self.count_operation >= 3:
                                self.current_operation = "ACTUATED"
                                self.count_operation = 0
            # Agent behaviour
            current_phase_index = self.phases_index[traci.trafficlight.getPhase(self.id_intersection) % self.nb_phases]
            # Check if maximum time is exceeded
            if self.countdown > self.min_phases_durations[current_phase_index]:
                if self.countdown < self.max_phases_durations[current_phase_index]:
                    if self._no_vehicles_to_pass():
                        self.current_phase += 1
                        traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                        self.countdown = 0
                        return True
                    elif not self.current_operation == 'ACTUATED':
                        if self._saturated_red_lanes():
                            self.current_phase += 1
                            traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                            self.countdown = 0
                            return True
                    elif self._green_lanes_blocked():
                        self.current_phase += 1
                        traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                        self.countdown = 0
                        return True
                    else:
                        self.countdown += 1
                        return False
                else:
                    self.current_phase += 1
                    traci.trafficlight.setPhase(self.id_tls_program, self.current_phase % self.nb_phases)
                    self.countdown = 0
                    return True
            else:
                self.countdown += 1
                return False

    def saturated_edges(self):
        """
        Return the list of the saturated incoming edges
        :return: A list of saturated incoming edges
        :rtype: list
        """
        saturated_edges = []
        related_edges = self.relations['related_edges']
        for i in range(len(related_edges)):
            saturation_detectors = self.relations['related_saturation_detectors'][i]
            if any([traci.lanearea.getJamLengthVehicle(det.id) > 0 for det in saturation_detectors]):
                saturated_edges.append(related_edges[i])
        return saturated_edges

    def _green_lanes_blocked(self):
        """
        Detect if the traffic of all green lanes is blocked.
        :return: True if all the green lanes are blocked
        :rtype: bool
        """
        green_detectors = self._boolean_detectors_green_lanes()
        traffic_blocked = all([traci.lanearea.getLastStepVehicleNumber(det.id) == traci.lanearea.getJamLengthVehicle(det.id) for det in green_detectors])
        return traffic_blocked

    def _no_vehicles_to_pass(self):
        """
        Return True if boolean detectors don't detect any vehicles on green lanes.
        :return: A boolean indicating if there still are vehicles to pass on green lanes.
        :rtype: bool
        """
        green_detectors = self._boolean_detectors_green_lanes()
        no_vehicle_detection = all([traci.lanearea.getLastStepVehicleNumber(det.id) == 0 for det in green_detectors])
        return no_vehicle_detection

    def _saturated_red_lanes(self):
        """
        Return True if one of the red lanes is saturated, i.e. the length of the queue has exceeded a maximum value.
        The detection is made by boolean detectors at the beginning of the road.
        :return: A boolean indicating if a red lane is saturated
        :rtype: bool
        """
        red_detectors = self._saturation_detectors_red_lanes()
        red_detection = any([traci.lanearea.getJamLengthVehicle(det.id) > 0 for det in red_detectors])
        return red_detection

    def _asymetric_saturation(self):
        """
        Return True if an asymetric saturation, i.e. one or more West-East and one or more North-South approach are saturated.
        :return: A boolean indicating if there is an asymetric saturation.
        :rtype: bool
        """
        green_detectors = self._saturation_detectors_green_lanes()
        red_detectors = self._saturation_detectors_red_lanes()
        green_detection = any([traci.lanearea.getJamLengthVehicle(det.id) > 0 for det in green_detectors])
        red_detection = any([traci.lanearea.getJamLengthVehicle(det.id) > 0 for det in red_detectors])
        return green_detection and red_detection

    def _saturation_detectors_red_lanes(self):
        """
        Return the saturation detectors related to red lanes for a phase.
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
                    if len(self.relations['related_saturation_detectors'][edge_index]) != 0:
                        detector = self.relations['related_saturation_detectors'][edge_index][lane_number]
                        if detector not in detectors:
                            detectors.append(detector)
        return detectors

    def _saturation_detectors_green_lanes(self):
        """
        Return the saturation detectors related to red lanes for a phase.
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
                    if len(self.relations['related_saturation_detectors'][edge_index]) != 0:
                        detector = self.relations['related_saturation_detectors'][edge_index][lane_number]
                        if detector not in detectors:
                            detectors.append(detector)
        return detectors

    def _boolean_detectors_green_lanes(self):
        """
        Return the boolean detectors related to green lanes for a phase.
        :return: The list of all concerned boolean detectors
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
        if self.max_phases_durations is not None:
            traci.trafficlight.setPhaseDuration(self.id_tls_program, self.max_phases_durations[0])
        else:
            traci.trafficlight.setPhaseDuration(self.id_tls_program, 10000)
        self.started = True
