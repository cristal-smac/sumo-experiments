from sumo_experiments.strategies import Strategy
from sumo_experiments.agents import SOTLAgent


class SOTLStrategy(Strategy):
    """
    Implement a boolean agent for all intersections of the network.
    """

    def __init__(self,
                 infrastructures,
                 detectors,
                 thresholds_switch,
                 thresholds_force,
                 min_phase_durations,
                 yellow_times):
        """
        Init of class.
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :param detectors: The detectors of the network
        :type detectors: DetectorBuilder
        :param thresholds_switch: Threshold of vehicle that will release a traffic light switch, for all intersections
        :type thresholds_switch: dict
        :param thresholds_force: Thresholds of vehicles for one step that will force a phase switch, even if vehicles
        are still crossing the intersection on green lanes, for all intersections
        :type thresholds_force: dict
        :param min_phase_durations: The minimum green phase duration for all intersections
        :type min_phase_durations: dict
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        """
        super().__init__(infrastructures, detectors)
        self.agents = self._generate_agents(thresholds_switch,
                                            thresholds_force,
                                            min_phase_durations,
                                            yellow_times)

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        for agent in self.agents:
            agent.choose_action()

    def _generate_agents(self, thresholds_switch, thresholds_force, min_phase_durations, yellow_times):
        """
        Generate all agents for the strategy.
        :param thresholds_switch: Threshold of vehicle that will release a traffic light switch, for all intersections
        :type thresholds_switch: dict
        :param thresholds_force: Thresholds of vehicles for one step that will force a phase switch, even if vehicles
        are still crossing the intersection on green lanes, for all intersections
        :type thresholds_force: dict
        :param min_phase_durations: The minimum green phase duration for all intersections
        :type min_phase_durations: dict
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        :return: All the agents of the network
        :rtype: list
        """
        agents = []
        for intersection in self.relations:
            agent = SOTLAgent(id_intersection=intersection,
                              id_tls_program=self.relations[intersection]['node_infos'].tl,
                              threshold_switch=thresholds_switch[intersection],
                              threshold_force=thresholds_force[intersection],
                              min_phase_duration=min_phase_durations[intersection],
                              yellow_time=yellow_times[intersection],
                              intersection_relations=self.relations[intersection])
            agents.append(agent)
        return agents
