from sumo_experiments.strategies import Strategy
from sumo_experiments.agents import NumericalAgent


class NumericalStrategy(Strategy):
    """
    Implement a boolean agent for all intersections of the network.
    """

    def __init__(self,
                 infrastructures,
                 detectors,
                 min_phases_durations,
                 max_phases_durations,
                 thresholds,
                 counted_vehicles,
                 yellow_times):
        """
        Init of class.
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :param detectors: The detectors of the network
        :type detectors: DetectorBuilder
        :param min_phases_durations: Minimum duration of each phases for all intersections
        :type min_phases_durations: dict
        :param max_phases_durations: Maximum duration of each phases for all intersections
        :type max_phases_durations: dict
        :param thresholds: Threshold of vehicle that will release a traffic light switch for all intersections
        :type thresholds: dict
        :param counted_vehicles: The method to count vehicles : 'all' will detect all running vehicles in the detector scope,
        'stopped' will only detect stopped vehicles in detector scope.
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        """
        super().__init__(infrastructures, detectors)
        self.agents = self._generate_agents(min_phases_durations,
                                            max_phases_durations,
                                            thresholds,
                                            counted_vehicles,
                                            yellow_times)

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        for agent in self.agents:
            agent.choose_action()

    def _generate_agents(self, min_phases_durations, max_phases_durations, thresholds, counted_vehicles, yellow_times):
        """
        Generate all agents for the strategy.
        :param max_phases_durations: The maximum duration for each green phases for all traffic light nodes
        :type max_phases_durations: dict
        :param min_phases_durations: Minimum duration of each phases for all intersections
        :type min_phases_durations: dict
        :param thresholds: Threshold of vehicle that will release a traffic light switch for all intersections
        :type thresholds: dict
        :param counted_vehicles: The method to count vehicles : 'all' will detect all running vehicles in the detector scope,
        'stopped' will only detect stopped vehicles in detector scope.
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        :return: All the agents of the network
        :rtype: list
        """
        agents = []
        for intersection in self.relations:
            agent = NumericalAgent(id_intersection=intersection,
                                   id_tls_program=self.relations[intersection]['node_infos'].tl,
                                   min_phases_durations=min_phases_durations[intersection],
                                   max_phases_durations=max_phases_durations[intersection],
                                   threshold=thresholds[intersection],
                                   counted_vehicles=counted_vehicles,
                                   yellow_time=yellow_times[intersection],
                                   intersection_relations=self.relations[intersection])
            agents.append(agent)
        return agents





