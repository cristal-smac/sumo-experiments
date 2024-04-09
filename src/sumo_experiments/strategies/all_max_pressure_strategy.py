from sumo_experiments.strategies import Strategy
from sumo_experiments.agents import MaxPressureAgent


class MaxPressureStrategy(Strategy):
    """
    Implement a max pressure agent for all intersections of the network.
    """

    def __init__(self,
                 infrastructures,
                 detectors,
                 periods,
                 counted_vehicles,
                 yellow_times):
        """
        Init of class.
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :param detectors: The detectors of the network
        :type detectors: DetectorBuilder
        :param periods: The period times (in s) for the max pressure agent.
        :type periods: dict
        :param counted_vehicles: The method to count vehicles : 'all' will detect all running vehicles in the detector scope,
        'stopped' will only detect stopped vehicles in detector scope.
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        """
        super().__init__(infrastructures, detectors)
        self.agents = self._generate_agents(periods,
                                            counted_vehicles,
                                            yellow_times)

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        for agent in self.agents:
            agent.choose_action()

    def _generate_agents(self, periods, counted_vehicles, yellow_times):
        """
        Generate all agents for the strategy.
        :param periods: The period times (in s) for the max pressure agent.
        :type periods: dict
        :param counted_vehicles: The method to count vehicles : 'all' will detect all running vehicles in the detector scope,
        'stopped' will only detect stopped vehicles in detector scope.
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        :return: All the agents of the network
        :rtype: list
        """
        agents = []
        for intersection in self.relations:
            agent = MaxPressureAgent(id_intersection=intersection,
                                     id_tls_program=self.relations[intersection]['node_infos'].tl,
                                     period=periods[intersection],
                                     counted_vehicles=counted_vehicles,
                                     yellow_time=yellow_times[intersection],
                                     intersection_relations=self.relations[intersection])
            agents.append(agent)
        return agents





