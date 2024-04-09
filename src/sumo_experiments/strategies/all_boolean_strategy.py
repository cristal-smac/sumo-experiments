from sumo_experiments.strategies import Strategy
from sumo_experiments.agents import BooleanAgent


class BooleanStrategy(Strategy):
    """
    Implement a boolean agent for all intersections of the network.
    """

    def __init__(self, infrastructures, detectors, max_phases_durations, yellow_times):
        """
        Init of class.
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :param detectors: The detectors of the network
        :type detectors: DetectorBuilder
        :param max_phases_durations: Maximum duration of each phases for all intersections
        :type max_phases_durations: dict
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        """
        super().__init__(infrastructures, detectors)
        self.agents = self._generate_agents(max_phases_durations, yellow_times)

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        for agent in self.agents:
            agent.choose_action()

    def _generate_agents(self, max_phases_durations, yellow_times):
        """
        Generate all agents for the strategy.
        :param max_phases_durations: The maximum duration for each green phases for all traffic light nodes
        :type max_phases_durations: dict
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        :return: All the agents of the network
        :rtype: list
        """
        agents = []
        for intersection in self.relations:
            agent = BooleanAgent(id_intersection=intersection,
                                 id_tls_program=self.relations[intersection]['node_infos'].tl,
                                 max_phases_durations=max_phases_durations[intersection],
                                 yellow_time=yellow_times[intersection],
                                 intersection_relations=self.relations[intersection])
            agents.append(agent)
        return agents





