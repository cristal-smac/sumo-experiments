from sumo_experiments.strategies import Strategy
from sumo_experiments.agents import FixedTimeAgent

class FixedTimeStrategy(Strategy):
    """
    Implement a fixed time agent for all intersections of the network.
    """

    def __init__(self, infrastructures, detectors, phases_duration):
        """
        Init of class.
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :param detectors: The detectors of the network
        :type detectors: DetectorBuilder
        :param phases_duration: Duration of each phases for all intersections
        :type phases_duration: dict
        """
        super().__init__(infrastructures, detectors)
        self.agents = self._generate_agents(phases_duration)

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        for agent in self.agents:
            agent.choose_action()

    def _generate_agents(self, phases_duration):
        """
        Generate all agents for the strategy.
        :param phases_duration: The duration for each phases for all traffic light nodes
        :type phases_duration: dict
        :return: All the agents of the network
        :rtype: list
        """
        agents = []
        for intersection in self.relations:
            agent = FixedTimeAgent(id_intersection=intersection,
                                   id_tls_program=self.relations[intersection]['node_infos'].tl,
                                   phases_durations=phases_duration[intersection])
            agents.append(agent)
        return agents





