import traci

from sumo_experiments.strategies import Strategy
from sumo_experiments.agents import RLAgent2


class RL2Strategy(Strategy):
    """
    Implement a RL2 agent for all intersections of the network.
    """

    def __init__(self,
                 infrastructures,
                 detectors,
                 yellow_times,
                 period_durations,
                 epsilons,
                 epsilon_updaters,
                 min_epsilons,
                 decreasing_episode,
                 episode_length,
                 nb_not_correlated_episodes,
                 max_batch_size,
                 gammas,
                 nbs_epochs,
                 frequences_target_network_update
                 ):
        """
        Init of class.
        :param infrastructures: The infrastructures of the network
        :type infrastructures: InfrastructureBuilder
        :param detectors: The detectors of the network
        :type detectors: DetectorBuilder
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        :param period_durations: The duration of a period where traffic light can't switch, for each intersection.
        :type period_durations: dict
        :param epsilons: Epsilon value for the greedy action selection at the beginning of the simulation, for all intersections. Must be between 0 and 1 included.
        :type epsilons: dict
        :param epsilon_updaters: Coefficient that will be used to compute the new epsilon after each selection, if the episode number is greater than decreasing_episode, for all intersections. Must be between 0 and 1 included.
        :type epsilon_updaters: dict
        :param min_epsilons: Minimum value for epsilon, for all intersections. Must be between 0 and 1 included.
        :type min_epsilons: dict
        :param decreasing_episode: The episode where epsilon value will start to decrease, for all intersections. All intersections must have the same value.
        :type decreasing_episode: int
        :param episode_length: Length of an episode, for all intersections, in number of simulation steps. All intersections must have the same value.
        :type episode_length: int
        :param nb_not_correlated_episodes: Number of episodes after which all the vehicles are removed (network reset).
        :type nb_not_correlated_episodes: int
        :param max_batch_size: The maximum size of the batch to fit the model, for all intersections. Must be lower than the episode length. All intersections must have the same value.
        :type max_batch_size: int
        :param gammas: Discount factor to compute the Q value, for all intersections. Must be between 0 and 1 included.
        :type gammas: dict
        :param nbs_epochs: Number of epochs for main network fitting, for all intersections.
        :type nbs_epochs: dict
        :param frequences_target_network_update: Frequence (in number of episodes) when the target network will be updated with the main network weights, for all intersections.
        :type frequences_target_network_update: dict
        """
        super().__init__(infrastructures, detectors)
        self.agents = self._generate_agents(yellow_times,
                                            period_durations,
                                            epsilons,
                                            epsilon_updaters,
                                            min_epsilons,
                                            decreasing_episode,
                                            episode_length,
                                            max_batch_size,
                                            gammas,
                                            nbs_epochs,
                                            frequences_target_network_update
                                            )
        self.episode_length = episode_length
        self.nb_not_correlated_episodes = nb_not_correlated_episodes
        self.current_episode = 0
        self.trained = False

    def run_all_agents(self):
        """
        Process agents to make one action each.
        :return: Nothing
        """
        if traci.simulation.getTime() % self.episode_length == 0 and self.current_episode < self.nb_not_correlated_episodes:
            self.current_episode += 1
            self._remove_vehicles()
        for agent in self.agents:
            agent.choose_action()

    def _generate_agents(self,
                         yellow_times,
                         period_durations,
                         epsilons,
                         epsilon_updaters,
                         min_epsilons,
                         decreasing_episode,
                         episode_length,
                         max_batch_size,
                         gammas,
                         nbs_epochs,
                         frequences_target_network_update
                         ):
        """
        Generate all agents for the strategy.
        :param yellow_times: Yellow phases duration for all intersections
        :type yellow_times: dict
        :param period_durations: The duration of a period where traffic light can't switch, for each intersection.
        :type period_durations: dict
        :param epsilons: Epsilon value for the greedy action selection at the beginning of the simulation, for all intersections. Must be between 0 and 1 included.
        :type epsilons: dict
        :param epsilon_updaters: Coefficient that will be used to compute the new epsilon after each selection, if the episode number is greater than decreasing_episode, for all intersections. Must be between 0 and 1 included.
        :type epsilon_updaters: dict
        :param min_epsilons: Minimum value for epsilon, for all intersections. Must be between 0 and 1 included.
        :type min_epsilons: dict
        :param decreasing_episode: The episode where epsilon value will start to decrease, for all intersections. All intersections must have the same value.
        :type decreasing_episode: int
        :param episode_length: Length of an episode, for all intersections, in number of simulation steps. All intersections must have the same value.
        :type episode_length: int
        :param max_batch_size: The maximum size of the batch to fit the model, for all intersections. Must be lower than the episode length. All intersections must have the same value.
        :type max_batch_size: int
        :param gammas: Discount factor to compute the Q value, for all intersections. Must be between 0 and 1 included.
        :type gammas: dict
        :param nbs_epochs: Number of epochs for main network fitting, for all intersections.
        :type nbs_epochs: dict
        :param frequences_target_network_update: Frequence (in number of episodes) when the target network will be updated with the main network weights, for all intersections.
        :type frequences_target_network_update: dict
        :return: All the agents of the network
        :rtype: list
        """
        agents = []
        for intersection in self.relations:
            agent = RLAgent2(id_intersection=intersection,
                             id_tls_program=self.relations[intersection]['node_infos'].tl,
                             yellow_time=yellow_times[intersection],
                             period_duration=period_durations[intersection],
                             intersection_relations=self.relations[intersection],
                             epsilon=epsilons[intersection],
                             epsilon_updater=epsilon_updaters[intersection],
                             min_epsilon=min_epsilons[intersection],
                             decreasing_episode=decreasing_episode,
                             episode_length=episode_length,
                             max_batch_size=max_batch_size,
                             gamma=gammas[intersection],
                             nb_epochs=nbs_epochs[intersection],
                             frequence_target_network_update=frequences_target_network_update[intersection])
            agents.append(agent)
        return agents

    def _remove_vehicles(self):
        """
        Remove all vehicles from simulation.
        :return: None
        """
        for vehicle_id in traci.vehicle.getIDList():
            traci.vehicle.remove(vehicle_id)
