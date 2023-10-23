import traci
import numpy as np


def get_nb_vehicles(config):
    """
    Return the number of running vehicles on the network.
    :param config: Configuration data for the function
    :type config: dict
    :return: A dictionary with number of vehicle value
    :rtype: dict
    """
    vehicles = traci.vehicle.getIDList()
    res = {
        'nb_running_vehicles': len(vehicles)
    }
    return res

def get_acceleration_data(config):
    """
    Return the mean and sum of all running vehicles acceleration on the network.
    :param config: Configuration data for the function
    :type config: dict
    :return: A dictionary with acceleration values
    :rtype: dict
    """
    accels = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        accels.append(traci.vehicle.getAcceleration(vehicle))
    res = {
        'mean_acceleration': np.mean(accels)
    }
    return res


def get_speed_data(config):
    """
    Return the mean and sum of all running vehicles speed on the network.
    :param config: Configuration data for the function
    :type config: dict
    :return: A dictionary with speed values
    :rtype: dict
    """
    speeds = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        speeds.append(traci.vehicle.getSpeed(vehicle))
    res = {
        'mean_speed': np.mean(speeds)
    }
    return res
