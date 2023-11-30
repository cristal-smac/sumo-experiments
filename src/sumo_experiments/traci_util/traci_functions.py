import traci
import numpy as np


def get_nb_vehicles():
    """
    Return the number of running vehicles on the network.
    :return: A dictionary with number of vehicle value
    :rtype: dict
    """
    count = traci.vehicle.getIDCount()
    res = {
        'nb_running_vehicles': count
    }
    return res


def get_acceleration_data():
    """
    Return acceleration data for all running vehicles on the network.
    :return: A dictionary with acceleration values
    :rtype: dict
    """
    accels = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        accels.append(traci.vehicle.getAcceleration(vehicle))
    res = {
        'mean_acceleration': np.mean(accels) if len(accels) > 0 else np.nan,
        'sum_acceleration': np.sum(accels) if len(accels) > 0 else np.nan,
        'max_acceleration': np.max(accels) if len(accels) > 0 else np.nan,
        'min_acceleration': np.min(accels) if len(accels) > 0 else np.nan,
        'std_dev_acceleration': np.std(accels) if len(accels) > 0 else np.nan
    }
    return res


def get_speed_data():
    """
    Return speed data for all running vehicles on the network.
    :return: A dictionary with speed values
    :rtype: dict
    """
    speeds = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        speeds.append(traci.vehicle.getSpeed(vehicle))
    res = {
        'mean_speed': np.mean(speeds) if len(speeds) > 0 else np.nan,
        'sum_speed': np.sum(speeds) if len(speeds) > 0 else np.nan,
        'max_speed': np.max(speeds) if len(speeds) > 0 else np.nan,
        'min_speed': np.min(speeds) if len(speeds) > 0 else np.nan,
        'std_dev_speed': np.std(speeds) if len(speeds) > 0 else np.nan
    }
    return res


def get_co2_emissions_data():
    """
    Return CO2 emissions data for all running vehicles on the network.
    :return: A dictionary with CO2 values
    :rtype: dict
    """
    emissions = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        emissions.append(traci.vehicle.getCO2Emission(vehicle))
    res = {
        'mean_co2_emissions': np.mean(emissions) if len(emissions) > 0 else np.nan,
        'sum_co2_emissions': np.sum(emissions) if len(emissions) > 0 else np.nan,
        'max_co2_emissions': np.max(emissions) if len(emissions) > 0 else np.nan,
        'min_co2_emissions': np.min(emissions) if len(emissions) > 0 else np.nan,
        'std_dev_co2_emissions': np.std(emissions) if len(emissions) > 0 else np.nan
    }
    return res


def get_co_emissions_data():
    """
    Return CO emissions data for all running vehicles on the network.
    :return: A dictionary with CO values
    :rtype: dict
    """
    emissions = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        emissions.append(traci.vehicle.getCOEmission(vehicle))
    res = {
        'mean_co_emissions': np.mean(emissions) if len(emissions) > 0 else np.nan,
        'sum_co_emissions': np.sum(emissions) if len(emissions) > 0 else np.nan,
        'max_co_emissions': np.max(emissions) if len(emissions) > 0 else np.nan,
        'min_co_emissions': np.min(emissions) if len(emissions) > 0 else np.nan,
        'std_dev_co_emissions': np.std(emissions) if len(emissions) > 0 else np.nan
    }
    return res


def get_nox_emissions_data():
    """
    Return NOx emissions data for all running vehicles on the network.
    :return: A dictionary with NOx values
    :rtype: dict
    """
    emissions = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        emissions.append(traci.vehicle.getNOxEmission(vehicle))
    res = {
        'mean_nox_emissions': np.mean(emissions) if len(emissions) > 0 else np.nan,
        'sum_nox_emissions': np.sum(emissions) if len(emissions) > 0 else np.nan,
        'max_nox_emissions': np.max(emissions) if len(emissions) > 0 else np.nan,
        'min_nox_emissions': np.min(emissions) if len(emissions) > 0 else np.nan,
        'std_dev_nox_emissions': np.std(emissions) if len(emissions) > 0 else np.nan
    }
    return res


def get_fuel_consumption_data():
    """
    Return fuel consumption data for all running vehicles on the network.
    :return: A dictionary with speed values
    :rtype: dict
    """
    fuel_consumption = []
    vehicles = traci.vehicle.getIDList()
    for vehicle in vehicles:
        fuel_consumption.append(traci.vehicle.getFuelConsumption(vehicle))
    res = {
        'mean_fuel_consumption': np.mean(fuel_consumption) if len(fuel_consumption) > 0 else np.nan,
        'sum_fuel_consumption': np.sum(fuel_consumption) if len(fuel_consumption) > 0 else np.nan,
        'max_fuel_consumption': np.max(fuel_consumption) if len(fuel_consumption) > 0 else np.nan,
        'min_fuel_consumption': np.min(fuel_consumption) if len(fuel_consumption) > 0 else np.nan,
        'std_dev_fuel_consumption': np.std(fuel_consumption) if len(fuel_consumption) > 0 else np.nan
    }
    return res