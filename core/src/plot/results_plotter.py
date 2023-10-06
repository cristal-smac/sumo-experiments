import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os


class ResultsPlotter:
    """
    The DetectorBuilder class can be used to plot the results of a series of experiments. Experiments results must be
    saved in a CSV file, with the 'export_results_to_csv' function, to be read by the plotter.
    """

    def __init__(self, file, sampling_rate, max_ticks, save_folder):
        """
        Init of class.
        :param file: Path to the csv file where data are stored
        :type file: str
        :param sampling_rate: Data scaling in the CSV file (in simulation steps)
        :type sampling_rate: int
        :param max_ticks: Final number of simulation steps
        :type max_ticks: int
        :param save_folder: Path to the folder where plots will be saved
        :type save_folder: str
        """
        self.file = file
        self.df = pd.read_csv(file, )
        self.max_ticks = max_ticks
        self.all_samples = np.arange(start=sampling_rate, stop=max_ticks, step=sampling_rate).tolist()
        self.save_folder = save_folder

    def plot_mean_waiting_time_n_worst(self, n=5):
        """
        Plot the mean waiting time of the vehicles for the n worst settings.
        The n worsts settings are those who maximize the mean waiting time.
        :param n: The number of worst experiments to plot.
        :type n: int
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['exp_name'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanWaitingTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, -n)[-n:]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Average waiting time as a function of time ({n} worsts)"
        ax.set_ylabel("Average waiting time (in s)")
        ax.set_xlabel("Simulation steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_worst_mean_waiting_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_worst_mean_waiting_time")

    def plot_mean_waiting_time_n_best(self, n=5):
        """
        Plot the mean waiting time of the vehicles for the n best settings.
        The n bests settings are those who minimize the mean waiting time.
        :param n: The number of best experiments to plot.
        :type n: int
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['exp_name'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanWaitingTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, n)[:n]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Average waiting time as a function of time ({n} bests)"
        ax.set_ylabel("Average waiting time (in s)")
        ax.set_xlabel("Simulation steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_best_mean_waiting_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_best_mean_waiting_time")

    def plot_mean_travel_time_n_worst(self, n=5):
        """
        Plot the mean travel time of the vehicles for the n worst settings.
        The n worsts settings are those who maximize the mean travel time.
        :param n: The number of worst experiments to plot.
        :type n: int
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['exp_name'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanTravelTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, -n)[-n:]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Average travel time as a function of time ({n} worsts)"
        ax.set_ylabel("Average travel time (in s)")
        ax.set_xlabel("Simulation steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_worst_mean_travel_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_worst_mean_travel_time")

    def plot_mean_travel_time_n_best(self, n=5):
        """
        Plot the mean travel time of the vehicles for the n best settings.
        The n bests settings are those who minimize the mean travel time.
        :param n: The number of best experiments to plot.
        :type n: int
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['exp_name'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanTravelTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, n)[:n]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Average travel time as a function of time ({n} bests)"
        ax.set_ylabel("Average travel time (in s)")
        ax.set_xlabel("Simulation steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_best_mean_travel_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_best_mean_travel_time")

    def plot_mean_speed_n_worst(self, n=5):
        """
        Plot the mean speed of the vehicles for the n worst settings.
        The n worsts settings are those who minimize the mean speed.
        :param n: The number of worst experiments to plot.
        :type n: int
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['exp_name'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanSpeed_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, n)[:n]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Average speed as a function of time ({n} worsts)"
        ax.set_ylabel("Average speed (in m/s)")
        ax.set_xlabel("Simulation steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_worst_mean_speed")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_worst_mean_speed")

    def plot_mean_speed_n_best(self, n=5):
        """
        Plot the mean travel time of the vehicles for the n best settings.
        The n bests settings are those who minimize the mean travel time.
        :param n: The number of best experiments to plot.
        :type n: int
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['exp_name'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanSpeed_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, -n)[-n:]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Average speed as a function of time ({n} bests)"
        ax.set_ylabel("Average speed (in m/s)")
        ax.set_xlabel("Simulation steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_best_mean_speed")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_best_mean_speed")

    def plot_all(self, n_best, n_worst):
        """
        Plot all graphs of the class.
        :param n_best: The number of best experiments to plot.
        :type n_best: int
        :param n_worst: The number of worst experiments to plot.
        :type n_worst: int
        """
        self.plot_mean_speed_n_best(n_best)
        self.plot_mean_speed_n_worst(n_worst)
        self.plot_mean_travel_time_n_best(n_best)
        self.plot_mean_travel_time_n_worst(n_worst)
        self.plot_mean_waiting_time_n_best(n_best)
        self.plot_mean_waiting_time_n_worst(n_worst)
