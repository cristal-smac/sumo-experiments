import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

"""
    Fichier contenant une classe 'Plotter' servant à dessiner les résultats d'une série d'expérience.
"""


class ResultsPlotter:
    """
    Classe servant à plotter les résultats d'un ensemble d'expérience.
    """

    def __init__(self, file, sampling_rate, max_ticks, save_folder):
        self.file = file
        self.df = pd.read_csv(file, )
        self.sampling_rate = sampling_rate
        self.max_ticks = max_ticks
        self.all_samples = np.arange(start=sampling_rate, stop=max_ticks, step=sampling_rate).tolist()
        self.save_folder = save_folder

    def plot_meanWaitingTime_n_worst(self, n=5):
        """
        Trace les courbes des n meilleurs réglages qui optimisent le temps d'attente des véhicules.
        Les n meilleurs réglages sont ceux qui minimisent en moyenne le temps d'attente.
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['nom_exp'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanWaitingTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, -n)[-n:]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Evolution du temps d'attente moyen en fonction du temps ({n} pires)"
        ax.set_ylabel("Temps d'attente moyen (en s)")
        ax.set_xlabel("Nombre de steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_worst_mean_waiting_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_worst_mean_waiting_time")

    def plot_meanWaitingTime_n_best(self, n=5):
        """
        Trace les courbes des n meilleurs réglages qui optimisent le temps d'attente des véhicules.
        Les n meilleurs réglages sont ceux qui minimisent en moyenne le temps d'attente.
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['nom_exp'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanWaitingTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, n)[:n]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Evolution du temps d'attente moyen en fonction du temps ({n} meilleurs)"
        ax.set_ylabel("Temps d'attente moyen (en s)")
        ax.set_xlabel("Nombre de steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_best_mean_waiting_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_best_mean_waiting_time")

    def plot_meanTravelTime_n_worst(self, n=5):
        """
        Trace les courbes des n meilleurs réglages qui optimisent le temps de trajet des véhicules.
        Les n meilleurs réglages sont ceux qui minimisent en moyenne le temps de trajet.
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['nom_exp'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanTravelTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, -n)[-n:]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Evolution du temps de trajet moyen en fonction du temps ({n} pires)"
        ax.set_ylabel("Temps de trajet moyen (en s)")
        ax.set_xlabel("Nombre de steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_worst_mean_travel_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_worst_mean_travel_time")

    def plot_meanTravelTime_n_best(self, n=5):
        """
        Trace les courbes des n meilleurs réglages qui optimisent le temps de trajet des véhicules.
        Les n meilleurs réglages sont ceux qui minimisent en moyenne le temps de trajet.
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['nom_exp'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanTravelTime_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, n)[:n]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Evolution du temps de trajet moyen en fonction du temps ({n} meilleurs)"
        ax.set_ylabel("Temps de trajet moyen (en s)")
        ax.set_xlabel("Nombre de steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_best_mean_travel_time")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_best_mean_travel_time")

    def plot_meanSpeed_n_worst(self, n=5):
        """
        Trace les courbes des n meilleurs réglages qui optimisent la vitesse des véhicules.
        Les n meilleurs réglages sont ceux qui minimisent en moyenne la vitesse.
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['nom_exp'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanSpeed_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, n)[:n]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Evolution de la vitesse moyenne en fonction du temps ({n} pires)"
        ax.set_ylabel("Vitesse moyenne (en m/s)")
        ax.set_xlabel("Nombre de steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_worst_mean_speed")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_worst_mean_speed")

    def plot_meanSpeed_n_best(self, n=5):
        """
        Trace les courbes des n meilleurs réglages qui optimisent la vitesse des véhicules.
        Les n meilleurs réglages sont ceux qui minimisent en moyenne la vitesse.
        """
        fig, ax = plt.subplots()
        noms_experiences = self.df['nom_exp'].to_numpy()
        results = np.zeros((self.df.shape[0], 1))
        for step in self.all_samples:
            colonne = self.df[f'meanSpeed_{step}']
            results = np.hstack((results, colonne.to_numpy().reshape(-1, 1)))
        moyennes = np.mean(results, 1)
        ind = np.argpartition(moyennes, -n)[-n:]
        for i in range(n):
            ax.plot([0] + self.all_samples, results[ind[i]], label=noms_experiences[ind[i]])
        fig.title = f"Evolution de la vitesse moyenne en fonction du temps ({n} meilleurs)"
        ax.set_ylabel("Vitesse moyenne (en m/s)")
        ax.set_xlabel("Nombre de steps")
        ax.legend()
        if os.path.isdir(self.save_folder):
            fig.savefig(self.save_folder + f"/{n}_best_mean_speed")
        else:
            os.mkdir(self.save_folder)
            fig.savefig(self.save_folder + f"/{n}_best_mean_speed")

    def plot_all(self, n_best, n_worst):
        self.plot_meanSpeed_n_best(n_best)
        self.plot_meanSpeed_n_worst(n_worst)
        self.plot_meanTravelTime_n_best(n_best)
        self.plot_meanTravelTime_n_worst(n_worst)
        self.plot_meanWaitingTime_n_best(n_best)
        self.plot_meanWaitingTime_n_worst(n_worst)