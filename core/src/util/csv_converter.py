import numpy as np

def import_flows_parameters_from_csv(file, delimiter=','):
    """
        Fonction permettant d'importer la matrice de coefficients et le vecteur de charge pour les expériences à flux variables.

        :param file: Le chemin du fichier.
    """

    data = np.genfromtxt(file, delimiter=delimiter)
    vecteur_charge = data[0]
    coeff_matrix = data[1:]
    return vecteur_charge, coeff_matrix