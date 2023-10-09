import numpy as np

def import_flows_parameters_from_csv(file, delimiter=','):
    """
    Load the load vector and coff matrix from a csv file, to be used with flows generated with a matrix.
    The load vector and the matrix are of type numpy.array.
    :param file: The path to the file where flows data are stored
    :type file: str
    :param delimiter: Separation character between columns in the CSV file
    :type delimiter: str
    :return: A tuple containing the load vector in first position, and the matrix in second position.
    :rtype: tuple
    """
    data = np.genfromtxt(file, delimiter=delimiter)
    load_vector = data[0]
    coeff_matrix = data[1:]
    return load_vector, coeff_matrix