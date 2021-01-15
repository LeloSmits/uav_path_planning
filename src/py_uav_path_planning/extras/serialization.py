import numpy as np


def serialize_coordinates(array_2d):
    # type: (np.ndarray) -> np.ndarray
    array_1d = np.array(array_2d).ravel()
    return array_1d


def deseralize_coordinates(array_1d, n_pts):
    # type: (np.ndarray, int) -> np.ndarray
    array_1d = np.array(array_1d)
    array_size = len(array_1d)
    if not array_size % n_pts == 0:
        raise Warning("Array size must be a clean multiple of n_pts. Array length: {0}, n_pts: {1}".format(array_size, n_pts))
    array_2d = array_1d.reshape((array_size/n_pts, n_pts))
    return array_2d
