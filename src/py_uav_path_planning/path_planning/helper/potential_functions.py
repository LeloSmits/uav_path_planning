import numpy as np
import typing

close_to_zero_tol = 0.1


def attractive_potential_function(X, Xdes, ka, der=False):
    # type: (np.array, np.array, float, bool) -> typing.Union[float, np.array]
    """:arg X: np.array of shape (samples x dimension) or (dimension)
    :arg Xdes: np.array of shape (dimension)
    :arg ka: scalar
    :arg der: boolean, default: False"""

    if len(X.shape) == 1:  # Only one X-value: [x, y, z]
        dist = np.linalg.norm(X - Xdes)
        if not der:
            return .5 * ka * dist
        else:
            if not np.isclose(dist, 0, atol=close_to_zero_tol):
                return ka * (X - Xdes) / dist
            else:
                return 0

    else:  # multiple X-values: [[1,2,1],[3,8,7],....]
        dist = np.linalg.norm(X - Xdes, axis=1)

        if not der:
            return .5 * ka * dist
        else:
            return ka * (1/dist)[:, np.newaxis] * (X - Xdes)


def obstacle_potential_function(X, Xobs, kr, rho0, der=False):
    # type: (np.array, np.array, float, float, bool) -> typing.Union[float, np.array]
    """:arg X: np.array of shape (samples x dimension) or (dimension)
    :arg Xobs: np.array of shape (dimension)
    :arg kr: scalar
    :arg rho0: scalar
    :arg der: boolean, default: False"""

    if len(X.shape) == 1:  # Only one X-value: [x, y, z]
        dist = np.linalg.norm(X - Xobs)
        if not der:
            if not np.isclose(dist, 0, atol=close_to_zero_tol):
                return (dist < rho0 ** 2) * (.5 * kr * (1 / dist - 1 / rho0) ** 2)  # Set to 0 if distance larger than influence
            else:
                return np.inf
        else:
            if not np.isclose(dist, 0, atol=close_to_zero_tol):
                res = -kr * 1 / dist ** 3 * (1 / dist - 1 / rho0) * (X - Xobs)
                return res * (dist < rho0**2)
            else:
                return np.full(X.shape, np.inf)

    else:  # multiple X-values: [[1,2,1],[3,8,7],....]
        dist = np.linalg.norm(X - Xobs, axis=1)

        if not der:
            res = .5 * kr * (1 / dist - 1 / rho0) ** 2
            res[np.where(dist > rho0**2)] = 0  # if outside of influence, set pot to 0
            return res
        else:
            res = -kr * (1/dist**3 * (1/dist - 1/rho0))[:, np.newaxis] * (X - Xobs)  # ToDo: Check if derivative is correct
            res[np.where(dist > rho0 ** 2), :] = 0
            return res

