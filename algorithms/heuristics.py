from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem
from math import sqrt


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    # TODO: Add your code here
    position, hasKit, pendingSystems = state

    if not hasKit:
        target = problem.kitPosition

        return (abs(position[0] - target[0])+ abs(position[1] - target[1]))

    if len(pendingSystems) > 0:
        return min(abs(position[0] - system[0]) + abs(position[1] - system[1]) for system in pendingSystems)

    target = problem.controlPosition

    return (abs(position[0] - target[0]) + abs(position[1] - target[1]))


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    # TODO: Add your code here
    position, hasKit, pendingSystems = state

    if not hasKit:
        target = problem.kitPosition

        dx = position[0] - target[0]
        dy = position[1] - target[1]

        return sqrt(dx ** 2 + dy ** 2)

    if len(pendingSystems) > 0:
        return min(
            sqrt((position[0] - system[0]) ** 2 + (position[1] - system[1]) ** 2) for system in pendingSystems)

    target = problem.controlPosition

    dx = position[0] - target[0]
    dy = position[1] - target[1]

    return sqrt(dx ** 2 + dy ** 2)


def systemRepairHeuristic(
    state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem
):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    # TODO: Add your code here
    utils.raiseNotDefined()



# =====================================================
# VERSION MEJORADA CON AYUDA DE IA (COMENTADA)
# =====================================================
# A continuacion se dejan las versiones refactorizadas sugeridas
# por la IA para manhattanHeuristic y euclideanHeuristic:
#
# 1) Encapsulan el calculo de distancia en funciones auxiliares locales
#    ('manhattan' y 'euclidean') para mejorar la legibilidad y evitar codigo duplicado.
# 2) Simplifican la comprobacion de tuplas con 'if pendingSystems:' en lugar
#    de verificar explicitamente su longitud.
#
# def manhattanHeuristic(state, problem):
#     """
#     The Manhattan distance heuristic.
#
#     Baseline rule for this workshop: estimate the direct distance to the next
#     mandatory target:
#     - K if the robot does not have the kit yet.
#     - the nearest pending T if the robot has the kit and systems remain.
#     - C if all systems have been repaired.
#     """
#     position, hasKit, pendingSystems = state
#
#     def manhattan(a, b):
#         return abs(a[0] - b[0]) + abs(a[1] - b[1])
#
#     if not hasKit:
#         return manhattan(position, problem.kitPosition)
#
#     if pendingSystems:
#         return min(manhattan(position, system) for system in pendingSystems)
#
#     return manhattan(position, problem.controlPosition)
#
#
# def euclideanHeuristic(state, problem):
#     """
#     The Euclidean distance heuristic.
#
#     Baseline rule for this workshop: estimate the direct distance to the next
#     mandatory target:
#     - K if the robot does not have the kit yet.
#     - the nearest pending T if the robot has the kit and systems remain.
#     - C if all systems have been repaired.
#     """
#     position, hasKit, pendingSystems = state
#
#     def euclidean(a, b):
#         return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
#
#     if not hasKit:
#         return euclidean(position, problem.kitPosition)
#
#     if pendingSystems:
#         return min(euclidean(position, system) for system in pendingSystems)
#
#     return euclidean(position, problem.controlPosition)
# =====================================================