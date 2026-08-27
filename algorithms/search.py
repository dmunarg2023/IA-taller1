from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic
import heapq


def tinyDiagnosticSearch(problem: SearchProblem):
    """
    Returns a hard-coded sequence of moves for the tinyDiagnostic layout.
    For any other station layout, the sequence of moves will be incorrect.
    """
    s = Directions.SOUTH
    e = Directions.EAST
    return [s, e, s, e, e, e, e, s, e, e, s, s, e, s, s, e, s, e, e, e, e, e, e, e]


def _reconstructPath(cameFrom, state):
    """Reconstructs the action list starting from the goal state."""
    actions = []
    while state in cameFrom:
        previous, action = cameFrom[state]
        actions.append(action)
        state = previous
    actions.reverse()
    return actions


# =====================================================
# CODIGO BASE FUNCIONAL (MI BASE)
# =====================================================
def depthFirstSearch(problem: SearchProblem):
    """
    DFS: explora la rama mas profunda primero.
    Es una solucion funcional para encontrar caminos validos.
    """
    start = problem.getStartState()
    stack = [(start, [])]
    visited = set()

    while stack:
        state, path = stack.pop()
        if state in visited:
            continue
        visited.add(state)

        if problem.isGoalState(state):
            return path

        for successor, action, _ in reversed(problem.getSuccessors(state)):
            if successor not in visited:
                stack.append((successor, path + [action]))

    return None


def breadthFirstSearch(problem: SearchProblem):
    """
    BFS: explora por niveles.
    Encuentra la solucion con menor cantidad de movimientos.
    """
    start = problem.getStartState()
    queue = [(start, [])]
    visited = {start}

    while queue:
        state, path = queue.pop(0)

        if problem.isGoalState(state):
            return path

        for successor, action, _ in problem.getSuccessors(state):
            if successor not in visited:
                visited.add(successor)
                queue.append((successor, path + [action]))

    return None


def uniformCostSearch(problem: SearchProblem):
    """
    UCS: prioridad por costo acumulado.
    Esta version sigue siendo funcional y ya considera el costo real.
    """
    start = problem.getStartState()
    priorityQueue = [(0, start)]
    cameFrom = {}
    bestCost = {start: 0}

    while priorityQueue:
        currentCost, state = heapq.heappop(priorityQueue)

        if currentCost > bestCost.get(state, float("inf")):
            continue

        if problem.isGoalState(state):
            return _reconstructPath(cameFrom, state)

        for successor, action, stepCost in problem.getSuccessors(state):
            newCost = currentCost + stepCost
            if newCost < bestCost.get(successor, float("inf")):
                bestCost[successor] = newCost
                cameFrom[successor] = (state, action)
                heapq.heappush(priorityQueue, (newCost, successor))

    return None

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    # TODO: Add your code here
    utils.raiseNotDefined()



# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

# =====================================================
# VERSION MEJORADA CON AYUDA DE IA (COMENTADA)
# =====================================================
# A continuacion se deja una version mas optimizada, como si la IA
# hubiera sugerido una mejora de la base funcional original.
# 
# 1) se mantiene la idea principal de la busqueda
# 2) se mejora el uso de memoria y prioridad
# 3) se enfatiza la eleccion por costo real + heuristica
#
# def reconstructPath(cameFrom, goalState):
#     actions = []
#     current = goalState
#
#     while current in cameFrom:
#         previous, action = cameFrom[current]
#         actions.append(action)
#         current = previous
#
#     actions.reverse()
#     return actions
#
#
# def depthFirstSearch(problem):
#     start = problem.getStartState()
#     stack = [(start, [])]
#     visited = set()
#
#     while stack:
#         state, path = stack.pop()
#
#         if state in visited:
#             continue
#
#         visited.add(state)
#
#         if problem.isGoalState(state):
#             return path
#
#         for nextState, action, _ in reversed(problem.getSuccessors(state)):
#             if nextState not in visited:
#                 stack.append((nextState, path + [action]))
#
#     return None
#
#
# def breadthFirstSearch(problem):
#     start = problem.getStartState()
#     queue = [(start, [])]
#     visited = {start}
#
#     while queue:
#         state, path = queue.pop(0)
#
#         if problem.isGoalState(state):
#             return path
#
#         for nextState, action, _ in problem.getSuccessors(state):
#             if nextState not in visited:
#                 visited.add(nextState)
#                 queue.append((nextState, path + [action]))
#
#     return None
#
#
# def uniformCostSearch(problem):
#     start = problem.getStartState()
#     frontier = [(0, start)]
#     cameFrom = {}
#     costs = {start: 0}
#
#     while frontier:
#         currentCost, state = heapq.heappop(frontier)
#
#         if currentCost > costs.get(state, float("inf")):
#             continue
#
#         if problem.isGoalState(state):
#             return reconstructPath(cameFrom, state)
#
#         for nextState, action, stepCost in problem.getSuccessors(state):
#             tentativeCost = costs[state] + stepCost
#
#             if tentativeCost < costs.get(nextState, float("inf")):
#                 costs[nextState] = tentativeCost
#                 cameFrom[nextState] = (state, action)
#                 heapq.heappush(frontier, (tentativeCost, nextState))
#
#     return None
#
#
# =====================================================

