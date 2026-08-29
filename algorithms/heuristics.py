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
    
    # primero revisamos si estamos en un estado meta
    if state[0] == problem.controlPosition and len(state[2]) == 0:
        return 0
    
    estimado = 0
    
    """
    Vamos a usar MST como heuristica entonces: 
        - Vamos a hacer una matriz con las distancias de cada punto a los demas
        - Luego haremos un ciclo donde buscamos el costo mas pequeno que conecte
          los nodos y sumamos todos los costos
    """
    
    #pongamos todos los puntos en una lista
    
    # puntosAVisitar es una lista de tuplas que indican la posicion de todos
    # los elementos que tenemos que visitar.
    # comienza con la posicion actual del robot
    puntosAVisitar = [] 
    # si no tenemos el kit debemos buscarlo primero.
    if state[1] == False: 
        estimado += abs(state[0][0]-problem.kitPosition[0]) + abs(state[0][1]-problem.kitPosition[1])
        puntosAVisitar.append(problem.kitPosition)
    else:
        puntosAVisitar.append(state[0])
    
    # ahora agregamops las posiciones de todos los T.
    for posicion in state[2]:
        puntosAVisitar.append(posicion)
    
    # finalmente tenemos que agregar el destino final, el centro de control.
    puntosAVisitar.append(problem.controlPosition)
        
    tamano = len(puntosAVisitar)
    
    #Creamos un arreglo donde tendremos las distancias de cada nodo a todos los 
    #demas.
    matriz = [[0]*tamano for i in range(tamano)]
    
    for fila in range(tamano):
        for columna in range(tamano):
            if fila == columna:
                matriz[fila][columna] = 0
            else:
                puntoFila=puntosAVisitar[fila]
                puntoColumna=puntosAVisitar[columna]
                distanciaManhattan = abs(puntoFila[0]-puntoColumna[0]) + abs(puntoFila[1]-puntoColumna[1])
                matriz[fila][columna] = distanciaManhattan
    
    # ahora vamos a buscar el camino mas barato
    
    # crearemos dos listas para saber cuales nodos ya estan conectados
    conectados = [puntosAVisitar[0]]
    pendientes = puntosAVisitar.copy()[1:]
    
    #vamos a buscar todas las conexiones entre nodos con los caminos mas cortos
    while len(pendientes)>0:
        minimo = 100000
        posMin = 10000
        
        for nodoConectado in conectados:
            indice = puntosAVisitar.index(nodoConectado)
            costos = matriz[indice]
            for nodo in pendientes:
                pos = puntosAVisitar.index(nodo)
                if costos[pos]<minimo:
                    minimo = costos[pos]
                    posMin = pos
        
        estimado += minimo
        conectados.append(puntosAVisitar[posMin])
        pendientes.remove(puntosAVisitar[posMin])
        
    
    return estimado



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

# =====================================================
# VERSION MEJORADA CON AYUDA DE IA (COMENTADA)
# =====================================================

# A continuacion se deja la version refactorizada sugerida
# por la IA para systemRepairHeuristic:
#
# 1) Utiliza un Minimum Spanning Tree (MST) para estimar
#    el costo minimo necesario para conectar todos los
#    puntos que todavia deben ser visitados.
#
# 2) Utiliza distancia Manhattan entre los puntos, ya que
#    esta representa una cota inferior del costo real de
#    desplazamiento en los mapas.
#
# 3) Si el robot todavia no tiene el kit, primero se agrega
#    la distancia entre la posicion actual y K. El MST
#    comienza desde K porque el robot debe recoger el kit
#    antes de reparar los sistemas.
#
# 4) Si el robot ya tiene el kit, el MST comienza desde la
#    posicion actual.
#
# 5) Se utiliza el algoritmo de Prim para construir el MST.
#    En lugar de mantener listas de nodos conectados y
#    pendientes, se utilizan arreglos de indices para
#    reducir busquedas innecesarias.
#
# 6) La heuristica retorna 0 cuando el estado ya es un
#    estado meta.
#
#
# def systemRepairHeuristic(
#     state: Tuple[Tuple, bool, Tuple],
#     problem: SystemRepairProblem
# ):
#     """
#     Heuristica para SystemRepairProblem.
#
#     Utiliza distancia Manhattan y un Minimum Spanning Tree
#     para estimar el costo restante de la mision.
#     """
#
#     position, hasKit, pendingSystems = state
#
#     # Si el robot ya reparo todos los sistemas y llego
#     # al centro de control, no queda ningun costo.
#     if position == problem.controlPosition and not pendingSystems:
#         return 0
#
#     estimado = 0
#
#     # -------------------------------------------------
#     # Construccion de los puntos que deben ser visitados
#     # -------------------------------------------------
#
#     puntosAVisitar = []
#
#     # Si todavia no tenemos el kit, primero debemos
#     # desplazarnos hasta K.
#     if not hasKit:
#
#         distanciaKit = (
#             abs(position[0] - problem.kitPosition[0])
#             + abs(position[1] - problem.kitPosition[1])
#         )
#
#         estimado += distanciaKit
#
#         # El MST comienza desde el kit.
#         puntosAVisitar.append(problem.kitPosition)
#
#     else:
#
#         # Si ya tenemos el kit, el MST comienza desde
#         # la posicion actual.
#         puntosAVisitar.append(position)
#
#     # Agregamos todos los sistemas que todavia estan
#     # pendientes de reparacion.
#     for system in pendingSystems:
#         puntosAVisitar.append(system)
#
#     # El robot debe terminar en el centro de control.
#     puntosAVisitar.append(problem.controlPosition)
#
#     tamano = len(puntosAVisitar)
#
#     # -------------------------------------------------
#     # Construccion de la matriz de distancias
#     # -------------------------------------------------
#
#     matriz = [[0] * tamano for _ in range(tamano)]
#
#     for fila in range(tamano):
#
#         for columna in range(tamano):
#
#             puntoFila = puntosAVisitar[fila]
#             puntoColumna = puntosAVisitar[columna]
#
#             matriz[fila][columna] = (
#                 abs(puntoFila[0] - puntoColumna[0])
#                 + abs(puntoFila[1] - puntoColumna[1])
#             )
#
#     # -------------------------------------------------
#     # Construccion del MST utilizando Prim
#     # -------------------------------------------------
#
#     # distancias[i] representa el costo minimo conocido
#     # para conectar el nodo i al MST.
#     distancias = [float("inf")] * tamano
#
#     # El primer nodo comienza conectado.
#     distancias[0] = 0
#
#     # Indica si cada nodo ya pertenece al MST.
#     conectados = [False] * tamano
#
#     costoMST = 0
#
#     for _ in range(tamano):
#
#         minimo = float("inf")
#         nodoMin = -1
#
#         # Buscamos el nodo no conectado cuya conexion
#         # al MST sea la mas barata.
#         for i in range(tamano):
#
#             if not conectados[i] and distancias[i] < minimo:
#
#                 minimo = distancias[i]
#                 nodoMin = i
#
#         # Agregamos el nodo seleccionado al MST.
#         conectados[nodoMin] = True
#         costoMST += minimo
#
#         # Actualizamos las distancias de los nodos que
#         # todavia no pertenecen al MST.
#         for i in range(tamano):
#
#             if (
#                 not conectados[i]
#                 and matriz[nodoMin][i] < distancias[i]
#             ):
#
#                 distancias[i] = matriz[nodoMin][i]
#
#     # Sumamos el costo del MST al costo necesario para
#     # llegar al kit, si este todavia no habia sido recogido.
#     estimado += costoMST
#
#     return estimado
#
# =====================================================

