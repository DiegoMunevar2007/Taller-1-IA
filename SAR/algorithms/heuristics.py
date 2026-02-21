from typing import Any, Tuple
from algorithms.utils import PriorityQueue
from algorithms.problems import MultiSurvivorProblem


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.
    """
    return abs(state[0] - problem.goal[0]) + abs(state[1] - problem.goal[1])


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.
    """
    return ((state[0] - problem.goal[0]) ** 2 + (state[1] - problem.goal[1]) ** 2) ** 0.5

# def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
#     """
#     Your heuristic for the MultiSurvivorProblem.

#     state: (position, survivors_grid)
#     problem: MultiSurvivorProblem instance

#     This must be admissible and preferably consistent.

#     Hints:
#     - Use problem.heuristicInfo to cache expensive computations
#     - Go with some simple heuristics first, then build up to more complex ones
#     - Consider: distance to nearest survivor + MST of remaining survivors
#     - Balance heuristic strength vs. computation time (do experiments!)
#     """
#     posicion, survivors_grid = state
    
#     if problem.heuristicInfo == {}: # Sacar las posiciones de todos los supervivientes y guardarlas en una PriorityQueue ordenada por distancia Manhattan a la posición actual
#         problem.heuristicInfo['survivor_positions'] = PriorityQueue()
        
#         for i in range(survivors_grid.width):
#             for j in range(survivors_grid.height):
#                 if survivors_grid[i][j]:  # Si hay un superviviente en esta posición
#                     distancia_manhattan = abs(posicion[0] - i) + abs(posicion[1] - j)
#                     problem.heuristicInfo['survivor_positions'].push((i, j), distancia_manhattan)
                    
#     else: # Actualizar las distancias Manhattan de los supervivientes restantes a la posición actual
#         survivor_positions = problem.heuristicInfo['survivor_positions']
#         nueva_pq = PriorityQueue()
        
#         while not survivor_positions.isEmpty():
#             survivor_pos = survivor_positions.pop()
#             print(survivor_pos)
#             if verificar_existencia(survivor_pos, survivors_grid):
#                 nueva_pq.push(survivor_pos, abs(posicion[0] - survivor_pos[0]) + abs(posicion[1] - survivor_pos[1]))
                
#         problem.heuristicInfo['survivor_positions'] = nueva_pq
        
#     distancia_al_superviviente_mas_cercano = problem.heuristicInfo['survivor_positions'].pop()[1] if not problem.heuristicInfo['survivor_positions'].isEmpty() else 0
#     return distancia_al_superviviente_mas_cercano
            
def crear_matriz_distancias(sobrevivientes, problem):
    """
    Calcula una vez todas las distancias entre sobrevivientes.
    """
    matriz = {}

    for i in range(len(sobrevivientes)):
        for j in range(i + 1, len(sobrevivientes)):
            s1 = sobrevivientes[i]
            s2 = sobrevivientes[j]

            # suponiendo que tienes una funcion que calcula distancia real
            dist = mazeDistance(s1, s2, problem.startingGameState)

            matriz[(s1, s2)] = dist
            matriz[(s2, s1)] = dist  # simetrico

    return matriz

def mst_desde_grid(grid, cache):
    """
    Calcula el MST usando solo las posiciones activas (True) del grid.
    """

    if grid in cache:
        return cache[grid]

    sobrevivientes = grid.asList(True)

    if len(sobrevivientes) <= 1:
        cache[grid] = 0
        return 0

    visitados = []
    total = 0

    primero = sobrevivientes[0]
    visitados.append(primero)

    while len(visitados) < len(sobrevivientes):
        mejor = float("inf")
        mejor_nodo = None

        for v in visitados:
            for u in sobrevivientes:
                if u not in visitados:
                    d = distancia_manhattan(v, u)
                    if d < mejor:
                        mejor = d
                        mejor_nodo = u

        visitados.append(mejor_nodo)
        total += mejor

    cache[grid] = total
    return total

def distancia_manhattan(pos1, pos2):
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def survivorHeuristic(state: Tuple[Tuple, Any], problem: MultiSurvivorProblem):
    """
    Your heuristic for the MultiSurvivorProblem.

    state: (position, survivors_grid)
    problem: MultiSurvivorProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider: distance to nearest survivor + MST of remaining survivors
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    pos_agente, grid = state

    if grid.count(True) == 0:
        return 0

    sobrevivientes = grid.asList(True)

    # Distancia mínima del agente a un sobreviviente
    dist_min = float('inf')
    
    for sobreviviente in sobrevivientes:
        dist = distancia_manhattan(pos_agente, sobreviviente)
        if dist < dist_min:
            dist_min = dist

    # MST del subconjunto actual
    mst_cost = mst_desde_grid(grid, problem.heuristicInfo)

    return dist_min + mst_cost
    
    # distancia_a_superviviente = float('inf')
    # for i in range(survivors_grid.width):
    #     for j in range(survivors_grid.height):
    #         if survivors_grid[i][j]:  # Si hay un superviviente en esta posición
    #             distancia = abs(posicion[0] - i) + abs(posicion[1] - j)
    #             distancia_a_superviviente = min(distancia_a_superviviente, distancia)
                
    # if distancia_a_superviviente == float('inf'):
    #     return 0  # No hay supervivientes restantes
    # return distancia_a_superviviente
     
