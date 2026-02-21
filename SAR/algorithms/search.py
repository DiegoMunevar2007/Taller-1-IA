from algorithms.problems import SearchProblem
import algorithms.utils as utils
from world.game import Directions
from algorithms.heuristics import nullHeuristic
from algorithms.utils import Stack, Queue, PriorityQueue

def tinyHouseSearch(problem: SearchProblem):
    """
    Returns a sequence of moves that solves tinyHouse. For any other building, the
    sequence of moves will be incorrect, so only use this for tinyHouse.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    stack = Stack()
    stack.push((problem.getStartState(), []))
    visitados = []

    while not stack.isEmpty():
        state, actions = stack.pop()
        if problem.isGoalState(state):
            return actions
        if state not in visitados:
            visitados.append(state)
            for (successor, action, cost) in problem.getSuccessors(state):
                new_actions = actions + [action]
                stack.push((successor, new_actions))
    return []

def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    queue = Queue()
    start = problem.getStartState()
    queue.push((start, []))

    visitados = []

    while not queue.isEmpty():
        state, actions = queue.pop()

        if problem.isGoalState(state):
            return actions

        if state not in visitados:
            visitados.append(state)

            for successor, action, cost in problem.getSuccessors(state):
                nuevas_acciones = actions + [action]
                queue.push((successor, nuevas_acciones))
    return []

def uniformCostSearch(problem: SearchProblem):
    """
    Search the node of least total cost first.
    """

    pq = PriorityQueue()
    pq.push((problem.getStartState(), 0, []), 0) # Estado inicial tiene costo 0
    visitados = []

    while not pq.isEmpty():
        state, cost, actions = pq.pop()
        if problem.isGoalState(state):
            return actions
        if state not in visitados:
            visitados.append(state)
            for successor, action, stepCost in problem.getSuccessors(state):
                new_cost = cost + stepCost
                new_actions = actions + [action]
                pq.push((successor, new_cost, new_actions), new_cost)
    return []

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """

    pq = PriorityQueue()
    pq.push((problem.getStartState(), 0, []), 0) # Estado inicial tiene costo 0
    visitados = []
    
    while not pq.isEmpty():
        estado, costo, acciones = pq.pop()
        if problem.isGoalState(estado):
            return acciones
        if estado not in visitados:
            visitados.append(estado)
            for sucesor, accion, costo_paso in problem.getSuccessors(estado):
                heurstic_cost = heuristic(sucesor, problem)
                new_cost = costo + costo_paso
                new_actions = acciones + [accion]
                pq.push((sucesor, new_cost, new_actions), new_cost + heurstic_cost)

# Abbreviations (you can use them for the -f option in main.py)
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
