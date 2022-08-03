from node import Node
from problem import Problem
import heapq
import matplotlib.pyplot as plt


def visualize_route_problem_solution(problem, goal_node, file_name):
    goal_path = get_path_states(goal_node)

    for state in problem.map_coords:  # return the coordinates of the path states
        x, y = problem.map_coords[state]
        # if it is equal to first state
        if state == goal_path[0]:
            plt.scatter(x, y, c='r', marker='s')
        elif state == goal_node.state:
            plt.scatter(x, y, c='g', marker='s')
        else:
            plt.scatter(x, y, c='b', marker='s')

    for state1, state2 in problem.map_graph:  # drawing all edges
        x1, y1 = problem.map_coords[state1]
        x2, y2 = problem.map_coords[state2]
        dx = x2 - x1
        dy = y2 - y1
        plt.arrow(x1, y1, dx, dy, color='k')
        if state1 in goal_path and state2 in goal_path:
            plt.arrow(x1, y1, dx, dy, color='magenta')

    # plt.grid()
    plt.savefig(file_name)
    plt.show()
    plt.close()


def visualize_grid_problem_solution(problem, goal_node, file_name):
    for x, y in problem.wall_coords:
        plt.scatter(x, y, c='k', marker="s")
    for x, y in problem.food_coords:
        plt.scatter(x, y, c='r', marker="o")

    goal_states = get_path_states(goal_node)
    for i in range(len(goal_states)):
        if i == 0:
            x, y = goal_states[i][0]
            plt.scatter(x, y, c='g', marker='s')  # printing the last state
        else:
            x1, y1 = goal_states[i-1][0]
            x2, y2 = goal_states[i][0]
            dx = x2 - x1
            dy = y2 - y1
            plt.arrow(x1, y1, dx, dy, color='magenta')

    # plt.grid()
    plt.savefig(file_name)
    plt.show()
    plt.close()


class PriorityQueue:
    def __init__(self, items=(), priority_function=(lambda x: x)):
        self.priority_function = priority_function
        self.pqueue = []
        # add the items to the PQ
        for item in items:
            self.add(item)

    """
    Add item to PQ with priority-value given by call to priority_function
    """

    def add(self, item):
        pair = (self.priority_function(item), item)
        heapq.heappush(self.pqueue, pair)

    """
    pop and return item from PQ with min priority-value
    """

    def pop(self):
        return heapq.heappop(self.pqueue)[1]

    """
    gets number of items in PQ
    """

    def __len__(self):
        return len(self.pqueue)


def expand(problem, node):
    children_nodes = []
    # returns the available actions in a certain state
    actions_available = problem.actions(node.state)
    if actions_available is not None:
        for action in actions_available:
            # get result state from the given action
            result_state = problem.result(node.state, action)
            action_cost = node.path_cost + \
                problem.action_cost(node.state, action,
                                    result_state)  # cost of a given action
            children_nodes.append(
                Node(result_state, node, action, action_cost))  # create Node object for the given info

    return children_nodes


def get_path_actions(node):  # the node list from the initial node to the current node
    path_actions = []
    if node is None or node.parent_node is None:
        return []
    else:
        while node.parent_node is not None:
            if node.action_from_parent is not None:
                path_actions.insert(0, node.action_from_parent)
            node = node.parent_node

        return path_actions


def get_path_states(node):
    path_states = []
    if node is None:
        return []
    else:
        while node is not None:
            path_states.insert(0, node.state)
            node = node.parent_node
        return path_states


def best_first_search(problem, f):
    node = Node(problem.initial_state)
    # stores the possible nodes to go in the next step
    frontier = PriorityQueue([node], priority_function=f)
    reached = {node.state: node}
    while len(frontier) != 0:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        else:
            for child in expand(problem, node):
                s = child.state
                if s not in reached or child.path_cost < reached[s].path_cost:
                    reached[s] = child
                    frontier.add(child)
    return None


def best_first_search_treelike(problem, f):
    node = Node(problem.initial_state)
    frontier = PriorityQueue([node], priority_function=f)
    # reached = {node.state: node}
    while len(frontier) != 0:
        node = frontier.pop()
        if problem.is_goal(node.state):
            return node
        else:
            for child in expand(problem, node):
                # s = child.state
                # if s not in reached or child.path_cost < reached[s].path_cost:
                # reached[s] = child
                frontier.add(child)
    return None


def breadth_first_search(problem, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, f=lambda node: node.depth)
    else:
        return best_first_search(problem, f=lambda node: node.depth)


def depth_first_search(problem, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, f=lambda node: -node.depth)
    else:
        return best_first_search(problem, f=lambda node: -node.depth)


def uniform_cost_search(problem, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, f=lambda node: node.path_cost)
    else:
        return best_first_search(problem, f=lambda node: node.path_cost)


def greedy_search(problem, h, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, f=lambda node: h(node))
    else:
        return best_first_search(problem, f=lambda node: h(node))


def astar_search(problem, h, treelike=False):
    if treelike:
        return best_first_search_treelike(problem, f=lambda node: h(node) + node.path_cost)
    else:
        return best_first_search(problem, f=lambda node: h(node) + node.path_cost)
