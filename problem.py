class Problem:

    def __init__(self, initial_state, goal_state=None):
        self.initial_state = initial_state
        self.goal_state = goal_state

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def is_goal(self, state):
        return state == self.goal_state

    def action_cost(self, state1, action, state2):
        return 1

    def h(self, node):
        return 0


class RouteProblem(Problem):

    def __init__(self, initial_state, goal_state=None, map_graph=None, map_coords=None):
        self.initial_state = initial_state
        self.goal_state = goal_state
        self.map_graph = map_graph
        self.map_coords = map_coords

    def actions(self, state):  # returns the possible states that can be reached from the current state
        reachable = []
        for key in self.map_graph:
            if key[0] == state:
                reachable.append(key[1])
        return reachable

    def result(self, state, action):  # returns the result state when doing the action (if possible)
        for key in self.map_graph:
            if state == key[0] and action == key[1]:
                return action
        return state

    def action_cost(self, state1, action, state2):
        for key in self.map_graph:
            if state1 == key[0] and state2 == key[1]:
                return self.map_graph[key]
        return 0

    def h(self, node):
        point_goal = point_current = []
        for key in self.map_coords:
            if node.state == key:
                point_current = self.map_coords[key]
            if self.goal_state == key:
                point_goal = self.map_coords[key]
        euclidean_distance = ((point_goal[0] - point_current[0])
                              ** 2 + (point_goal[1] - point_current[1]) ** 2) ** (0.5)
        return euclidean_distance


############################ GRID PROBLEM ################################


class GridProblem(Problem):

    def __init__(self, initial_state, N, M, wall_coords, food_coords):
        self.initial_state = initial_state
        self.N = N
        self.M = M
        self.wall_coords = wall_coords
        self.food_coords = food_coords
        self.goal_state = None
        self.food_eaten = tuple([False] * len(food_coords))
        self.initial_state = (initial_state, self.food_eaten)

    def actions(self, state):
        possible_actions = []
        (x, y) = state[0]
        if y+1 <= self.N and (x, y + 1) not in self.wall_coords:
            possible_actions.append('up')
        if y-1 > 0 and (x, y - 1) not in self.wall_coords:
            possible_actions.append('down')
        if x+1 <= self.M and (x + 1, y) not in self.wall_coords:
            possible_actions.append('right')
        if x-1 > 0 and (x - 1, y) not in self.wall_coords:
            possible_actions.append('left')
        return possible_actions

    def result(self, state, action):
        result_state = []
        if action not in self.actions(state):
            return state
        else:
            (x, y) = state[0]
            if action == 'up':
                result_state = (x, y + 1)
            elif action == 'down':
                result_state = (x, y - 1)
            elif action == 'right':
                result_state = (x + 1, y)
            elif action == 'left':
                result_state = (x - 1, y)

            for i in range(len(self.food_coords)):
                if result_state == self.food_coords[i]:
                    boolean_tuple = list(state[1])
                    boolean_tuple[i] = True
                    return (result_state, tuple(boolean_tuple))

            return (result_state, state[1])

    def action_cost(self, state1, action, state2):
        return 1

    def is_goal(self, state):
        return False not in state[1]

    # def h(self, node):
    #     if(self.is_goal(node.state)):
    #         return 0
    #     x, y = self.food_coords[node.state[1].index(False)]
    #
    #     curent_x, curent_y = node.state[0]
    #     min = abs(curent_x - x)+abs(curent_y-y)
    #
    #     for food in enumerate(self.food_coords):
    #         x, y = food[1]
    #         save = abs(curent_x-x)+abs(curent_y-y)
    #         if(save < min and not node.state[1][food[0]]):
    #             min = save
    #     return min

    # def h(self, node):
    #     if self.is_goal(node.state):
    #         return 0
    #     min_distance = float('inf')
    #     for x, y in self.food_coords:
    #         x_diff = abs(x - y)
    #         y_diff = abs(x - y)
    #         manhattan = (x_diff + y_diff)
    #         if manhattan < min_distance or min_distance is None:
    #             min_distance = manhattan
    #     return min_distance
