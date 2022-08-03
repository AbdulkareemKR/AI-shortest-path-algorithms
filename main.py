from problem import Problem, RouteProblem
import search_algorithms
import matplotlib.pyplot as plt


example_map_graph = {
    ('A', 'B'): 1,
    ('A', 'C'): 1,
    ('A', 'D'): 1,
    ('B', 'A'): 1,
    ('B', 'C'): 1,
    ('B', 'E'): 1,
    ('C', 'B'): 1
}
example_coords = {
    'A': (1, 2),
    'B': (0, 1),
    'C': (1, 1),
    'D': (2, 1),
    'E': (0, 0),
}


# example_route_problem = RouteProblem(initial_state='A', goal_state='E', map_graph=example_map_graph,
#                                      map_coords=example_coords)

# print(search_algorithms.breadth_first_search(example_route_problem).state)

# x = (1, 3, 6, 5, 8, 9)
# y = (2, 4, 6, 7, 3, 5)
# plt.scatter(x, y, s=500, marker='s')
# plt.scatter(x[0], y[0], s=500,  c='g', marker='s')
# plt.scatter(x[-1], y[-1], s=500, c='r', marker='s')
# # plt.pyplot.arrow(x, y, x[-1], y[-1])

# plt.show()


# def visualize_route_problem_solution(problem, goal_node, file_name):
#     # return the coordinates of the path states
#     for i, state in enumerate(search_algorithms.get_path_states(goal_node)):
#         x, y = example_map_graph[state]
#         if i == 0:
#             plt.scatter(x, y, r='r', marker='s')
#         else:
#             plt.scatter(x, y, r='b')

#     plt.scatter(x, y, r='g', marker='s')

#     for state1, state2 in problem.map_graph[0]:
#         x1, y1 = example_coords[state1]
#         x2, y2 = example_coords[state2]
#         dx = x1 - x2
#         dy = y1 - y2
#         plt.pyplot.arrow(x1, y1, dx, dy)

#     plt.show()
#     plt.close()
#     plt.savefig(file_name)


# visualize_route_problem_solution(Problem('A', 'E'), 'E', "aaa")
