if __name__ == "__main__":
    from problem import *
    from search_algorithms import *
    from node import Node

    # route problem example
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

    example_route_problem = RouteProblem(initial_state='A', goal_state='E',
                                         map_graph=example_map_graph,
                                         map_coords=example_coords)

    goal_node = breadth_first_search(example_route_problem)
    visualize_route_problem_solution(
        example_route_problem, goal_node, './route.png')
    plt.close()

    # grid problem example
    example_walls = [(4, 3), (5, 1), (5, 2)]

    example_food = [(3, 1), (2, 3), (4, 5)]

    example_grid_problem = GridProblem(initial_state=(7, 4),
                                       N=5, M=7,
                                       wall_coords=example_walls,
                                       food_coords=example_food)

    goal_node = breadth_first_search(example_grid_problem)
    visualize_grid_problem_solution(
        example_grid_problem, goal_node, './grid.png')
    plt.close()
