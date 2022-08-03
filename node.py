class Node:

    def __init__(self, state, parent_node=None, action_from_parent=None, path_cost=0):
        self.state = state
        self.parent_node = parent_node
        self.path_cost = path_cost
        self.action_from_parent = action_from_parent

        if parent_node is not None:
            self.depth = parent_node.depth + 1
        else:
            self.depth = 0

    def __lt__(self, other):
        return self.state < other.state
