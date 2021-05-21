import sys
from collections import deque

#from utils import *


class Problem:


    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal

    def actions(self, state):
        #this is action method, it will get a state and it will return a list of all possible actions that can be executed
        if state[2] == 1: #the boat will be towards the right side
            if state[0] == 3: #if there are 3 missionaries on the right side
                if state[1] >= 2:
                    return ['MM', 'M', 'MC', 'C', 'CC'];
                elif state[1] == 1:
                    return ['MM', 'M', 'MC', 'C'];
                else:
                    return ['MM', 'M'];
            elif state[0] == 2: #if there is 1 missionary on the left side and 2 on the right side
                if (state[1] == 2):
                    return ['MM', 'M', 'MC', 'C', 'CC'];
                else:
                    return [];
            elif state[0] == 1: #if there are 2 missionaries on the left side and 1 missionary on the right side
                if state[1] == 1:
                    return ['M', 'MC', 'C'];
                else:
                    return [];
            else: #there are not any missionaries on the right side, but there are 3 on the left side
                if state[1] >= 2: #if there are either 2 or 3 cannibals on the right side, and there are 0 or 1 on the left
                    return ['C', 'CC'];
                else: #if there is only 1 cannibal on the right side and 2 on the left side
                    return ['C'];
        if state[2] == 0:
            #the following function is the definition of when the boat is on the left side
            if state[0] == 3:
                if state[1] == 2:
                    return ['C'];
                else:
                    return ['C', 'CC'];
            elif state[0] == 2:
                if state[1] == 2:
                    return ['M', 'MC', 'C'];
                else:  # dead-state
                    return [];
            elif state[0] == 1:
                if state[1] == 1:
                    return ['MM', 'M', 'MC', 'C', 'CC'];
                else:  # dead-state
                    return [];
            else:
                if (state[1] == 3):
                    return ['M', 'MM'];
                elif (state[1] == 2):
                    return ['M', 'MM', 'MC', 'C'];
                else:
                    return ['M', 'MM', 'MC', 'CC'];
        else:
            return None;

    def result(self, state, action):
        #the result method will get a state and an action and will return a new state result
        #of applying that action to the given state
        state = list(state)
        if (state[2] == 1):
            if action == 'M':
                state[0] = state[0] - 1;# move one ball other side and subract
                state[2] = 0;# other side
            elif action == 'MM':
                state[0] = state[0] - 2;
                state[2] = 0
            elif action == 'MC':
                state[0] = state[0] - 1;
                state[1] = state[1] - 1;
                state[2] = 0
            elif action == 'C':
                state[1] = state[1] - 1;
                state[2] = 0
            else:
                state[1] = state[1] - 2;
                state[2] = 0;
        else:
            if action == 'M':
                state[0] = state[0] + 1;
                state[2] = 1
            elif action == 'MM':
                state[0] = state[0] + 2;
                state[2] = 1
            elif action == 'MC':
                state[0] = state[0] + 1;
                state[1] = state[1] + 1;
                state[2] = 1
            elif action == 'C':
                state[1] = state[1] + 1;
                state[2] = 1
            else:
                state[1] = state[1] + 2;
                state[2] = 1
        state = tuple(state)
        self.state = state
        return self.state

    def goal_test(self, state):
        #goal_test will be checking if the current state is a goal or not
        if state == self.goal:
            return True;
        else:
            return False;



    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError


# ______________________________________________________________________________

class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state. Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node. Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        leftSideOfRiver="Left Side {" + 'ML, '*self.state[0]+ 'CL, '*self.state[1] + "Boat"*self.state[2]+"}"
        rightSideOfRiver="Right Side{"+ 'MR, '*(3-self.state[0])+  'CR '*(3-self.state[1])+"Boat"*(1-self.state[2])+ "}"


        return "State:" +leftSideOfRiver+rightSideOfRiver;

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)


# ______________________________________________________________________________
def breadth_first_tree_search(problem):
    """
    [Figure 3.7]
    Search the shallowest nodes in the search tree first.
    Search through the successors of a problem to find a goal.
    The argument frontier should be an empty queue.
    Repeats infinitely in case of loops.
    """
    node =Node(problem.initial)
    if problem.goal_test(node.state):
        return node

    frontier = deque([node])  # FIFO queue
    explored=set()


    while frontier:
        node = frontier.popleft()
        explored.add(node.state)
        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                if problem.goal_test(child.state):
                    return child
                frontier.append(child)
    return None





initial = (3, 3, 1)
goal = (0, 0, 0)
problem = Problem(initial, goal)

s=breadth_first_tree_search(problem);
for i, state in enumerate(s.path()):
    print("{}) {}".format(i+1,state))

print(s.path_cost)
print(s.action)
