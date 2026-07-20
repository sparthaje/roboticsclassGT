######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################

# If you see different scores locally and on Gradescope, this may be because:
# - you forgot that the test cases for each are different (e.g., if your
#   solution is not robust enough, you may pass Test Case 4 locally but still
#   fail Test Case 4 on Gradescope, or vice-versa);
# - you are uploading a different file than the one you are executing locally
#   (i.e., if this local ID doesn't match the ID on Gradescope, this indicates
#   that you uploaded a different file), in which case you should use the
#   OUTPUT_UNIQUE_FILE_ID to determine if this is the case; and/or
# - you modified one of the other files in the project in a way that causes your
#   local results to differ (since those changes don't carry over to
#   Gradescope), in which case you should download a fresh copy of all the
#   project files.

import math
import heapq

OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


# for part A, i dont think a* is the right solution but it works (modelled off path search hw)
class DeliveryPlanner_PartA:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo):

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    DELTAS = [
        (-1, 0, "n", 2),
        (1, 0, "s", 2),
        (0, 1, "e", 2),
        (0, -1, "w", 2),
        (-1, 1, "ne", 3),
        (-1, -1, "nw", 3),
        (1, 1, "se", 3),
        (1, -1, "sw", 3),
    ]
    LIFT_COST = 4
    DOWN_COST = 2

    def adjacent(self, pos, target):
        di = abs(target[0] - pos[0])
        dj = abs(target[1] - pos[1])
        return max(di, dj) == 1

    def floor_cost(self, pos):
        return self.warehouse_cost[pos[0]][pos[1]]

    def neighbors(self, pos):
        result = []
        for di, dj, direction, cost in self.DELTAS:
            ni, nj = pos[0] + di, pos[1] + dj
            if ni < 0 or nj < 0:
                continue
            try:
                cell = self.warehouse_state[ni][nj]
            except IndexError:
                continue
            if cell == '.' or cell == '@':
                result.append(((ni, nj), direction, cost))
        return result

    def direction(self, frm, to):
        di = to[0] - frm[0]
        dj = to[1] - frm[1]
        for ddi, ddj, name, _ in self.DELTAS:
            if ddi == di and ddj == dj:
                return name
        return None

    def h(self, s, target):
        # from: https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
        dx = abs(target[0] - s[0])
        dy = abs(target[1] - s[1])
        to_center = 2 * (dx + dy) - min(dx, dy)
        return max(0, to_center - 3)
    
    # fake final position (indicates a lift/drop action in the graph)
    GOAL = (-1, -1)
    def astar(self, start, target, goal_cells) -> tuple:
        open_set = [(0, start)]
        parent_dict = dict()
        g = {start: 0.0}

        while open_set:
            _, pos = heapq.heappop(open_set)

            if pos == self.GOAL:
                # reconstruct the grid path back to the start
                lift_cell = parent_dict[self.GOAL]
                path = [lift_cell]
                while path[-1] != start:
                    path.append(parent_dict[path[-1]])
                path.reverse()
                return path, g[self.GOAL]

            if pos in goal_cells:
                g_goal = g[pos] + goal_cells[pos]
                if g_goal < g.get(self.GOAL, float('inf')):
                    parent_dict[self.GOAL] = pos
                    g[self.GOAL] = g_goal
                    heapq.heappush(open_set, (g_goal, self.GOAL))

            for n, _, cost in self.neighbors(pos):
                g_n = g[pos] + cost + self.floor_cost(n)
                if g_n < g.get(n, float('inf')):
                    parent_dict[n] = pos
                    g[n] = g_n
                    f = g_n + self.h(n, target)
                    heapq.heappush(open_set, (f, n))

        return None, float('inf')

    def build_policy(self, target, goal_cells, terminal_action, on_target):
        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])
        policy = [['-1' for _ in range(cols)] for _ in range(rows)]

        for i in range(rows):
            for j in range(cols):
                pos = (i, j)
                cell = self.warehouse_state[i][j]
                if cell == '#':
                    policy[i][j] = '-1'
                    continue
                if pos == target and on_target is not None:
                    policy[i][j] = on_target
                    continue
                path, cosst = self.astar(pos, target, goal_cells)
                if path is None:
                    policy[i][j] = '-1'
                elif len(path) < 2:
                    # optimal to finish (lift / down) right here
                    policy[i][j] = terminal_action(pos)
                else:
                    policy[i][j] = 'move ' + self.direction(path[0], path[1])
        return policy

    def deliver_cost_to_go(self, cell, goal_cells, target):
        # cost of the optimal delivery starting from `cell` (holding the box)
        _, cost = self.astar(cell, target, goal_cells)
        return cost

    def generate_policies(self, debug=False):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """
        box = self.todo[0]
        box_loc = self.boxes[box]
        zone_floor = self.floor_cost(self.dropzone)
        box_floor = self.floor_cost(box_loc)

        self.warehouse_state[box_loc[0]][box_loc[1]] = '.'
        deliver_goals = {}
        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])
        for i in range(rows):
            for j in range(cols):
                pos = (i, j)
                if self.warehouse_state[i][j] in ('.', '@') and pos != self.dropzone \
                        and self.adjacent(pos, self.dropzone):
                    deliver_goals[pos] = self.DOWN_COST + zone_floor

        deliver_policy = self.build_policy(
            target=self.dropzone,
            goal_cells=deliver_goals,
            terminal_action=lambda pos: 'down ' + self.direction(pos, self.dropzone),
            on_target=None,  # robot may not drop while sitting on the dropzone -> must move off
        )

        box_goals = {}
        for i in range(rows):
            for j in range(cols):
                pos = (i, j)
                if self.warehouse_state[i][j] in ('.', '@') and self.adjacent(pos, box_loc):
                    d = self.deliver_cost_to_go(pos, deliver_goals, self.dropzone)
                    if d != float('inf'):
                        box_goals[pos] = self.LIFT_COST + box_floor + d

        self.warehouse_state[box_loc[0]][box_loc[1]] = box
        to_box_policy = self.build_policy(
            target=box_loc,
            goal_cells=box_goals,
            terminal_action=lambda pos: 'lift ' + box,
            on_target='B',
        )

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(deliver_policy)):
                print(deliver_policy[i])

        return (to_box_policy, deliver_policy)


# using value iteration learend in cs7642
class DeliveryPlanner_PartB:
    """
    [Doc string same as Part A]
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo
        self.stochastic_probabilities = stochastic_probabilities

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    DELTAS = [
        (-1, 0, "n", 2),
        (1, 0, "s", 2),
        (0, 1, "e", 2),
        (0, -1, "w", 2),
        (-1, 1, "ne", 3),
        (-1, -1, "nw", 3),
        (1, 1, "se", 3),
        (1, -1, "sw", 3),
    ]
    LIFT_COST = 4
    DOWN_COST = 2

    def adjacent(self, pos, target):
        di = abs(target[0] - pos[0])
        dj = abs(target[1] - pos[1])
        return max(di, dj) == 1

    def floor_cost(self, pos):
        return self.warehouse_cost[pos[0]][pos[1]]

    def neighbors(self, pos):
        result = []
        for di, dj, direction, cost in self.DELTAS:
            ni, nj = pos[0] + di, pos[1] + dj
            if ni < 0 or nj < 0:
                continue
            try:
                cell = self.warehouse_state[ni][nj]
            except IndexError:
                continue
            if cell == '.' or cell == '@':
                result.append(((ni, nj), direction, cost))
        return result

    def direction(self, frm, to):
        di = to[0] - frm[0]
        dj = to[1] - frm[1]
        for ddi, ddj, name, _ in self.DELTAS:
            if ddi == di and ddj == dj:
                return name
        return None


    DIRECTIONS = ["n", "nw", "w", "sw", "s", "se", "e", "ne"]
    DIRECTION_DELTAS = {
        "n": (-1, 0), "nw": (-1, -1), "w": (0, -1), "sw": (1, -1),
        "s": (1, 0), "se": (1, 1), "e": (0, 1), "ne": (-1, 1),
    }

    def move_cost(self, direction):
        return self.DIAGONAL_MOVE_COST if len(direction) == 2 else self.ORTHOGONAL_MOVE_COST

    def is_free(self, i, j):
        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])
        if not (0 <= i < rows and 0 <= j < cols):
            return False
        return self.warehouse_state[i][j] in ('.', '@')

    # formualte this as an RL env (state, action to reward)
    def build_env(self, spot, action_cost):
        self.ORTHOGONAL_MOVE_COST = 2
        self.DIAGONAL_MOVE_COST = 3
        self.ILLEGAL_MOVE_PENALTY = 100

        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])

        self.all_states = [(i, j) for i in range(rows) for j in range(cols)
                           if self.is_free(i, j)]
        self.state_to_index = {s: idx for idx, s in enumerate(self.all_states)}

        p = self.stochastic_probabilities
        outcome_offsets = [
            (-2, p['sideways']),
            (-1, p['slanted']),
            (0, p['as_intended']),
            (1, p['slanted']),
            (2, p['sideways']),
        ]

        self.terminal = set()
        self.P = dict()
        for s in self.all_states:
            self.P[s] = [[] for _ in range(self.n_actions)]

            if self.adjacent(s, spot):
                self.terminal.add(s)
                for a in range(self.n_actions):
                    self.P[s][a].append((1.0, s, -action_cost, True))
                continue

            for a in range(self.n_actions):
                for offset, prob in outcome_offsets:
                    if prob == 0:
                        continue
                    actual = self.DIRECTIONS[(a + offset) % self.n_actions]
                    di, dj = self.DIRECTION_DELTAS[actual]
                    ni, nj = s[0] + di, s[1] + dj
                    if self.is_free(ni, nj):
                        nxt = (ni, nj)
                        cost = self.move_cost(actual) + self.floor_cost(nxt)
                    else:
                        nxt = s
                        cost = self.ILLEGAL_MOVE_PENALTY + self.move_cost(actual)
                    self.P[s][a].append((prob, nxt, -cost, False))

        self.value_function = [0 for _ in range(len(self.all_states))]
        self.policy = [0 for _ in range(len(self.all_states))]

    def value_iteration(self):
        """ adaptation of cs7642 RL class value iteration
        """
        while True:
            delta = 0
            Q = [[0 for _ in range(self.n_actions)] for _ in range(len(self.all_states))]
            # it[erate over all states
            for state_index, state_array in enumerate(self.all_states):
                state = tuple(state_array)
                # iterate over all actions
                for a in range(self.n_actions):
                    # calculate the expected value of taking action `a` in state `state`
                    for prob, next_state, reward, done in self.P[state][a]:
                        next_state_index = self.state_to_index[tuple(next_state)]
                        Q[state_index][a] += prob * (reward + self.gamma *
                                                     self.value_function[next_state_index] * (not done))
                # update the value function with the best action value
                best_action_value = max(Q[state_index])
                delta = max(delta, abs(self.value_function[state_index] - best_action_value))
                self.value_function[state_index] = best_action_value
            # check for convergence
            if delta < self.theta:
                break
        # extract the policy from the Q-table
        for state_index in range(len(self.all_states)):
            self.policy[state_index] = 0
            for j, q in enumerate(Q[state_index]):
                if q > Q[state_index][self.policy[state_index]]:
                    self.policy[state_index] = j

    def solve(self, spot, action_cost, terminal_action, on_target):
        # spot is the dropoff/ pikcup. action cost is the cost to pick up / drop off. 
        self.build_env(spot, action_cost)
        self.value_iteration()

        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])
        policy = [['-1' for _ in range(cols)] for _ in range(rows)]
        values = [[0 for _ in range(cols)] for _ in range(rows)]

        for idx, s in enumerate(self.all_states):
            i, j = s
            values[i][j] = int(round(self.value_function[idx]))
            if s == spot and on_target is not None:
                policy[i][j] = on_target
            elif s in self.terminal:
                policy[i][j] = terminal_action(s)
            else:
                policy[i][j] = 'move ' + self.DIRECTIONS[self.policy[idx]]
        return policy, values

    def generate_policies(self, debug=False):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """
        self.n_actions = len(self.DIRECTIONS)
        self.gamma = 1.0 # dont need discounts here
        self.theta = 1e-6

        box = self.todo[0]
        box_loc = self.boxes[box]
        box_floor = self.floor_cost(box_loc)
        zone_floor = self.floor_cost(self.dropzone)

        self.warehouse_state[box_loc[0]][box_loc[1]] = box
        to_box_policy, to_box_values = self.solve(
            spot=box_loc,
            action_cost=self.LIFT_COST + box_floor,
            terminal_action=lambda s: 'lift ' + box,
            on_target='B',
        )

        # mark box cell walkable
        self.warehouse_state[box_loc[0]][box_loc[1]] = '.'
        to_zone_policy, to_zone_values = self.solve(
            spot=self.dropzone,
            action_cost=self.DOWN_COST + zone_floor,
            terminal_action=lambda s: 'down ' + self.direction(s, self.dropzone),
            on_target=None, 
        )

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nTo Zone Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith226).
    whoami = 'sparthaje3'
    return whoami


if __name__ == "__main__":
    """
    You may execute this file to develop and test the search algorithm prior to running
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will NOT be called by the autograder
    # This section is just a provided as a convenience to help in your development/debugging process

    # Testing for Part A
    # testcase 1
    print('\n~~~ Testing for part A: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[3, 5, 2],
                      [10, math.inf, 2],
                      [2, 10, 2]]

    todo = ['1']

    partA = DeliveryPlanner_PartA(warehouse, warehouse_cost, todo)
    partA.generate_policies(debug=True)

    # Testing for Part B
    # testcase 1
    print('\n~~~ Testing for part B: ~~~')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[13, 5, 6],
                      [10, math.inf, 2],
                      [2, 11, 2]]

    todo = ['1']

    stochastic_probabilities = {
        'as_intended': .70,
        'slanted': .1,
        'sideways': .05,
    }

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo, stochastic_probabilities)
    partB.generate_policies(debug=True)
