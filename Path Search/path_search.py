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


class DeliveryPlanner_PartA:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

      plan_delivery(self, debug = False):
       Stubbed out below.  You may not change the method signature
        as it will be called directly by the autograder but you
        may modify the internals as needed.

      __init__:
        Required to initialize the class.  Signature can NOT be changed.
        Basic template starter code is provided.  You may choose to
        use this starter code or modify and replace it based on
        your own solution.
    """

    def __init__(self, warehouse_viewer, dropzone_location, todo, box_locations):

        self.warehouse_viewer = warehouse_viewer
        self.dropzone_location = dropzone_location
        self.todo = todo
        self.box_locations = box_locations

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

    def h(self, s, box):
        # from: https://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
        goal = self.box_locations[box] if box is not None else self.dropzone_location
        dx = abs(goal[0] - s[0])
        dy = abs(goal[1] - s[1])
        to_center = 2 * (dx + dy) - min(dx, dy)
        return max(0, to_center - 3)

    def adjacent_to_box(self, pos, box):
        target = self.box_locations[box] if box is not None else self.dropzone_location
        di = abs(target[0] - pos[0])
        dj = abs(target[1] - pos[1])
        return max(di, dj) == 1

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

    def neighbors(self, pos):
        result = []
        for di, dj, direction, cost in self.DELTAS:
            ni, nj = pos[0] + di, pos[1] + dj
            if ni < 0 or nj < 0:
                continue
            try:
                cell = self.warehouse_viewer[ni][nj]
            except IndexError:
                continue
            if cell == '.' or cell == '@':
                result.append(((ni, nj), direction, cost))
        return result

    def astar(self, start, box) -> list:
        open_set = [(0, start)]
        parent_dict = dict()
        g = {start: 0.0}

        while open_set:
            _, pos = heapq.heappop(open_set)

            # goal
            if self.adjacent_to_box(pos, box):
                path = [pos]
                while pos in parent_dict: 
                    pos = parent_dict[pos]
                    path.append(pos)
                return path[::-1]

            for n, _, cost in self.neighbors(pos):
                g_n = g[pos] + cost
                if g_n < g.get(n, float('inf')):
                    parent_dict[n] = pos 
                    g[n] = g_n 
                    f = g_n + self.h(n, box)
                    heapq.heappush(open_set, (f, n))



    def direction(self, frm, to):
        di = to[0] - frm[0]
        dj = to[1] - frm[1]
        for ddi, ddj, name, _ in self.DELTAS:
            if ddi == di and ddj == dj:
                return name
        return None

    # if len path s zero this crashes but that means solution is wrong so is chill
    def path_to_moves(self, path):
        moves = []
        for i in range(len(path) - 1):
            moves.append("move " + self.direction(path[i], path[i + 1]))
        return moves

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """
        moves = []
        pos = self.dropzone_location

        # subroutine solution where a* is only from drop off to pick up and vice versa 
        for box in self.todo:
            # lift box
            path = self.astar(pos, box)
            moves += self.path_to_moves(path)
            pos = path[-1]
            moves.append("lift " + box)

            # clear the picked box as wackalbe 
            box_i, box_j = self.box_locations[box]
            self.warehouse_viewer[box_i][box_j] = '.'

            # drop off box
            path = self.astar(pos, None)
            moves += self.path_to_moves(path)
            pos = path[-1]
            moves.append("down " + self.direction(pos, self.dropzone_location))

        if debug:
            for m in moves:
                print(m)

        return moves


class PathSmoothing_PartB:
    """
    Note: All printouts must be conditioned on the debug parameter.

    Required methods in this class are:

        smooth_path(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder, but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that intializes class variables based on the definitions in
        testing_suite_partB.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, obs, path, robot_init,benchmark_path):

        self.path = path
        self.start_loc = robot_init
        self.goal_loc = self.path[-1]
        self.obs = obs
        self.warehouse_object = warehouse
        self.benchmark_path = benchmark_path

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        self.obstacles = set(tuple(o) for o in obs)
        self.rows = len(warehouse)
        self.cols = len(warehouse[0])

        self.path_cumlen = [0.0]
        for i in range(1, len(self.path)):
            self.path_cumlen.append(self.path_cumlen[-1] + math.dist(self.path[i - 1], self.path[i]))

        # You may use these symbols indicating direction for visual debugging
        # ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # or you may choose to use arrows instead
        # ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

    def _set_initial_state_from(self, warehouse):
        """Set initial state. Optional Function

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

    DELTAS = {
        "n": (-1, 0),
        "ne": (-1, 1),
        "e": (0, 1),
        "se": (1, 1),
        "s": (1, 0),
        "sw": (1, -1),
        "w": (0, -1),
        "nw": (-1, -1),
    }
    HEADINGS = ["n", "ne", "e", "se", "s", "sw", "w", "nw"]

    def allowed_turns(self, heading):
        if heading is None:
            return list(self.HEADINGS)
        idx = self.HEADINGS.index(heading)
        return [self.HEADINGS[(idx + k) % 8] for k in (-1, 0, 1)]

    def original_at_arclen(self, s):
        cum = self.path_cumlen
        if s <= 0:
            return self.path[0]
        if s >= cum[-1]:
            return self.path[-1]
        for i in range(1, len(cum)):
            if cum[i] >= s:
                t = (s - cum[i - 1]) / (cum[i] - cum[i - 1])
                y = self.path[i - 1][0] + t * (self.path[i][0] - self.path[i - 1][0])
                x = self.path[i - 1][1] + t * (self.path[i][1] - self.path[i - 1][1])
                return (y, x)
        return self.path[-1]

    def dist_to_original_path(self, pos, s):
        return math.dist(pos, self.original_at_arclen(s))

    def neighbors(self, pos, heading, depth):
        result = []
        for d in self.allowed_turns(heading):
            di, dj = self.DELTAS[d]
            ni, nj = pos[0] + di, pos[1] + dj
            # stay on the map
            if ni < 0 or nj < 0 or ni >= self.rows or nj >= self.cols:
                continue
            if self.warehouse_object[ni][nj] == '#':
                continue
            if (ni, nj) in self.obstacles:
                continue
            step_len = math.sqrt(2) if (di != 0 and dj != 0) else 1
            if self.dist_to_original_path((ni, nj), depth + step_len) > 2.8:
                continue
            result.append(((ni, nj), d, step_len))
        return result

    def direction(self, frm, to):
        di = to[0] - frm[0]
        dj = to[1] - frm[1]
        for name, (ddi, ddj) in self.DELTAS.items():
            if ddi == di and ddj == dj:
                return name
        return None

    def path_to_moves(self, path):
        moves = []
        for i in range(len(path) - 1):
            moves.append("move " + self.direction(path[i], path[i + 1]))
        return moves

    def bfs(self, start, goal):
        to_visit = [(start, None)]
        depth = [0.0]  # traveled arc length so far
        parent = dict()
        visited = {(start, None)}

        while to_visit:
            pos, heading = to_visit.pop(0)
            d = depth.pop(0)

            if pos == goal:
                path = [pos]
                state = (pos, heading)
                while state in parent:
                    state = parent[state]
                    path.append(state[0])
                return path[::-1]

            for npos, ndir, step_len in self.neighbors(pos, heading, d):
                state = (npos, ndir)
                if state not in visited:
                    visited.add(state)
                    parent[state] = (pos, heading)
                    to_visit.append(state)
                    depth.append(d + step_len)

        return None

    def smooth_path(self, debug=False):
        """
         You may use the starter code provided above in any way you choose,
         but please condition any printouts on the debug flag.

         You can add any helper functions above, but smooth_path will be the function
         called by the autograder directly, so do not modify its inputs or returns.

         Returns:
         smooth_path should be a list of integer points
         moves should be a list of strings representing the moves your robot will make to follow the smooth path
        """
        # extract the start and goal states (grader passes [y, x] lists)
        start = tuple(self.start_loc)
        goal = tuple(self.goal_loc)

        path = self.bfs(start, goal)

        smooth_path = [list(p) for p in path]
        moves = self.path_to_moves(path)

        if debug:
            print("Original Path:")
            print(self.path)
            print("Smooth Path:")
            print(smooth_path)
            if len(smooth_path) > self.benchmark_path:
                print("Not enough smoothing, your path is too long.")
                print("Benmark: " + str(self.benchmark_path) + "Your Path: " + str(len(smooth_path)))
            print("Moves:")
            print(moves)

        # please make sure you remove any duplicate points in your smooth_path and your moves match your path.
        return smooth_path, moves


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith226).
    whoami = 'sparthaje3'
    return whoami


