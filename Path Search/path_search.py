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

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        # The following is the hard coded solution to test case 1
        moves = ['move w',
                 'move nw',
                 'lift 1',
                 'move se',
                 'down e',
                 'move ne',
                 'lift 2',
                 'down s']

        if debug:
            for i in range(len(moves)):
                print(moves[i])

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

        # hard code solution for TC #1
        smooth_path = [[0, 0], [1, 0], [2, 1]]
        moves = ['move s', 'move se']


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
    whoami = ''
    return whoami


if __name__ == "__main__":
    """
    You may execute this file to develop and test the search algorithm prior to running
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python path_search.py
    """

    # Test code in here will NOT be called by the autograder
    # This section is just a provided as a convenience to help in your development/debugging process

    # Testing for Part A
    print('\n~~~ Testing for part A: ~~~\n')

    from testing_suite_partA import wrap_warehouse_object, Counter

    # test case data starts here
    # testcase 1
    warehouse = [
        '######',
        '#....#',
        '#.1#2#',
        '#..#.#',
        '#...@#',
        '######',
    ]
    todo = list('12')
    benchmark_cost = 23
    viewed_cell_count_threshold = 20
    dropzone = (4,4)
    box_locations = {
        '1': (2,2),
        '2': (2,4),
    }
    # test case data ends here

    viewed_cells = Counter()
    warehouse_access = wrap_warehouse_object(warehouse, viewed_cells)
    partA = DeliveryPlanner_PartA(warehouse_access, dropzone, todo, box_locations)
    partA.plan_delivery(debug=True)
    # Note that the viewed cells for the hard coded solution provided
    # in the initial template code will be 0 because no actual search
    # process took place that accessed the warehouse
    print('Viewed Cells:', len(viewed_cells))
    print('Viewed Cell Count Threshold:', viewed_cell_count_threshold)

    # Testing for Part B
    # testcase 1
    print('\nTesting for part B:')
    path = [[0, 0], [1, 0], [2, 0], [2, 1]]
    warehouse = ['....',
                 '....',
                 '.@..',
                 '...#']
    obs = [[3, 3]]
    robot_init = [[0, 0]],
    benchmark_path = 3

    partB = PathSmoothing_PartB(warehouse, obs, path, robot_init, benchmark_path)
    partB.smooth_path(debug=True)

