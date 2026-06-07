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

import unittest
import multiprocessing as mproc
import traceback
import sys
import copy
import io
import math
import random
from state import State
from visualizer_partB import GUI_partB

try:
    from path_search import PathSmoothing_PartB, who_am_i
    studentExc = None
except Exception as e:
    studentExc = traceback.format_exc()
    
########################################################################
# For debugging this flag can be set to True to print state 
# which could result in a timeout
########################################################################
VERBOSE_FLAG = False

########################################################################
# For visualization this flag can be set to True to display a GUI
# which could result in a timeout, but useful for debugging
# Note that enabling this will also enable DEBUGGING_SINGLE_PROCESS
########################################################################
VISUALIZE_FLAG = False

########################################################################
# For debugging set the time limit to a big number (like 600 or more)
########################################################################
TIME_LIMIT = 5  # seconds

########################################################################
# If your debugger does not handle multiprocess debugging very easily
# then when debugging set the following flag true.
########################################################################
DEBUGGING_SINGLE_PROCESS = False

# Necessary for GUI visualization, don't modify these lines
# if VISUALIZE_FLAG:
#     from visualizer import GUI
DEBUGGING_SINGLE_PROCESS = True if VISUALIZE_FLAG else DEBUGGING_SINGLE_PROCESS


def truncate_output( s, max_len = 2000 ):
    if len(s) > max_len:
        return s[:max_len-70] + "\n***************** OUTPUT TRUNCATED DUE TO EXCESSIVE LENGTH!**************\n"
    else:
        return s


DIRECTIONS = 'n,nw,w,sw,s,se,e,ne'.split(',')
DIRECTION_INDICES = {direction: index for index, direction in enumerate(DIRECTIONS)}
DELTA_DIRECTIONS = [
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
]

MOVE_DIRECTIONS = {"n":(-1,0),"ne":(-1,1),"e":(0,1),"se":(1,1),
                    "s":(1,0),"sw":(1,-1),"w":(0,-1),"nw":(-1,-1)}

class Submission:
    """Student Submission.

    Attributes:
        submission_score(Queue): Student score of last executed plan.
        submission_error(Queue): Error messages generated during last executed plan.
    """
    def __init__(self, fout=None):

        if DEBUGGING_SINGLE_PROCESS:
            import queue
            self.submission_score = queue.Queue(1)
            self.submission_error = queue.Queue(1)
            self.logmsgs = queue.Queue(1)
        else:
            self.submission_score = mproc.Manager().Queue(1)
            self.submission_error = mproc.Manager().Queue(1)
            self.logmsgs = mproc.Manager().Queue(1)

        self.fout = io.StringIO()

    def log(self, s):
        self.fout.write(s + '\n')

    def _reset(self):
        """Reset submission results.
        """
        while not self.submission_score.empty():
            self.submission_score.get()

        while not self.submission_error.empty():
            self.submission_error.get()

        while not self.logmsgs.empty():
            self.logmsgs.get()

    def moves_2_path(self, moves):
        # Map move -> (dy, dx)
        directions = {
            "move n": (-1, 0),
            "move s": (1, 0),
            "move e": (0, 1),
            "move w": (0, -1),
            "move ne": (-1, 1),
            "move nw": (-1, -1),
            "move se": (1, 1),
            "move sw": (1, -1),
        }

        # Start from self.start_loc
        y, x = self.robot_init
        converted_path = [[y, x]]

        for move in moves:
            if move not in directions:
                raise ValueError(f"Unknown move: {move}")
            dy, dx = directions[move]
            y += dy
            x += dx
            converted_path.append([y, x])

        return converted_path

    #test_case, path, smooth_path, obs, benchmark_path, converted_path
    def test_student_smoothing(self, test_case, warehouse, obs, path, robot_init, benchmark_path):
        """Execute student plan and store results in submission.

        Args:
            warehouse(list(list)): the warehouse map to test against.
            warehouse_cost(list(list)): integer costs for each warehouse position
            robot_initial_position (i,j): initial position of robot 
            boxes_todo(list): the order of boxes to deliver.
        """
        self._reset()
        self.test_case = test_case
        self.warehouse = warehouse
        self.obs = obs
        self.path = path
        self.robot_init = robot_init
        self.benchmark_path = benchmark_path

        #state = State(warehouse, warehouse_cost, robot_initial_position)
        quit_signal = None
        try:
            #warehouse, obs, path, robot_init,benchmark_cost
            student_planner = PathSmoothing_PartB(copy.deepcopy(warehouse), copy.deepcopy(obs), copy.deepcopy(path),
                                                  copy.deepcopy(robot_init), copy.deepcopy(benchmark_path))
            smooth_path, moves = student_planner.smooth_path(debug=VERBOSE_FLAG)

            #check to make sure smooth path and moves match
            path_moves_match = False
            converted_path = self.moves_2_path(moves)
            if smooth_path == converted_path:
                path_moves_match = True

            #test_case, path, smooth_path, obs, benchmark_path, converted_path
            viz_debug = GUI_partB(test_case, path, converted_path, obs, benchmark_path, converted_path)

            if VERBOSE_FLAG:
                viz_debug.print_debug()

            if VISUALIZE_FLAG:
                viz_debug.draw_graph()

            score = viz_debug.grading()
            self.submission_score.put(score)

        except Exception as err:
            if VERBOSE_FLAG:
                # very detailed stack trace - clutters everything up
                self.submission_error.put(traceback.format_exc())
            else:
                # slightly less cluttered output but the stack trace is much less informative
                self.submission_error.put(err)
            self.submission_score.put(0)

        #self.logmsgs.put( truncate_output( self.fout.getvalue() ) )


class PartBTestCase(unittest.TestCase):
    """ Test Part B.
    """

    results = ['', 'PART B TEST CASE RESULTS']
    SCORE_TEMPLATE = "\n".join((
        "\n-----------",
        "Test Case {test_case}",
        "Output: {output}",
        "credit: {score:.2f}"
    ))
    FAIL_TEMPLATE = "\n".join((
        "\n-----------",
        "Test Case {test_case}",
        "Output: {output}",
        "Failed: {message}",
        "credit: 0"
    ))

    credit = []
    totalCredit = 0

    fout = None

    @classmethod
    def _log(cls, s):
        (cls.fout or sys.stdout).write( s + '\n')

    def setUp(self):
        """Initialize test setup.
        """
        if studentExc:
            self.credit.append( 0.0 )
            self.results.append( "exception on import: %s" % str(studentExc) )
            raise studentExc

        self.student_submission = Submission( fout = self.__class__.fout )

    def tearDown(self):
        self.__class__.totalCredit = sum(self.__class__.credit)

    @classmethod
    def tearDownClass(cls):
        """Save student results at conclusion of test.
        """
        # Prints results after all tests complete
        for line in cls.results:
            cls._log(line)
        cls._log("\n-----------")
        cls._log('\nTotal Credit: {:.2f}'.format(cls.totalCredit))


    def check_results(self, params):

        error_message = ''
        logmsg = ''

        if not self.student_submission.logmsgs.empty():
            logmsg = self.student_submission.logmsgs.get()

        if not self.student_submission.submission_score.empty():
            score = self.student_submission.submission_score.get()
        if score > 1:
            score = 1

        if not self.student_submission.submission_error.empty():
            error_message = self.student_submission.submission_error.get()
            self.results.append(self.FAIL_TEMPLATE.format(message=error_message, output = logmsg, **params))

        else:
            self.results.append(self.SCORE_TEMPLATE.format(score=score, output = logmsg, **params))

        self.credit.append(score)

        self._log('test case {} credit: {}'.format(params['test_case'], score))
        if error_message:
            self._log('{}'.format(error_message))

        self.assertFalse(error_message, error_message)

        # fail the test if score is less than 1
        self.assertGreaterEqual(score, 1.0)

    def run_with_params(self, params):
        """Run test case using desired parameters.

        Args:
            params(dict): a dictionary of test parameters.
        """

        if DEBUGGING_SINGLE_PROCESS:
            self.student_submission.test_student_smoothing(params['test_case'],
                                                           params['warehouse'],
                                                           params['obs'],
                                                           params['path'],
                                                           params['robot_init'],
                                                           params['benchmark_path'])
        else:
            #warehouse, obs, path, robot_init,benchmark_path
            test_process = mproc.Process(target=self.student_submission.test_student_smoothing,
                                         args=(params['test_case'],
                                               params['warehouse'],
                                               params['obs'],
                                               params['path'],
                                               params['robot_init'],
                                               params['benchmark_path']))

        if DEBUGGING_SINGLE_PROCESS :

            # Note: no TIMEOUT is checked in this case so that debugging isn't 
            # inadvertently stopped

            self.check_results( params )

        else:

            logmsg = ''

            try:
                test_process.start()
                test_process.join(TIME_LIMIT)
            except Exception as exp:
                error_message = exp

            if test_process.is_alive():
                test_process.terminate()
                error_message = ('Test aborted due to timeout. ' +
                                'Test was expected to finish in fewer than {} second(s).'.format(TIME_LIMIT))
                if not self.student_submission.logmsgs.empty():
                    logmsg = self.student_submission.logmsgs.get()
                self.results.append(self.FAIL_TEMPLATE.format(message=error_message, output = logmsg, **params))

            else:

                self.check_results( params )
   
    def test_case_01(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [2, 1]]
        params = {'test_case': 1,
                  'warehouse': ['....',
                                '....',
                                '.@..',
                                '...#'],
                  'obs': [[3, 3]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 3}

        self.run_with_params(params)

    # Notice that we have included several extra test cases below.
    # You can uncomment one or more of these for extra tests.

    def test_case_02(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [2, 1], [2, 2]]
        params = {'test_case': 2,
                  'warehouse': ['....',
                                '....',
                                '..@.',
                                '...#'], #flipped warehouse for inverted y-axis
                  'obs': [[3, 3]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 4}

        self.run_with_params(params)

    def test_case_03(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], [8, 0], [9, 0],
                     [9, 1], [9, 2], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7], [9, 8], [9, 9]]
        params = {'test_case': 3,
                  'warehouse': ['..........',
                                '..........',
                                '..........',
                                '.....#....',
                                '....#.....',
                                '...#......',
                                '...#......',
                                '..#.......',
                                '..........',
                                '.........@'], #flipped warehouse for inverted y-axis
                  'obs': [[3, 5], [4, 4], [5, 3], [6, 3], [7, 2]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 16}

        self.run_with_params(params)

    def test_case_04(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], [8, 0], [9, 0],
                     [9, 1], [9, 2], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7], [9, 8], [9, 9], [8, 9],
                     [7, 9], [6, 9], [5, 9], [4, 9], [3, 9], [2, 9], [1, 9], [0, 9]]
        params = {'test_case': 4,
                  'warehouse': ['....#....@',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '..........'],
                  'obs': [[0, 4], [1, 3], [2, 4], [3, 3], [4, 4], [5, 3], [6, 4], [7, 3], [8, 4]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 22}

        self.run_with_params(params)

    def test_case_05(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], [8, 0], [9, 0],
                     [9, 1], [9, 2], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7], [9, 8], [9, 9], [8, 9],
                     [7, 9], [6, 9], [5, 9], [4, 9], [3, 9], [2, 9], [1, 9], [0, 9], [0, 8], [0, 7],
                     [0, 6], [0, 5], [0, 4], [0, 3], [0, 2]]
        params = {'test_case': 5,
                  'warehouse': ['.#@.......',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '..........'], #flipped warehouse for inverted y-axis
                  'obs': [[0, 1], [1, 3], [2, 4], [3, 3], [4, 4], [5, 3], [6, 4], [7, 3], [8, 4]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 26}

        self.run_with_params(params)

    def test_case_06(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [4, 1], [4, 2], [4, 3], [5, 3], [6, 3],
                     [7, 3], [8, 3], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7], [9, 8], [9, 9]]
        params = {'test_case': 6,
                  'warehouse': ['.#........',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '..........',
                                '......#...',
                                '.#........',
                                '..#.......',
                                '..#......@'], #flipped warehouse for inverted y-axis
                  'obs': [[0, 1], [1, 3], [2, 4], [3, 3], [4, 4], [7, 1], [8, 2], [9, 2], [6, 6]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 13}

        self.run_with_params(params)

    def test_case_07(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [4, 1], [4, 2], [4, 3], [5, 3], [6, 3],
                     [7, 3], [8, 3], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7], [9, 8], [9, 9], [8, 9],
                     [7, 9], [6, 9], [5, 9], [4, 9], [3, 9], [2, 9], [1, 9], [0, 9]]
        params = {'test_case': 7,
                  'warehouse': ['.#.......@',
                                '...#......',
                                '....#.....',
                                '...#......',
                                '....#.....',
                                '..........',
                                '......#...',
                                '.#........',
                                '..#.......',
                                '..#.......'], #flipped warehouse for inverted y-axis
                  'obs': [[0, 1], [1, 3], [2, 4], [3, 3], [4, 4], [7, 1], [8, 2], [9, 2], [6, 6]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 19}

        self.run_with_params(params)

    def test_case_08(self):
        temp_path = [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [4, 1], [4, 2], [4, 3], [5, 3], [6, 3],
                     [7, 3], [8, 3], [9, 3], [9, 4], [9, 5], [9, 6], [9, 7], [9, 8], [9, 9], [8, 9],
                     [7, 9], [6, 9], [5, 9], [4, 9], [3, 9], [2, 9], [1, 9], [1, 8], [1, 7], [1, 6],
                     [0, 6]]
        params = {'test_case': 8,
                  'warehouse': ['.#....@...',
                                '...#......',
                                '....#..#..',
                                '...#...#..',
                                '....#.....',
                                '..........',
                                '......#...',
                                '.#........',
                                '..#.......',
                                '..#.......'], #flipped warehouse for inverted y-axis
                  'obs': [[0, 1], [1, 3], [2, 4], [3, 3], [4, 4], [7, 1], [8, 2], [9, 2], [6, 6], [2, 7], [3, 7]],
                  'path': temp_path,
                  'robot_init': [0, 0],
                  'benchmark_path': 19}

        self.run_with_params(params)

    def test_case_09(self):
        temp_path = [[1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0], [7, 0], [8, 0], [9, 0], [9, 1],
                     [9, 2], [9, 3], [8, 3], [7, 3], [6, 3], [5, 3], [4, 3], [3, 3], [2, 3], [1, 3],
                     [0, 3], [0, 4], [0, 5], [0, 6], [1, 6], [2, 6], [3, 6], [4, 6], [5, 6], [6, 6],
                     [7, 6], [8, 6], [9, 6], [9, 7], [9, 8], [9, 9]]
        params = {'test_case': 9,
                  'warehouse': ['##........',
                                '..........',
                                '#.........',
                                '#...#.....',
                                '.#...#.#..',
                                '..........',
                                '.......##.',
                                '.........#',
                                '..........',
                                '.........@'],  #flipped warehouse for inverted y-axis
                  'obs': [[0, 0], [0, 1], [2, 0], [3, 0], [4, 1], [3, 4], [4, 5], [4, 7], [6, 7], [6, 8], [7, 9]],
                  'path': temp_path,
                  'robot_init': [1, 0],
                  'benchmark_path': 24}

        self.run_with_params(params)

    def test_case_10(self):
        temp_path = [[3, 0], [4, 0], [5, 0], [6, 0], [7, 0], [8, 0], [9, 0], [9, 1], [9, 2], [9, 3],
                     [8, 3], [7, 3], [6, 3], [5, 3], [4, 3], [3, 3], [2, 3], [1, 3], [0, 3], [0, 4],
                     [0, 5], [0, 6], [1, 6], [2, 6], [3, 6], [4, 6], [5, 6], [6, 6], [7, 6], [8, 6],
                     [9, 6], [9, 7], [9, 8], [9, 9], [8, 9], [7, 9], [6, 9], [5, 9]]
        params = {'test_case': 10,
                  'warehouse': ['.#........',
                                '..........',
                                '.#........',
                                '.#..#.....',
                                '.#...#.#.#',
                                '......#..@',
                                '........#.',
                                '..........',
                                '..........',
                                '..........'],  #flipped warehouse for inverted y-axis
                  'obs': [[0, 1], [2, 1], [3, 1], [4, 1], [3, 4], [4, 5], [4, 7], [5, 6], [6, 8], [4, 9]],
                  'path': temp_path,
                  'robot_init': [3, 0],
                  'benchmark_path': 24}

        self.run_with_params(params)

# Only run all of the test automatically if this file was executed from the command line.
# Otherwise, let Nose/py.test do it's own thing with the test cases.
if __name__ == "__main__":
    if studentExc:
        print(studentExc)
        print('score: 0')
    else:
        student_id = who_am_i()
        if student_id:
            PartBTestCase.fout = sys.stdout
            unittest.main()
        else:
            print("Student ID not specified.  Please fill in 'whoami' variable.")
            print('score: 0')
