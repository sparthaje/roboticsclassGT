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

OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


from rait import matrix

dt_s = 1.0

class KalmanFilter:
    """ [x, y, x_dot, y_dot, x_ddot, yddot] """
    def __init__(self, x: matrix):
        self.x = x

        self.P = matrix()
        self.P.zero(6, 6)
        for n in range(6):
            self.P.value[n][n] = 0.05

        # white noise model: https://nbviewer.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
        psd_factor = 1e-6
        q5 = psd_factor * dt_s**5 / 20
        q4 = psd_factor * dt_s**4 / 8
        q3 = psd_factor * dt_s**3 / 6
        q2 = psd_factor * dt_s**2 / 2
        q1 = psd_factor * dt_s
        self.Q = matrix([
            [q5, 0,  q4, 0,  q3, 0 ],
            [0,  q5, 0,  q4, 0,  q3],
            [q4, 0,  q3, 0,  q2, 0 ],
            [0,  q4, 0,  q3, 0,  q2],
            [q3, 0,  q2, 0,  q1, 0 ],
            [0,  q3, 0,  q2, 0,  q1],
        ])

        
        # CA motion modell

        self.F = matrix()
        self.F.zero(6, 6)
   
        # x and y rows

        self.F.value[0][0] = 1
        self.F.value[1][1] = 1
   
        self.F.value[0][2] = dt_s
        self.F.value[1][3] = dt_s

        self.F.value[0][4] = 0.5 * dt_s
        self.F.value[1][5] = 0.5 * dt_s

        # x_dot and y_dot rows

        self.F.value[2][2] = 1
        self.F.value[3][3] = 1

        self.F.value[2][4] = dt_s
        self.F.value[3][5] = dt_s

        # x_ddot and y_ddot row
        self.F.value[4][4] = 1
        self.F.value[5][5] = 1

        # measurement noise
        self.R = matrix()
        self.R.zero(2, 2)
        self.R.value[0][0] = 1.0
        self.R.value[1][1] = 1.0

    def predict(self):
        self.x = self.F * self.x
        self.P = self.F * self.P * self.F.transpose() + self.Q

    def update(self, measurement):
        # pulled the update equations from https://en.wikipedia.org/wiki/Kalman_filter
        H = matrix()
        H.zero(2, 6)
        H.value[0][0] = 1
        H.value[1][1] = 1
        innov = measurement - H * self.x
        S = H * self.P * H.transpose() + self.R
        K = self.P * H.transpose() * S.inverse()
        self.x = self.x + K * innov
        I = matrix()
        I.identity(6)
        self.P = (I - K * H) * self.P

class Spaceship():
    """A class representing the Environment within which the spaceship will run,
     and containing the methods that will act on the spaceship."""

    def __init__(self, bounds, xy_start):
        """Initialize the Spaceship."""
        self.x_bounds = bounds['x']
        self.y_bounds = bounds['y']
        self.agent_pos_start = xy_start

        self.asteroids = dict()

    def predict_from_observations(self, asteroid_observations):
        """Observe asteroid locations and predict their positions at time t+1.
        Parameters
        ----------
        self = a reference to the current object, the Spaceship
        asteroid_observations = A dictionary in which the keys represent asteroid IDs
        and the values are a tuple of noisy x-coordinate observations,
        and noisy y-coordinate observations taken at time t.
        asteroid_observations format:
        ```
        `{1: (x-measurement, y-measurement),
          2: (x-measurement, y-measurement)...
          100: (x-measurement, y-measurement),
          }`
        ```

        Returns
        -------
        The output of the `predict_from_observations` function should be a dictionary of tuples
        of estimated asteroid locations one timestep into the future
        (i.e. the inputs are for measurements taken at time t, and you return where the asteroids will be at time t+1).

        A dictionary of tuples containing i: (x, y), where i, x, and y are:
        i = the asteroid's ID
        x = the estimated x-coordinate of asteroid i's position for time t+1
        y = the estimated y-coordinate of asteroid i's position for time t+1
        Return format:
        `{1: (x-coordinate, y-coordinate),
          2: (x-coordinate, y-coordinate)...
          100: (x-coordinate, y-coordinate)
          }`
        """
        # To view the visualization with the default pdf output (incorrect) uncomment the line below
        # return asteroid_observations

        # FOR STUDENT TODO: Update the Spaceship's estimate of where the asteroids will be located in the next time step

        for key in self.asteroids:
            self.asteroids[key].predict()

        for key in asteroid_observations:
            mx, my = asteroid_observations[key]
            if key not in self.asteroids:
                init_state = matrix([[mx], [my], [0], [0], [0], [0]])
                self.asteroids[key] = KalmanFilter(init_state)
                continue
            self.asteroids[key].update(matrix([[mx], [my]]))

        # todo: i don't kill objects... should implement that

        return_dict = {}
        for key in asteroid_observations:
            f = self.asteroids[key]
            x_next = f.F * f.x
            return_dict[key] = (x_next.value[0][0], x_next.value[1][0])

        return return_dict


    def jump(self, asteroid_observations, agent_data):
        """ Return the id of the asteroid the spaceship should jump/hop onto in the next timestep
        ----------
        self = a reference to the current object, the Spaceship
        asteroid_observations: Same as predict_from_observations method
        agent_data: a dictionary containing agent related data:
        'jump_distance' - a float representing agent jumping distance,
        'ridden_asteroid' - an int representing the ID of the ridden asteroid if available, None otherwise.
        Note: 'agent_pos_start' - A tuple representing the (x, y) position of the agent at t=0 is available in the constructor.

        agent_data format:
        {'ridden_asteroid': None,
         'jump_distance': agent.jump_distance,
         }
        Returns
        -------
        You are to return two items.
        1: idx, this represents the ID of the asteroid on which to jump if a jump should be performed in the next timestep.
        Return None if you do not intend to jump on an asteroid in the next timestep
        2. Return the estimated positions of the asteroids (i.e. the output of 'predict_from_observations method)
        IFF you intend to have them plotted in the visualization. Otherwise return None
        -----
        an example return
        idx to hop onto in the next timestep: 3,
        estimated_results = {1: (x-coordinate, y-coordinate),
          2: (x-coordinate, y-coordinate)}

        return 3, estimated_return

        """
        asteroids_next_ts = self.predict_from_observations(asteroid_observations)
        agent_position = asteroids_next_ts[agent_data["ridden_asteroid"]] if agent_data["ridden_asteroid"] else self.agent_pos_start
        
        candidates = [(key, asteroids_next_ts[key][0], asteroids_next_ts[key][1]) for key in asteroids_next_ts]

        def consider(candidate):
            key, x, y = candidate
            distance = ((x - agent_position[0])**2 + (y - agent_position[1])**2)**0.5
            EPSILON = 0.1
            # too far
            if distance > agent_data["jump_distance"] - EPSILON:
                return False

            # don't pick asteroids that will take you out of bounds ever
            EPS = 0.4
            if abs(x - self.x_bounds[0]) < EPS and self.asteroids[key].x.value[2][0] < 0:
                return False

            if abs(x - self.x_bounds[1]) < EPS and self.asteroids[key].x.value[2][0] > 0:
                return False

            if abs(y - self.y_bounds[0]) < EPS and self.asteroids[key].x.value[3][0] < 0:
                return False

            return True
 
        # sort so the velocity most in the up direction is first. note this includes the current asteroid as a candidate, which is the same as not jumping. if current vel is good no reason to abandon
        candidates = [x for x in candidates if consider(x)]
        candidates.sort(key=lambda x: -(self.asteroids[x[0]].x.value[3][0] / ((self.asteroids[x[0]].x.value[2][0]**2 + self.asteroids[x[0]].x.value[3][0]**2)**0.5 + 1e-9)))

        # FOR STUDENT TODO: Update the idx of the asteroid on which to jump
        idx = candidates[0][0] if len(candidates) > 0 else None

        # is current asteroid going to win the game, then no reason to jump
        if agent_data["ridden_asteroid"] and self.asteroids[agent_data["ridden_asteroid"]].x.value[3][0] > 0 and abs(agent_position[1] - self.y_bounds[1]) < 0.2:
            idx = None  # stay on winner
        if idx == agent_data["ridden_asteroid"] and idx is not None:
            idx = None

        return idx, asteroids_next_ts

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith226).
    whoami = 'sparthaje3'
    return whoami
