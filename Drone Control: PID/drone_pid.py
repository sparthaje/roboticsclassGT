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

def general_pid(target, state, tau_p, tau_d, tau_i, data: dict):
    # Initialize data:
    if len(data.keys()) == 0:
        data["sum_error"] = 0.0
        data["prev_error"] = None

    error = target - state 
    data["error"] = error
    data["sum_error"] += error

    output = tau_p * error + tau_i * data["sum_error"]

    data["d_error"] = None
    if data["prev_error"] is not None:
        output += tau_d * (error - data["prev_error"])
        data["d_error"] = error - data["prev_error"]

    data["prev_error"] = error 

    return output, data

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

def pid_thrust(target_elevation, drone_elevation, tau_p=0, tau_d=0, tau_i=0, data: dict() = {}):
    '''
    Student code for Thrust PID control. Drone's starting x, y position is (0, 0).

    Args:
    target_elevation: The target elevation that the drone has to achieve
    drone_elevation: The drone's elevation at the current time step
    tau_p: Proportional gain
    tau_i: Integral gain
    tau_d: Differential gain
    data: Dictionary that you can use to pass values across calls.
        Reserved keys:
            max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.

    Returns:
        Tuple of thrust, data
        thrust - The calculated change in thrust using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.
            Reserved keys:
                max_rpm_reached: (True|False) - Whether Drone has reached max RPM in both its rotors.
    '''

    return general_pid(target_elevation, drone_elevation, tau_p, tau_d, tau_i, data)


def pid_roll(target_x, drone_x, tau_p=0, tau_d=0, tau_i=0, data:dict() = {}):
    '''
    Student code for PD control for roll. Drone's starting x,y position is 0, 0.

    Args:
    target_x: The target horizontal displacement that the drone has to achieve
    drone_x: The drone's x position at this time step
    tau_p: Proportional gain, supplied by the test suite
    tau_i: Integral gain, supplied by the test suite
    tau_d: Differential gain, supplied by the test suite
    data: Dictionary that you can use to pass values across calls.

    Returns:
        Tuple of roll, data
        roll - The calculated change in roll using PID controller
        data - A dictionary containing any values you want to pass to the next
            iteration of this function call.

    '''

    return general_pid(target_x, drone_x, tau_p, tau_d, 0.0, data)  # NOTE: PD controller


def compute_error(hover_error, max_allowed_vel, drone_max_vel, max_allowed_oscil, total_oscillations, accept=0.04):
    if drone_max_vel <= 0:
        return 3.0

    hover_score = 1.0 if hover_error <= 0 else min(accept / hover_error, 1.0)

    vel_score = 1.0
    if max_allowed_vel and drone_max_vel > max_allowed_vel:
        vel_score = min(max_allowed_vel / drone_max_vel, 1.0)

    osc_score = 1.0
    if max_allowed_oscil != -1 and total_oscillations > max_allowed_oscil:
        osc_score = min(max_allowed_oscil / total_oscillations, 1.0)

    return 3.0 - (hover_score + vel_score + osc_score)

def twiddle(run, params, dp, threshold=1e-4, plateau_window=30, plateau_frac=0.01, accept_error=1e-3):
    _n = [0]
    _run = run
    def run(p):
        _n[0] += 1
        return _run(p)

    best_error = compute_error(*run(params))
    error_history = []

    while sum(dp) > threshold and best_error > accept_error and _n[0] < 5000:
        for idx in range(len(params)):
            params[idx] += dp[idx]
            error = compute_error(*run(params))

            # good signal
            if error < best_error:
                dp[idx] *= 1.05
                best_error = error

            else:
                # check other direction
                params[idx] -= 2 * dp[idx]
                error = compute_error(*run(params))

                if error < best_error:
                    dp[idx] *= 1.05
                    best_error = error
                else:
                    params[idx] += dp[idx]  # reset to origin
                    dp[idx] *= 0.95

        error_history.append(best_error)
        if len(error_history) >= plateau_window:
            recent = error_history[-plateau_window:]
            if max(recent) - min(recent) < max(plateau_frac * best_error, accept_error):
                break

    return params


def find_parameters_thrust(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test cases only.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''

    # Initialize a list to contain your gain values that you want to tune
    params = [1.0, 30.0, 0.0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = lambda params: {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    run = lambda p: run_callback(thrust_params(p), roll_params, VISUALIZE=VISUALIZE)

    params = twiddle(run, params, [1.0, 5.0, 0.0])

    # Return the dict of gain values that give the best error.
    return thrust_params(params), roll_params

def find_parameters_with_int(run_callback, tune='thrust', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you can focus on
    tuning gain values for Thrust test case with Integral error

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''
    # Initialize a list to contain your gain values that you want to tune
    params = [1.0, 30.0, 0.0]

    # Create dicts to pass the parameters to run_callback
    thrust_params = lambda params: {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    roll_params   = {'tau_p': 0, 'tau_d': 0, 'tau_i': 0}

    run = lambda p: run_callback(thrust_params(p), roll_params, VISUALIZE=VISUALIZE)

    params = twiddle(run, params, [0.2, 5.0, 1e-3])

    # Return the dict of gain values that give the best error.
    return thrust_params(params), roll_params


def find_parameters_with_roll(run_callback, tune='both', DEBUG=False, VISUALIZE=False):
    '''
    Student implementation of twiddle algorithm will go here. Here you will
    find gain values for Thrust as well as Roll PID controllers.

    Args:
    run_callback: A handle to DroneSimulator.run() method. You should call it with your
                PID gain values that you want to test with. It returns an error value that indicates
                how well your PID gain values followed the specified path.

    tune: This will be passed by the test harness.
            A value of 'thrust' means you only need to tune gain values for thrust.
            A value of 'both' means you need to tune gain values for both thrust and roll.

    DEBUG: Whether or not to output debugging statements during twiddle runs
    VISUALIZE: Whether or not to output visualizations during twiddle runs

    Returns:
        tuple of the thrust_params, roll_params:
            thrust_params: A dict of gain values for the thrust PID controller
              thrust_params = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

            roll_params: A dict of gain values for the roll PID controller
              roll_params   = {'tau_p': 0.0, 'tau_d': 0.0, 'tau_i': 0.0}

    '''
    # Initialize a list to contain your gain values that you want to tune
    params = [1.0, 30.0, 0.0, 0.16, 9.6]

    # Create dicts to pass the parameters to run_callback
    thrust_params = lambda params: {'tau_p': params[0], 'tau_d': params[1], 'tau_i': params[2]}

    roll_params   = lambda params: {'tau_p': params[3], 'tau_d': params[4],'tau_i': 0}

    run = lambda p: run_callback(thrust_params(p), roll_params(p), VISUALIZE=VISUALIZE)

    params = twiddle(run, params, [0.2, 5.0, 1e-3, 0.1, 2.0])

    # Return the dict of gain values that give the best error.
    return thrust_params(params), roll_params(params)

def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith226).
    whoami = 'sparthaje3'
    return whoami
