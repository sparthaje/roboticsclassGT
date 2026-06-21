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

# These import statements give you access to library functions which you may
# (or may not?) want to use.
import random
import time
from math import *
from body import *
from solar_system import *
from satellite import *


def estimate_next_pos(gravimeter_measurement, get_theoretical_gravitational_force_at_point, distance, steering, other=None):
    """
    Estimate the next (x,y) position of the satelite.
    This is the function you will have to write for part A.
    :param gravimeter_measurement: float
        A floating point number representing
        the measured magnitude of the gravitation pull of all the planets
        felt at the target satellite at that point in time.
    :param get_theoretical_gravitational_force_at_point: Func
        A function that takes in (x,y) and outputs a float representing the magnitude of the gravitation pull from
        of all the planets at that (x,y) location at that point in time.
    :param distance: float
        The target satellite's motion distance
    :param steering: float
        The target satellite's motion steering
    :param other: any
        This is initially None, but if you return an OTHER from
        this function call, it will be passed back to you the next time it is
        called, so that you can use it to keep track of important information
        over time. (We suggest you use a dictionary so that you can store as many
        different named values as you want.)
    :return:
        estimate: Tuple[float, float]. The (x,y) estimate of the target satellite at the next timestep
        other: any. Any additional information you'd like to pass between invocations of this function
        optional_points_to_plot: List[Tuple[float, float, float]].
            A list of tuples like (x,y,h) to plot for the visualization
    """
    # time.sleep(1)  # uncomment to pause for the specified seconds each timestep
    
    # initialize particle filter
    N = 20000
    def random_particle():
        x = random.random() * 8*AU - 4*AU
        y = random.random() * 8*AU - 4*AU
        h = atan2(y, x) + pi/2
        return (x, y, h)
 
    if other is None:
        other = [
            random_particle() for _ in range(N)
        ]

    def compute_weight(x, y):
        VAR = 1e-14
        expected_gravity_magnitude = get_theoretical_gravitational_force_at_point(x, y)
        w = (1 / (2 * pi * VAR) ** 0.5) * e ** (-((gravimeter_measurement - expected_gravity_magnitude)**2) / (2 * VAR))
        return w

    weights = [compute_weight(x[0], x[1]) for x in other]
    if sum(weights) > 0:
        other = random.choices(other, weights=weights, k=N)
    else:
        # if sum weights is zero all the particles are probably bad -> take another guess
        other = [random_particle() for _ in range(N)]

    for idx in range(len(other)):
        # only fuzz a few.
        if random.random() < 0.90:
            continue

        FUZZ_SIZE = 0.02 * AU
        FUZZ_HEADING = 0.01 * pi
        x, y, h = other[idx]
        x += random.random() * (FUZZ_SIZE) - (0.5 * FUZZ_SIZE)
        y += random.random() * FUZZ_SIZE - 0.5*FUZZ_SIZE
        h += random.random() * FUZZ_HEADING - 0.5 * FUZZ_HEADING
        h = atan2(sin(h), cos(h)) # realias
        other[idx] = (x, y, h)

    for idx in range(len(other)):
        x, y, heading = other[idx]

        # propagate forward with bicycle model
        beta = (distance / 10.2) * tan(steering)
        
        radius = distance / beta
        cx = x - sin(heading) * radius
        cy = y + cos(heading) * radius
        
        heading = (heading + beta)
        # alias heading
        heading = atan2(sin(heading), cos(heading))

        x = cx + sin(heading) * radius
        y = cy - cos(heading) * radius

        other[idx] = (x, y, heading)

    mean = (0.0, 0.0)
    for x, y, _ in other:
        mean = (mean[0] + x, mean[1] + y)
    
    mean = (mean[0] / len(other), mean[1] / len(other))

    return mean, other, other


def next_angle(solar_system, percent_illuminated_measurements, percent_illuminated_sense_func,
               distance, steering, other=None):
    """
    Gets the next angle at which to send out an sos message to the home planet,
    the last planet in the solar system.
    This is the function you will have to write for part B.
    :param solar_system: SolarSystem
        A model of the solar system containing the sun and planets as Bodys (contains positions, velocities, and masses)
        Planets are listed in order from closest to furthest from the sun
    :param percent_illuminated_measurements: List[float]
        A list of floating point number from 0 to 100 representing
        the measured percent illumination of each planet in order from closest to furthest to sun
        as seen by the target satellite.
    :param percent_illuminated_sense_func: Func
        A function that takes in (x,y) and outputs the list of percent illuminated measurements of each planet
        as would be seen by satellite at that (x,y) location.
    :param distance: float
        The target satellite's motion distance
    :param steering: float
        The target satellite's motion steering
    :param other: any
        This is initially None, but if you return an OTHER from
        this function call, it will be passed back to you the next time it is
        called, so that you can use it to keep track of important information
        over time. (We suggest you use a dictionary so that you can store as many
        different named values as you want.)
    :return:
        bearing: float. The absolute angle from the satellite to send an sos message between -pi and pi
        xy_estimate: Tuple[float, float]. The (x,y) estimate of the target satellite at the next timestep
        other: any. Any additional information you'd like to pass between invocations of this function
        optional_points_to_plot: List[Tuple[float, float, float]].
            A list of tuples like (x,y,h) to plot for the visualization
    """

    # At what angle to send an SOS message this timestep
    bearing = 0.0
    xy_estimate = (110172640485.32968, -66967324464.19617)

    # You may optionally also return a list of (x,y) or (x,y,h) points that
    # you would like the PLOT_PARTICLES=True visualizer to plot.
    optional_points_to_plot = [ (1*AU,1*AU), (2*AU,2*AU), (3*AU,3*AU) ]  # Sample plot points

    return bearing, xy_estimate, other, optional_points_to_plot


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith226).
    whoami = 'sparthaje3'
    return whoami
