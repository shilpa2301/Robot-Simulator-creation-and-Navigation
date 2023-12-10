# A typical robot.
#     This file describes a robot that has a similar size to many household
#     vacuum cleaner robots.

body:
    # Radius of the robot's main body, in meters.
    radius: 0.2
    
    # Height of the robot's main body, in meters.
    height: 0.06

wheels:
    # Distance between the wheels, in meters.
    distance: 0.3

    # The velocity of each wheel experiences a random multiplicative error.
    # The errors should be randomly selected from a Gaussian distribution,
    # with mean 1 and the variance shown below.  When the robot moves, the
    # target velocity given by the user is MULTIPLIED by the error value to
    # get the actual simulated velocity.
    error_variance_left: 0.0001
    error_variance_right: 0.0001

    # How often are the randomly selected error values updated, in seconds?
    # That is, specific numbers for the left and right wheels should be
    # selected at random every <error_update_rate> seconds, and that same
    # error should apply to all of the robot's movements for that entire
    # period, even if the robot's position is re-calculated more than once
    # in that interaval.
    error_update_rate: 0.5


laser:
    # How often, in seconds, does the robot's lidar publish a new scan?
    rate: 1

    # How many measurements are in each scan?  That is, how many elements in
    # the scan.ranges array?
    count: 25
  
    # What range of angles, in radians, does the scan sweep through?
    angle_min: -1.8
    angle_max: +1.8

    # What are the minimun and maximum measurable ranges, in meters?
    range_min:  0.01
    range_max: 40.00

    # Each measurement will experience some error.  The errors should be
    # selected from a Gaussian distribution with mean 0 and the variance
    # shown below.  For each measurement (i.e. each element of the ranges
    # array in a laser scan), an error value should be selected from this
    # distribution and ADDED to the actual simulated distance to the
    # nearest obstacle.
    error_variance: 0.001

    # In addition to the additive error described above, each measurement
    # has some probability, given below, to simply fail completely.  These
    # measurements should show up as NaN in the published scans.
    fail_probability: 0.1


