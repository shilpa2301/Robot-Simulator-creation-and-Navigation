# An idealized robot.
#     A vacuum-like robot that, somehow, does not experience any sensing or
#     movement errors.  See normal.robot for explanations of these
#     parameters.

body:
    radius: 0.2
    height: 0.06
wheels:
    distance: 0.3
    error_variance_left: 0.0
    error_variance_right: 0.0
    error_update_rate: 60.0
laser:
    rate: 1
    count: 25
    angle_min: -1.8
    angle_max: +1.8
    range_min:  0.01
    range_max: 40.00
    error_variance: 0.0
    fail_probability: 0.0

