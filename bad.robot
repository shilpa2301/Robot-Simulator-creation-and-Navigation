# A bad robot.
#     This file describes a robot that experiences errors that are
#     relatively large.  See normal.robot for explanations of these
#     parameters.

body:
    radius: 0.2
    height: 0.06

wheels:
    distance: 0.3
    error_variance_left: 0.01
    error_variance_right: 0.01
    error_update_rate: 0.5

laser:
    rate: 1
    count: 25
    angle_min: -1.8
    angle_max: +1.8
    range_min:  0.01
    range_max: 40.00
    error_variance: 0.01
    fail_probability: 0.3


