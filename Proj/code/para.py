import numpy as np

# all the unit: m
# actuator limit
v_max = 0.3
a_max = 0.3

# initial condition
pa_0 = np.array([0.182959, 0, 0.386151])
# when the value on the conveyor is 50, the v_b is set below
v_b = np.array([0, 0.0483, 0])

# sample time
# freq is 20Hz
ts = 0.05
t_max = 20
freq = 1 / ts
