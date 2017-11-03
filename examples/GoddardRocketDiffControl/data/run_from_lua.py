
import sys, datetime, os
sys.path.append('/usr/local/maverick/python')
import maverick

solver = maverick.Solver('GoddardRocket','../sources/')
output = solver.solve('data.lua')
sol = output['solution']['Phase0']

import matplotlib.pyplot as plt # DO NOT IMPORT MATPLOTLIB OR NUMPY BEFORE SOLVING THE PROBEM OTHERWISE IPOPT MAY USE THE WRONG BLAS!!!!!
a1, = plt.plot(sol['t'],sol['thrust'],label='thrust');
plt.xlabel('time [s]')
plt.ylabel('thrust [N]')
plt.legend(handles=[a1])

