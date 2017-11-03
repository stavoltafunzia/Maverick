import sys, datetime, os
sys.path.append('/usr/local/maverick/python')
import maverick

#%% Data
mf = 4539
m0 = 12500

model_parameters = {
                        'g' : 9.81,
                        'c' : 1715,
                        'sigma' : 0.0665,
                        'h0' : 90000,

                        #mass
                        'mi' : m0,
                        'mmin' : 0.8*mf,
                        'mmax' : 1.2*m0,
                        'mguess' : mf,

                        # speed
                        'Vi' : 0,
                        'Vmin' : -1500,
                        'Vmax' : 1500,
                        'Vguess' : 300,

                        #height
                        'hi' : 0,
                        'hmin' : 0,
                        'hmax' : 200000,

                        #Control
                        'Fmin' : 0,
                        'Fmax' : 1.9*9.81*m0,

                        # time
                        'Tmin' : 0,
                        'Tmax' : 500,
                        'Tguess' : 100,

                        # target
                        'wl1' : 0,
                        'wl2' : 1e-16, # to suppress oscillations: does not affect results at least up to 1e-5
                        'wm' : 1,
                        'wlt' : 0,
};
#%%
Data = {
    'info_level' : 5,

    'Model' : {

        'Parameters' : model_parameters,

        'Scaling' : {
            'Phase0' : {
                'target' : 100,
                'states_and_controls' : "automatic-bounds", # V, h ,m,
                'parameters' : "automatic-bounds", # T
                'fo_equations' : [20, 500, 60 ], # automatic bounds is not allowed for equations
                'boundary_conditions' : [500, 5000, 7000],
            },
        },

    },

    'Mesh' : {
        'max_iterations' : 0,
        'tolerance' : 1e-8,
        'Phase0' : {
           'Segments' : [ {'length' : 1, 'num_points' : 1000} ],
        },
    },

    'Solver' : {
        'max_iterations' : 300,
        'nlp_solver' : "Ipopt",
        'IpoptOptions' : {
            'linear_solver' : "ma27",
            'ma57_automatic_scaling' : "yes",
            'mu_strategy' : "adaptive",
            # nlp_scaling_method' : "none",
            'check_derivatives_for_naninf' : "yes",
            # 'tol' : 1e-7,
            # 'hessian_approximation' : "limited-memory",
            # 'derivative_test' : "second-order",
            # 'derivative_test_perturbation' : 1e-8,
        },
    },
}

#%% Solver

solver = maverick.Solver('GoddardRocket','../sources/')
output = solver.solve(Data)
sol = output['solution']['Phase0']

#%%
import matplotlib.pyplot as plt # DO NOT IMPORT MATPLOTLIB OR NUMPY BEFORE SOLVING THE PROBEM OTHERWISE IPOPT MAY USE THE WRONG BLAS!!!!!
plt.close('all')
#%%
plt.figure(1); plt.clf();
a1, = plt.plot(sol['t'],sol['thrust'],label='thrust');
plt.xlabel('time [s]')
plt.ylabel('thrust [N]')
plt.legend(handles=[a1])
plt.show()

#%%
plt.figure(2); plt.clf();
a1, = plt.plot(sol['t'],sol['h'],label='height');
plt.xlabel('time [s]')
plt.ylabel('height [m]')
plt.legend(handles=[a1])
plt.show()

#%%
plt.figure(3); plt.clf();
a1, = plt.plot(sol['t'],sol['V'],label='speed');
plt.xlabel('time [s]')
plt.ylabel('height [m]')
plt.legend(handles=[a1])
plt.show()
