import sys, datetime, os
sys.path.append('/usr/local/maverick/python')
import maverick
#%%
road_data = maverick.readTextTable('Adria2D.txt')

#%%
mp = {
    'g' : 9.81,
    'm' : 1200,
    'DX' : 1,
    'DY' : 1.3,
    'Klambda' : 10,
    'Fx_max' : 4,
    'Omega_max' : 1,
    'beta' : 0,
    'road_width' : 5,
    'w_reg' : 00.0,
    'w_diff_reg' : 0.6,

    'V0' : 7,
    'Vf' : 7,
    'Vmax' : 60,
    'Pmax' : 400*1000,
    'Pmin' : 400*1000*10,
    'Emin' : 10 * 1000000,
    'Emax' : 5 * 1000000,
    'max_power' : 1000,
    'min_power' : 0,
    'v_guess' : 20,
}

road_length=road_data['zeta'][-1]

mapped_objects = {
    'RoadCurvature' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['kappa'],
    },
    'RoadX' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['x'],
    },
    'RoadY' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['y'],
    },
    'RoadXR' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['xR'],
    },
    'RoadYR' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['yR'],
    },
    'RoadXL' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['xL'],
    },
    'RoadYL' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['yL'],
    },
    'RoadHeading' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['psi'],
    },
    'RoadRightWidth' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['rightWidth'],
    },
    'RoadLeftWidth' : {
        'type' : '1arg_spline',
        'spline_type' : 'Akima',
        'x' : road_data['zeta'],
        'y' : road_data['leftWidth'],
    },
    'RegularizedPositive' : {
        'type' : '1arg_reg_pos_sqrt',
        'epsilon' : 0.01,
    },
}

Data = {

    'Run0' : {
        'info_level' : 4,

        'Model' : {

            'Parameters' : mp,

            'MappedObjects' : mapped_objects,

            'Scaling' : {
                'Phase0' : {
                    'target' : 100,
                    'states_and_controls' : 'automatic-bounds', # n, alpha, V, lambda, ux, uy
                    'parameters' : 'automatic-bounds',
                    'fo_equations' : [1, 1, mp['Fx_max'] * mp['g'], 1 ],
                    'path_constraints' : 'automatic-bounds',
    			    'integral_constraints' : 'automatic-bounds',
                    'boundary_conditions' : [1, 1, mp['V0'], 1, mp['V0']],
                },
            },
        },

        'Mesh' : {
            'Phase0' : {
               'Segments' : [ {'length' : road_length, 'num_points' : 300} ],
            },
        },

        'Solver' : {
            # max_iterations' : 2,
            'nlp_solver' : 'Ipopt',
            # continuation_mode' : 'force_warm_start_with_multipliers',
            'IpoptOptions' : {
                'linear_solver' : 'ma57',
                'ma57_automatic_scaling' : 'yes',
                'mu_strategy' : 'adaptive',
                # nlp_scaling_method' : 'none',
                'check_derivatives_for_naninf' : 'yes',
                'tol' : 1e-7,
                # hessian_approximation' : 'limited-memory',
                # derivative_test' : 'second-order',
                # derivative_test_perturbation' : 1e-8,
            },
        },

    },
}

Data2 = {
    'Run0' : {
        # do not specify info level: will be unchanged
        # Model' : mp,

        'MappedObjects' : mapped_objects,

        # change the mesh: increase num points
        'Mesh' : {
            'Phase0' : {
               'Segments' : [ {'length' : road_length, 'num_points' : 3000} ],
            },
        },

        # solver: set warm start
        # NOTE: we do not need to re-define everything. Previous settings are not lost
        'Solver' : {
            'max_iterations' : 200,
            #'start_mode' : 'warm_start_with_multipliers',
            'start_mode' : 'warm_start',
            #'start_mode' : 'cold_start',
        },
    },
}
#%% Solver
solver = maverick.Solver('SingleMassPointE','../sources/')
output = solver.solve(Data, False)
#output = solver.solve('../data/advanced_example.lua', True)
#output = solver.solve('../data/auto_mesh.lua', False)
output0 = output.copy()
sol = output['solution']['Phase0']
sol0 = sol.copy();
x0 = sol0['zeta']

#%%
output = solver.solve(Data2, False)
output1 = output.copy()
sol = output['solution']['Phase0']
sol1 = sol.copy();
x1 = sol1['zeta']

#%% PLOTS
import matplotlib.pyplot as plt # DO NOT IMPORT MATPLOTLIB OR NUMPY BEFORE SOLVING THE PROBEM OTHERWISE IPOPT MAY USE THE WRONG BLAS!!!!!

#plt.close('all')

#%%
plt.figure(1); plt.clf();
a1, = plt.plot(x0,sol0['states_controls'][2],label='speed');
plt.legend(handles=[a1])
plt.show()

#%%
plt.figure(2); plt.clf();
for y in sol0['lambda_equations']:
    plt.plot(x0,y);

for y in sol1['lambda_equations']:
    plt.plot(x1,y,'+');

plt.show()

#%%
plt.figure(3); plt.clf();
for y in sol0['lambda_']:
    plt.plot(x0,y);

for y in sol1['lambda_']:
    plt.plot(x1,y,'+');

plt.show()

#%%
plt.figure(4); plt.clf();
for y in sol0['lambda_states_controls_upper']:
    plt.plot(x0,y);

for y in sol1['lambda_states_controls_upper']:
    plt.plot(x1,y,'+');

plt.show()

#%%
plt.figure(5); plt.clf();
for y in sol0['lambda_states_controls_lower']:
    plt.plot(x0,y);

for y in sol1['lambda_states_controls_lower']:
    plt.plot(x1,y,'+');

plt.show()

#%%
plt.figure(6); plt.clf();

plt.plot(sol0['lambda_boundary_conditions']);
plt.plot(sol1['lambda_boundary_conditions'],'+');


plt.show()

#%%
print(sol0['lambda_integral_constraints'])
print(sol1['lambda_integral_constraints'])
print(sol0['lambda_parameters_upper'])
print(sol1['lambda_parameters_upper'])
print(sol0['lambda_parameters_lower'])
print(sol1['lambda_parameters_lower'])
