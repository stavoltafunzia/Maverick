require "readtexttable"
ground_altitude_profile_x = readtexttable("./../data/ground_altitude_profile_x.txt")
ground_altitude_profile_y = readtexttable("./../data/ground_altitude_profile_y.txt")
ground_altitude_profile_z = readtexttable("./../data/ground_altitude_profile_z.txt")

altitude =      {-100, -20, -10, 100, 200, 400, 600, 800, 1e3, 1.2e3, 1.4e3}
thrust_factor = {1,    1,     1,   1,   1, 0.8, 0.7, 0.6, 0.5, 0.3,   0.2} --exaggerated

-- Define the problem boundary conditions
x_i = 0
y_i = 0
z_i = 0

x_f = 10e3
y_f = 10e3
z_f = 100

distance = math.sqrt( (x_i-x_f)^2 + (y_i-y_f)^2 + (z_i-z_f)^2 )

-- model data
m = 3
g = 9.81

ref_thrust = m*g

md = {
    -- quadcopter data
    g         = g,
    rho_aria  = 1.2,
    m         = m,
    l         = 0.25,
    Id        = 0.47e-1,
    Iz        = 0.21e-1,
    CxA       = 0.09*1.5,
    CyA       = 0.03*1.5,
    CzA       = 0.09*1.5,
    CrollA    = 0.1*1.5,
    CpitchA   = 0.1*1.5,
    CyawA     = 0.02*1.5,
    V0        = 1.0,
    thrust_0  = ref_thrust,

    --boundary conditions: start from initial position and zero speed, and go to final position and zero speed
    x_i       = x_i,
    y_i       = y_i,
    z_i       = z_i,
    v_x_i     = 0,
    v_y_i     = 0,
    v_z_i     = 0,
    mu_i      = 0,
    phi_i     = 0,
    psi_i     = 0,

    x_f       = x_f,
    y_f       = y_f,
    z_f       = z_f,
    v_x_f     = 0,
    v_y_f     = 0,
    v_z_f     = 0,
    mu_f      = 0,
    phi_f     = 0,
    psi_f     = 0,

    -- guess
    time_guess  = distance/10,
    thrust_guess = ref_thrust,

    --state variables, controls and optimisation parameter bounds
    x_min     =  math.min(-100, math.min(x_i, x_f)) -100,
    x_max     =  math.max(100, math.max(x_i, x_f))  +100,
    y_min     =  math.min(-100, math.min(y_i, y_f)) -100,
    y_max     =  math.max(100, math.max(y_i, y_f))  +100,
    z_min     =  math.min(0, math.min(z_i, z_f))    -100,
    z_max     =  math.max(1e3,math.max(z_i, z_f))   +100,

    psi_max   =  math.pi/3,
    mu_max    =  math.pi/3,
    phi_max   =  math.pi/3,
    psi_min   = -math.pi/3,
    mu_min    = -math.pi/3,
    phi_min   = -math.pi/3,
    thrust_max = ref_thrust*1.5,
    thrust_min = ref_thrust*0.0,

    phi_dot_max    =  math.pi/3,
    psi_dot_max    =  math.pi/3,
    mu_dot_max     =  math.pi/3,
    thrust_dot_max =  ref_thrust,
    phi_dot_min    = -math.pi/3,
    psi_dot_min    = -math.pi/3,
    mu_dot_min     = -math.pi/3,
    thrust_dot_min = -ref_thrust,
    psi_dot_rel_imp = 2e1,

    v_x_min   = -50,
    v_x_max   =  50,
    v_y_min   = -50,
    v_y_max   =  50,
    v_z_min   = -50,
    v_z_max   =  50,

    T_min      = distance/40,
    T_max      = distance/5,

    --Constraints
    -- max_roll_rate   =  0.5,
    -- max_pitch_rate  =  0.5,
    -- max_yaw_rate    =  0.5,

    -- target
    wlt = 1,
    wlu = 5e-1,  -- penalize quadcopter thrust and angle derivatives
    wmt = 0,
}

Data = {

    Run0 = {
        info_level = 5,

        Model = {

            Parameters = md,

            MappedObjects = {
                RegularizedAbsoluteValue = {
                    type = "1arg_reg_abs_val_sqrt",
                    epsilon = 1e-2,
                },
                ThrustFactor = {
                    type = "1arg_spline",
                    spline_type = "Quintic",
                    x = altitude,
                    y = thrust_factor,
                },
                MinimumHeight = {
                    type = "2arg_spline",
                    spline_type = "Biquintic",
                    x1 = ground_altitude_profile_x.x,
                    x2 = ground_altitude_profile_y.y,
                    y = ground_altitude_profile_z.z,
                },
            },

            Scaling = {
                Phase0 = {
                    target = time_guess,
                    states_and_controls = "automatic-bounds",
                    parameters = {10}, -- T
                    fo_equations = { 10, 10, 10, -- x, y, z,
                                     ref_thrust, ref_thrust, ref_thrust, -- v_x, v_y, v_z
                                     1, 1, 1, -- phi_dot, mu_dot, psi_dot
                                     ref_thrust -- thrust_dot
                                   },
                    boundary_conditions = { distance, distance, distance, -- initial position
                                            10, 10, 10, -- initial speed
                                            1, 1, 1,    -- initial orientation
                                            distance, distance, distance, -- final position
                                            10, 10, 10, -- final speed
                                            1, 1, 1     -- final orientation
                                           },
                     path_constraints = { 1000 }, -- minimum height
                },
            },
        },

        Mesh = {
            max_iterations = 5,
            tolerance = 1e-5,
            Phase0 = {
               Segments = { {length=1, num_points=200} },
            },
        },

        Solver = {
            max_iterations = 300,
            nlp_solver = "Ipopt",

            IpoptOptions = {
                linear_solver = "ma57",
                ma57_automatic_scaling = "yes",
                mu_strategy = "adaptive",
                -- nlp_scaling_method = "none",
                check_derivatives_for_naninf = "yes",
            },
        },

        write_solution = './../results/out.txt',
        -- write_mesh_history = './../results/mh_out.txt',
    },
}
