mf = 4539
m0 = 12500

mp = {
    g = 9.81,
    c = 1715,
    sigma = 0.0665,
    h0 = 90000,

    -- mass
    mi = m0,
    mmin = 0.8*mf,
    mmax = 1.2*m0,
    mguess = mf,

    -- speed
    Vi = 0,
    Vmin = -1500,
    Vmax = 1500,
    Vguess = 300,

    --height
    hi = 0,
    hmin = 0,
    hmax = 200000,

    --Control
    Fmin = 0,
    Fmax = 1.9*9.81*m0,

    -- time
    Tmin = 0,
    Tmax = 500,
    Tguess = 100,

    -- target
    wl1 = 0,
    wm  = 1,
    wlt = 0,

}

Data = {

    Run0 = {
        info_level = 5,

        Model = {

            Parameters = mp,

            Scaling = {
                Phase0 = {
                    target = 100,
                    states_and_controls = "automatic-bounds", -- V, h ,m,
                    algebraic_states_and_controls = "automatic-bounds", -- F
                    parameters = "automatic-bounds", -- T
                    fo_equations = {20, 500, 60 }, -- automatic bounds is not allowed for equations
                    boundary_conditions = {500, 5000, 7000},
                },
            },

        },

        Mesh = {
            max_iterations = 20,
            tolerance = 5e-9,
            Phase0 = {
               Segments = { {length=1, num_points=100} },
            },
        },

        Solver = {
            max_iterations = 300,
            nlp_solver = "Ipopt",
            IpoptOptions = {
                linear_solver = "ma27",
                ma57_automatic_scaling = "yes",
                mu_strategy = "adaptive",
                -- nlp_scaling_method = "none",
                check_derivatives_for_naninf = "yes",
                -- tol = 1e-7,
                -- hessian_approximation = "limited-memory",
                -- derivative_test = "second-order",
                -- derivative_test_perturbation = 1e-8,
            },
        },

        write_solution = './../results/out.txt',
        write_mesh_history = './../results/mh_out.txt',
    },
}
