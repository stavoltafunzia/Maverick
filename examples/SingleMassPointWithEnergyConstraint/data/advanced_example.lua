-- Dati test N1.
require "readtexttable"
road_table = readtexttable("./../data/Adria2D.txt")

mp = {
    g = 9.81,
    m = 1200,
    DX = 1,
    DY = 1.3,
    Klambda = 10,
    Fx_max = 4,
    Omega_max = 1,
    beta = 0,
    road_width = 5,
    w_reg = 0.05,

    V0 = 7,
    Vf = 7,
    Vmax = 60,
    Pmax = 400*1000,
    Pmin = 400*1000*10,
    Emin = 10 * 1000000,
    Emax = 5 * 1000000,
    max_power = 1000,
    min_power = 0,
    v_guess = 20,
}

road_length=road_table.zeta[#road_table.zeta]

mapped_objects = {
    RoadCurvature = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.kappa,
    },
    RoadX = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.x,
    },
    RoadY = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.y,
    },
    RoadXR = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.xR,
    },
    RoadYR = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.yR,
    },
    RoadXL = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.xL,
    },
    RoadYL = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.yL,
    },
    RoadHeading = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.psi,
    },
    RoadRightWidth = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.rightWidth,
    },
    RoadLeftWidth = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = road_table.zeta,
        y = road_table.leftWidth,
    },
    RegularizedPositive = {
        type = "1arg_reg_pos_sqrt",
        epsilon = 0.01,
    },
}

Data = {

    Run0 = {
        info_level = 4,

        Model = {

            Parameters = mp,

            MappedObjects = mapped_objects,

            Scaling = {
                Phase0 = {
                    target = 50,
                    states_and_controls = "automatic-bounds", -- n, alpha, V, lambda
                    algebraic_states_and_controls = "automatic-bounds", -- ux, uy
                    parameters = "automatic-bounds",
                    fo_equations = {1, 1, mp.Fx_max * mp.g, 1 },
                    path_constraints = "automatic-bounds",
                    integral_constraints = "automatic-bounds",
                    boundary_conditions = {1, 1, mp.V0, 1, mp.V0},
                },

            },
        },

        Mesh = {
            Phase0 = {
               Segments = { {length=road_length, num_points=100} },
            },
        },

        Solver = {
            max_iterations = 1000,
            -- num_threads = 1,
            nlp_solver = "Ipopt",
            IpoptOptions = {
                linear_solver = "mumps",
                ma57_automatic_scaling = "yes",
                mu_strategy = "adaptive",
                -- nlp_scaling_method = "none",
                check_derivatives_for_naninf = "yes",
                tol = 1e-7,
                -- hessian_approximation = "limited-memory",
                -- derivative_test = "second-order",
                -- derivative_test_perturbation = 1e-8,
            },
        },

	  --  write_solution = './../results/out0.txt',
    },

    Run1 = {
        -- do not specify info level: will be unchanged
        -- Model = mp,

        MappedObjects = mapped_objects,

        -- change the mesh: increase num points
        Mesh = {
            Phase0 = {
               Segments = { {length=road_table.zeta[#road_table.zeta], num_points=3000} },
            },
        },

        -- solver: set warm start
        -- NOTE: we do not need to re-define everything. Previous settings are not lost
        Solver = {
            -- start_mode = "warm_start_with_multipliers",
            start_mode = "warm_start",
            -- start_mode = "cold_start",
        },
        write_solution = './../results/out.txt',
    },

}
