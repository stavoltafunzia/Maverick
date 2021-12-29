-- Dati test N1.
require "readtexttable"
road_table = readtexttable("./../data/Adria2D.csv")

mp = {
    g = 9.81,
    m = 1200,
    DX = 1,
    DY = 1,
    Klambda = 10,
    Fx_max = 4,
    Omega_max = 1,
    beta = 0,
    road_width = 5,
    w_reg = 00.001,

    V0 = 7,
    Vf = 7,
    Vmax = 60,
    Pmax = 400*1000,
    Pmin = 400*1000*10,
    max_power = 1000,
    min_power = 0,
    v_guess = 20,
}

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
}

Data = {

        info_level = 4,

        Model = {

            Parameters = mp,

            MappedObjects = mapped_objects,

        },

        Mesh = {
            Phase0 = {
               Segments = { {length=2000, num_points=300} },
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
                tol = 1e-7,
                -- hessian_approximation = "limited-memory",
                -- derivative_test = "second-order",
                -- derivative_test_perturbation = 1e-8,
            },
        },

        Scaling = {
            Phase0 = {
                target = 100,
                states_and_controls = "automatic-bounds", -- n, alpha, V, lambda, ux, uy
                parameters = "automatic-bounds",
                fo_equations = {1, 1, mp.Fx_max * mp.g, 1 },
                path_constraints = "automatic-bounds",
                boundary_conditions = {1, 1, mp.V0, 1, mp.V0},
            },
        },

        write_solution = './../results/out.txt',

}
