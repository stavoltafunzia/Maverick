require "readtexttable"
bryson_aircraft_aero = readtexttable("./../data/bryson_aircraft_aero.txt")
us19476 = readtexttable("./../data/us1976_atmosphere.txt")

-- bryson aircraft propulsion data
bac_mach = {0, 0.2, 0.4, 0.6, 0.8, 1, 1.2, 1.4, 1.6, 1.8}
bac_altitude = {0, 5, 10, 15, 20, 25, 30, 40, 50, 70}
for key,value in ipairs(bac_altitude) do bac_altitude[key] = value * 304.8 end
bac_thrust = {24.2, 24.0, 20.3, 17.3, 14.5, 12.2, 10.2,  5.7,  3.4, 0.1,
              28.0, 24.6, 21.1, 18.1, 15.2, 12.8, 10.7,  6.5,  3.9, 0.2,
              28.3, 25.2, 21.9, 18.7, 15.9, 13.4, 11.2,  7.3,  4.4, 0.4,
              30.8, 27.2, 23.8, 20.5, 17.3, 14.7, 12.3,  8.1,  4.9, 0.8,
              34.5, 30.3, 26.6, 23.2, 19.8, 16.8, 14.1,  9.4,  5.6, 1.1,
              37.9, 34.3, 30.4, 26.8, 23.3, 19.8, 16.8, 11.2,  6.8, 1.4,
              36.1, 38.0, 34.9, 31.3, 27.3, 23.6, 20.1, 13.4,  8.3, 1.7,
              36.1, 36.6, 38.5, 36.1, 31.6, 28.1, 24.2, 16.2, 10.0, 2.2,
              36.1, 35.2, 42.1, 38.7, 35.7, 32.0, 28.1, 19.3, 11.9, 2.9,
              36.1, 33.8, 45.7, 41.3, 39.8, 34.6, 31.1, 21.7, 13.3, 3.1}
for key,value in ipairs(bac_thrust) do bac_thrust[key] = value * 4448.222 end

mp = {
    -- model parameters
    R         = 6378145,
    mu        = 3.986e14,
    S         = 49.2386,
    g0        = 9.80665,
    Isp       = 1600,
    H         = 7254.24,
    rho0      = 1.225,

    -- inital and final states
    h_i       = 0,         -- Initial altitude (meters)
    h_f       = 19994.88,  -- Final altitude (meters)
    V_i       = 129.314,   -- Initial speed (m/s)
    V_f       = 295.092,   -- Final speed (m/s)
    fpa_i     = 0,         -- Initial flight path angle (rad)
    fpa_f     = 0,         -- Final flight path angle (rad)
    m_i       = 19050.864, -- Initial mass (kg)

    -- states, controls and parameters bounds
    h_min     = 0,
    h_max     = 21031.2,
    V_min     = 5,
    V_max     = 1000,
    fpa_min   = -40*math.pi/180,
    fpa_max   = 40*math.pi/180,
    m_min     = 22,
    m_max     = 20410,
    alpha_min = -45*math.pi/180,
    alpha_max = 45*math.pi/180,
    T_min     = 10,
    T_max     = 1000,

    -- guess
    t_guess   = 30,

    -- target
    wm        = 1,
    wl        = 0,
    wl1       = 0,
}

spline_type = "Cubic"

mapped_obejcts = {
    CD0 = {
        type = "1arg_spline",
        spline_type = spline_type,
        x = bryson_aircraft_aero.mach,
        y = bryson_aircraft_aero.drag,
    },

    CLalpha = {
        type = "1arg_spline",
        spline_type = spline_type,
        x = bryson_aircraft_aero.mach,
        y = bryson_aircraft_aero.lift,
    },

    Eta = {
        type = "1arg_spline",
        spline_type = spline_type,
        x = bryson_aircraft_aero.mach,
        y = bryson_aircraft_aero.load_factor,
    },

    SoundSpeed = {
        type = "1arg_spline",
        spline_type = "Akima",
        x = us19476.altitude,
        y = us19476.sound_speed,
    },

    Rho = {
        type = "1arg_spline",
        spline_type = spline_type,
        x = us19476.altitude,
        y = us19476.density,
    },

    Thrust = {
        type = "2arg_spline",
        spline_type = "Biquintic",
        x1 = bac_mach, --mach
        x2 = bac_altitude, --altitude
        y = bac_thrust, --height
    },

}

Data = {

    Run0 = {
        info_level = 5,

        Model = {

            Parameters = mp,

            MappedObjects = mapped_obejcts,

            Scaling = {
                Phase0 = {
                    target = 1e2,
                    states_and_controls = "automatic-bounds", -- h, V, fpa, m
                    algebraic_states_and_controls = "automatic-bounds", -- alpha
                    parameters = {1e2},
                    fo_equations = {1e2, 10, 0.1, 1e1},
                    boundary_conditions = {mp.h_max-mp.h_min, mp.V_max-mp.V_min, mp.fpa_max-mp.fpa_min, mp.m_max-mp.m_min, mp.h_max-mp.h_min, mp.V_max-mp.V_min, mp.fpa_max-mp.fpa_min}, -- h_i, V_i, fpa_i, m_i, h_f, V_f, fpa_f
                },
            },
        },

        Mesh = {
            max_iterations = 15,
            tolerance = 1e-6,
            Phase0 = {
               Segments = { {length=1, num_points=50} },
            },
        },

        Solver = {
            max_iterations = 100,
            nlp_solver = "Ipopt",
            -- continuation_mode = "force_warm_start_with_multipliers",
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
