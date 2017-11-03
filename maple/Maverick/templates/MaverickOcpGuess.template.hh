        // +----------------------------+
        // |                            |
        // |   __ _ _   _  ___ ___ ___  |
        // |  / _` | | | |/ _ Y __/ __| |
        // | | (_| | |_| |  __|__ \__ \ |
        // |  \__, |\__,_|\___|___/___/ |
        // |  |___/                     |
        // +----------------------------+

        virtual void evalAtMesh(integer const i_phase,
                        real    const zeta,

                        integer const num_states_controls,  real * states_controls,       real * states_controls_upper_bounds_mult,  real * states_controls_lower_bounds_mult,
                        integer const num_algebraic_states_controls,  real * algebraic_states_controls,       real * algebraic_states_controls_upper_bounds_mult,  real * algebraic_states_controls_lower_bounds_mult,
                        integer const num_fo_eqns,          real * fo_eqns_mult,
                        integer const num_point_constr,     real * point_constr_mult,
                        integer const num_diff_constr,      real * diff_constr_mult
                        ) const;

        virtual void eval(integer const i_phase,
                  real const initial_zeta,   real const final_zeta,
                  
                  integer const num_parameters,          real * parameters,            real * params_upper_bounds_mult,    real * params_lower_bounds_mult,
                  integer const num_boundary_conditions, real * boundary_conditions_mult,
                  integer const num_int_constr,          real * int_constr_mult
                  ) const;
