// +----------------------------+
// |                            |
// |   __ _ _   _  ___ ___ ___  |
// |  / _` | | | |/ _ Y __/ __| |
// | | (_| | |_| |  __|__ \__ \ |
// |  \__, |\__,_|\___|___/___/ |
// |  |___/                     |
// +----------------------------+


void _MAVERICK_ENTER_CLASS_NAME::evalAtMesh(integer const i_phase,
                        real    const __zeta,

                        integer const num_states_controls,  real * __states_controls,       real * states_controls_upper_bounds_mult,  real * states_controls_lower_bounds_mult,
                        integer const num_algebraic_states_controls,  real * __algebraic_states_controls,       real * algebraic_states_controls_upper_bounds_mult,  real * algebraic_states_controls_lower_bounds_mult,
                        integer const num_fo_eqns,          real * fo_eqns_mult,
                        integer const num_point_constr,     real * point_constr_mult,
                        integer const num_path_constr,      real * path_constr_mult
                ) const {

    // call the super classh method to initialise the guess to zero and to make some consistency check
    MaverickOcp::evalAtMesh(i_phase,
                            __zeta,
                            num_states_controls,  __states_controls,   states_controls_upper_bounds_mult, states_controls_lower_bounds_mult,
                            num_algebraic_states_controls,  __algebraic_states_controls, algebraic_states_controls_upper_bounds_mult, algebraic_states_controls_lower_bounds_mult,
                            num_fo_eqns,          fo_eqns_mult,
                            num_point_constr,     point_constr_mult,
                            num_path_constr,      path_constr_mult
                        );

    // write the guess
    if (__states_controls) {
_MAVERICK_ENTER_GUESS_EVAL_AT_MESH_BODY
    }
    if (__algebraic_states_controls) {
_MAVERICK_ENTER_GUESS_EVAL_AT_MESH_ALGEBRAIC_BODY
    }
}

void _MAVERICK_ENTER_CLASS_NAME::eval(integer const i_phase,
                  real const __zeta_i,   real const __zeta_f,

                  integer const num_parameters,          real * __parameters,            real * params_upper_bounds_mult,    real * params_lower_bounds_mult,
                  integer const num_boundary_conditions, real * boundary_conditions_mult,
                  integer const num_int_constr,          real * int_constr_mult
          ) const {

    // call the super classh method to initialise the guess to zero and to make some consistency check
    MaverickOcp::eval(i_phase,
                      __zeta_i,   __zeta_f,

                      num_parameters,          __parameters,            params_upper_bounds_mult,    params_lower_bounds_mult,
                      num_boundary_conditions, boundary_conditions_mult,
                      num_int_constr,          int_constr_mult
                  );

    // write the guess
    if (__parameters) {
_MAVERICK_ENTER_GUESS_EVAL_BODY
    }

}
