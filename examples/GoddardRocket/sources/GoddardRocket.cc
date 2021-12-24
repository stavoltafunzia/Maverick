#include "GoddardRocket.hh"
#include <tgmath.h>

using namespace Maverick;
using namespace GoddardRocketNamespace;


GoddardRocket::GoddardRocket() {

    _num_p = 1;

    _dim_x = {3};
    _states_names = { {"V", "h", "m"} };

    _dim_ax = {0};
    _algebraic_states_names = { {} };

    _dim_u = {0};
    _controls_names = { {} };

    _dim_au = {1};
    _algebraic_controls_names = { {"F"} };

    _dim_p = {1};
    _parameters_names = { {"T"} };

    _dim_poc = {0};
    _point_constraints_names = { {} };

    _dim_pc = {0};
    _path_constraints_names = { {} };

    _dim_ic = {0};
    _integral_constraints_names = { {} };

    _dim_bc = {3};
    _boundary_conditions_names = { {"init_speed", "init_position", "init_mass"} };

    _dim_ec = {0};
    _event_constraints_names = { {} };

    _post_processing_names = {  {"t"} };
    _differential_post_processing_names = {  {"thrust"} };
    _integral_post_processing_names = {  {"t2"} };

    _model_params = new real[NUM_MODEL_PARAMS]; // deleted automatically by parent's destructor
    _model_params_names = {"Vmin", "Vmax", "Vi", "Vguess", "hmin", "hmax", "hi", "mmin", "mmax", "mi", "mguess", "Tmin", "Tmax", "Tguess", "Fmin", "Fmax", "h0", "sigma", "c", "g", "wl1", "wm", "wlt"};

}

void GoddardRocket::derivedSetup(GC::GenericContainer const & gc) {
    GC::GenericContainer const * gc_params = nullptr;
    try {
        gc_params = &gc("Parameters");
    } catch (...) {
        throw std::runtime_error("Cannot find 'Parameters' inside 'Model' the lua data file\n");
    }
    for (integer i=0; i<NUM_MODEL_PARAMS; i++) {
        _is_model_param_set[i] = gc_params->get_if_exists(_model_params_names[i].c_str(), _model_params[i]);
    }
    std::stringstream missing_params;
    for (integer i=0; i<NUM_MODEL_PARAMS; i++) {
        if (!_is_model_param_set[i])
            missing_params << _model_params_names[i] << "\n";
    }
    if (missing_params.str().compare("")!=0) {
        throw std::runtime_error("GoddardRocket: not all parameters have been set. Missing parameters are:\n" + missing_params.str());
    }


}

void GoddardRocket::printDerivedInfo(std::ostream & out, InfoLevel info_level) const {
    if (info_level >= info_level_verbose) {

    }
}

//   +-------------------------------------+
//   |  _                           _      |
//   | | |__   ___  _   _ _ __   __| |___  |
//   | | '_ \ / _ \| | | | '_ \ / _` / __| |
//   | | |_) | (_) | |_| | | | | (_| \__ \ |
//   | |_.__/ \___/ \__,_|_| |_|\__,_|___/ |
//   |                                     |
//   +-------------------------------------+

void GoddardRocket::getStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_Vmin];
    lower[1] = _model_params[MOD_PAR_INDEX_hmin];
    lower[2] = _model_params[MOD_PAR_INDEX_mmin];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_Vmax];
    upper[1] = _model_params[MOD_PAR_INDEX_hmax];
    upper[2] = _model_params[MOD_PAR_INDEX_mmax];
    }

}

void GoddardRocket::getAlgebraicStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_Fmin];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_Fmax];
    }

}

void GoddardRocket::getParametersBounds(integer const i_phase,
                                                        real          lower[],
                                                        real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_Tmin];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_Tmax];
    }

}

void GoddardRocket::getPathConstraintsBounds(integer const i_phase,
                                                             real    const __zeta,
                                                             real          lower[],
                                                             real          upper[] ) const {
    
    
}

void GoddardRocket::getIntConstraintsBounds(integer const i_phase,
                                                            real    const __zeta_i,
                                                            real    const __zeta_f,
                                                            real          lower[],
                                                            real          upper[] ) const {
    
    
}

void GoddardRocket::getBoundaryConditionsBounds(integer const i_phase,
                                                                real    const __zeta_i,
                                                                real    const __zeta_f,
                                                                real          lower[],
                                                                real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_Vi];
    lower[1] = _model_params[MOD_PAR_INDEX_hi];
    lower[2] = _model_params[MOD_PAR_INDEX_mi];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_Vi];
    upper[1] = _model_params[MOD_PAR_INDEX_hi];
    upper[2] = _model_params[MOD_PAR_INDEX_mi];
    }

}

void GoddardRocket::getPointConstraintsBounds(integer const i_phase,
                                                              real    const __zeta,
                                                              real          lower[],
                                                              real          upper[] ) const {
    
    
}

void GoddardRocket::getEventConstraintsBounds(integer const i_phase,
                                                              real    const __zeta_i,
                                                              real    const __zeta_f,
                                                              real          lower[],
                                                              real          upper[] ) const {
    
    
}

// +----------------------------+
// |                            |
// |   __ _ _   _  ___ ___ ___  |
// |  / _` | | | |/ _ Y __/ __| |
// | | (_| | |_| |  __|__ \__ \ |
// |  \__, |\__,_|\___|___/___/ |
// |  |___/                     |
// +----------------------------+


void GoddardRocket::evalAtMesh(integer const i_phase,
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
    __states_controls[0] = _model_params[MOD_PAR_INDEX_Vguess];
    __states_controls[1] = __zeta * _model_params[MOD_PAR_INDEX_Tguess] * __states_controls[0] + _model_params[MOD_PAR_INDEX_hi];
    __states_controls[2] = _model_params[MOD_PAR_INDEX_mguess];

    }
    if (__algebraic_states_controls) {
    

    }
}

void GoddardRocket::eval(integer const i_phase,
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
    __parameters[0] = _model_params[MOD_PAR_INDEX_Tguess];

    }

}


//   +-----------------------------------+
//   |                                   |
//   |  _ __ ___   __ _ _   _  ___ _ __  |
//   | | '_ ` _ \ / _` | | | |/ _ \ '__| |
//   | | | | | | | (_| | |_| |  __/ |    |
//   | |_| |_| |_|\__,_|\__, |\___|_|    |
//   |                  |___/            |
//   +-----------------------------------+

void GoddardRocket::mayer ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real     & __value ) const {
        __value = -_model_params[MOD_PAR_INDEX_wm] * __final_state_control[1];

}

void GoddardRocket::mayerJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
        __jac_xu_fin[0] = -_model_params[MOD_PAR_INDEX_wm];

}

integer GoddardRocket::mayerJacXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerJacXuInitPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer GoddardRocket::mayerJacXuFinNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void GoddardRocket::mayerJacXuFinPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 1;

}

integer GoddardRocket::mayerJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerJacPPattern ( integer const i_phase, integer cols[] ) const {
        

}

void GoddardRocket::mayerHess ( integer const i_phase,
                                  real const __initial_state_control[],
                                  real const __final_state_control[],
                                  real const __parameters[],
                                  real const __lambda_0,
                                  real       __hess_xu_init_xu_init[],
                                  real       __hess_xu_init_xu_fin[],
                                  real       __hess_xu_init_p[],
                                  real       __hess_xu_fin_xu_fin[],
                                  real       __hess_xu_fin_p[],
                                  real       __hess_p_p[] ) const {
        

}

integer GoddardRocket::mayerHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::mayerHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::mayerHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::mayerHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::mayerHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::mayerHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::mayerHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


//   +--------------------------------------------+
//   |  _                                         |
//   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
//   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
//   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
//   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
//   |          |___/                 |___/       |
//   +--------------------------------------------+

 void GoddardRocket::lagrange ( integer const i_phase,
                           real    const __states_controls[],
                           real    const __state_control_derivatives[],
                           real    const __algebraic_states_controls[],
                           real    const __parameters[],
                           real          __zeta,
                           real        & __value ) const {
       real t5 = pow(__algebraic_states_controls[0], 0.2e1);
    __value = __parameters[0] * (_model_params[MOD_PAR_INDEX_wl1] * t5 - _model_params[MOD_PAR_INDEX_wlt] * __states_controls[0]);

}

 void GoddardRocket::lagrangeJac ( integer const i_phase,
                                real    const __states_controls[],
                                real    const __state_control_derivatives[],
                                real    const __algebraic_states_controls[],
                                real    const __parameters[],
                                real          __zeta,
                                real          __jac_xu[],
                                real          __jac_dxu[],
                                real          __jac_axu[],
                                real          __jac_p[] ) const {
        real t1 =  __parameters[0];
    real t3 =  _model_params[MOD_PAR_INDEX_wlt];
    __jac_xu[0] = - t1 *  t3;
    real t6 =  _model_params[MOD_PAR_INDEX_wl1];
    real t8 =  __algebraic_states_controls[0];
    __jac_axu[0] = 2 * t1 * t6 * t8;
    real t10 = t8 * t8;
    __jac_p[0] = t6 * t10 - t3 * __states_controls[0];

}

integer GoddardRocket::lagrangeJacXuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void GoddardRocket::lagrangeJacXuPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

integer GoddardRocket::lagrangeJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::lagrangeJacDxuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer GoddardRocket::lagrangeJacAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void GoddardRocket::lagrangeJacAxuPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

integer GoddardRocket::lagrangeJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void GoddardRocket::lagrangeJacPPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

 void GoddardRocket::lagrangeHess ( integer const i_phase,
                                real    const __states_controls[],
                                real    const __state_control_derivatives[],
                                real    const __algebraic_states_controls[],
                                real    const __parameters[],
                                real          __zeta,
                                real    const __lambda_0,
                                real          __hess_xu_xu[],
                                real          __hess_xu_dxu[],
                                real          __hess_xu_axu[],
                                real          __hess_xu_p[],
                                real          __hess_dxu_dxu[],
                                real          __hess_dxu_axu[],
                                real          __hess_dxu_p[],
                                real          __hess_axu_axu[],
                                real          __hess_axu_p[],
                                real          __hess_p_p[] ) const {
         __hess_xu_p[0] = -_model_params[MOD_PAR_INDEX_wlt] *  __lambda_0;
    real t6 =  _model_params[MOD_PAR_INDEX_wl1];
    __hess_axu_axu[0] = 2 * __parameters[0] * t6 * __lambda_0;
    __hess_axu_p[0] = 2 * t6 * __algebraic_states_controls[0] * __lambda_0;

}

integer GoddardRocket::lagrangeHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void GoddardRocket::lagrangeHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

integer GoddardRocket::lagrangeHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::lagrangeHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer GoddardRocket::lagrangeHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::lagrangeHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer GoddardRocket::lagrangeHessXuPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
 void GoddardRocket::lagrangeHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;

         cols[0] = 0;

 }

 integer GoddardRocket::lagrangeHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void GoddardRocket::lagrangeHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer GoddardRocket::lagrangeHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void GoddardRocket::lagrangeHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer GoddardRocket::lagrangeHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void GoddardRocket::lagrangeHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer GoddardRocket::lagrangeHessAxuAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
 }
 void GoddardRocket::lagrangeHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;

         cols[0] = 0;

 }

 integer GoddardRocket::lagrangeHessAxuPNnz ( integer const i_phase ) const {
    return 1;
return 0;
 }
 void GoddardRocket::lagrangeHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;

         cols[0] = 0;

 }

 integer GoddardRocket::lagrangeHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void GoddardRocket::lagrangeHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

//   +-------------------------------------------------------------+
//   |   __                                 _   _                  |
//   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
//   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
//   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
//   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
//   |                       |_|                                   |
//   +-------------------------------------------------------------+

void GoddardRocket::foEqns ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t3 = 0.1e1 / __parameters[0];
    real t5 = __algebraic_states_controls[0];
    real t8 = __states_controls[0];
    real t9 = t8 * t8;
    real t16 = exp(-__states_controls[1] / _model_params[MOD_PAR_INDEX_h0]);
    __values[0] = __state_control_derivatives[0] * t3 - (-_model_params[MOD_PAR_INDEX_sigma] * t9 * t16 + t5) / __states_controls[2] + _model_params[MOD_PAR_INDEX_g];
    __values[1] = __state_control_derivatives[1] * t3 - t8;
    __values[2] = __state_control_derivatives[2] * t3 + t5 / _model_params[MOD_PAR_INDEX_c];

}

void GoddardRocket::foEqnsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        real t2 = _model_params[MOD_PAR_INDEX_sigma];
    real t3 = __states_controls[0];
    real t8 = 0.1e1 / _model_params[MOD_PAR_INDEX_h0];
    real t10 = exp(-__states_controls[1] * t8);
    real t11 = __states_controls[2];
    real t12 = 0.1e1 / t11;
    __jac_xu[0] = 0.2e1 * t2 * t3 * t10 * t12;
    __jac_xu[1] = -0.1e1;
    real t15 = t3 * t3;
    real t16 = t2 * t15;
    __jac_xu[2] = -t16 * t8 * t10 * t12;
    real t23 = t11 * t11;
    __jac_xu[3] = (-t16 * t10 + __algebraic_states_controls[0]) / t23;
    real t25 = __parameters[0];
    __jac_dxu[0] = 0.1e1 / t25;
    __jac_dxu[1] = __jac_dxu[0];
    __jac_dxu[2] = __jac_dxu[1];
    __jac_axu[0] = -t12;
    __jac_axu[1] = 0.1e1 / _model_params[MOD_PAR_INDEX_c];
    real t29 = t25 * t25;
    real t30 = 0.1e1 / t29;
    __jac_p[0] = -__state_control_derivatives[0] * t30;
    __jac_p[1] = -__state_control_derivatives[1] * t30;
    __jac_p[2] = -__state_control_derivatives[2] * t30;

}

integer GoddardRocket::foEqnsJacXuNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void GoddardRocket::foEqnsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 0;
    rows[3] = 0;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 1;
    cols[3] = 2;

}

integer GoddardRocket::foEqnsJacDxuNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void GoddardRocket::foEqnsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;

}

integer GoddardRocket::foEqnsJacAxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void GoddardRocket::foEqnsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 2;

        cols[0] = 0;
    cols[1] = 0;

}

integer GoddardRocket::foEqnsJacPNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void GoddardRocket::foEqnsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 0;

}

void GoddardRocket::foEqnsHess(integer const i_phase,
                           real    const __states_controls[],
                           real    const __state_control_derivatives[],
                           real    const __algebraic_states_controls[],
                           real    const __parameters[],
                           real          __zeta,
                           real    const __lambda[],
                           real          __hess_xu_xu[],
                           real          __hess_xu_dxu[],
                           real          __hess_xu_axu[],
                           real          __hess_xu_p[],
                           real          __hess_dxu_dxu[],
                           real          __hess_dxu_axu[],
                           real          __hess_dxu_p[],
                           real          __hess_axu_axu[],
                           real          __hess_axu_p[],
                           real          __hess_p_p[] ) const {
        real t1 = __lambda[0];
    real t3 = _model_params[MOD_PAR_INDEX_sigma];
    real t4 = t1 * t3;
    real t7 = _model_params[MOD_PAR_INDEX_h0];
    real t8 = 0.1e1 / t7;
    real t10 = exp(-__states_controls[1] * t8);
    real t11 = __states_controls[2];
    real t12 = 0.1e1 / t11;
    __hess_xu_xu[0] = 0.2e1 * t4 * t10 * t12;
    real t15 = __states_controls[0];
    real t17 = t8 * t10;
    __hess_xu_xu[1] = -0.2e1 * t4 * t15 * t17 * t12;
    real t22 = t11 * t11;
    real t23 = 0.1e1 / t22;
    __hess_xu_xu[2] = -0.2e1 * t4 * t15 * t10 * t23;
    __hess_xu_xu[3] = __hess_xu_xu[1];
    real t27 = t15 * t15;
    real t28 = t4 * t27;
    real t29 = t7 * t7;
    __hess_xu_xu[4] = t28 / t29 * t10 * t12;
    __hess_xu_xu[5] = t28 * t17 * t23;
    __hess_xu_xu[6] = __hess_xu_xu[2];
    __hess_xu_xu[7] = __hess_xu_xu[5];
    __hess_xu_xu[8] = -0.2e1 * t1 * (-t3 * t27 * t10 + __algebraic_states_controls[0]) / t22 / t11;
    __hess_xu_axu[0] = t1 * t23;
    real t43 = __parameters[0];
    real t44 = t43 * t43;
    real t45 = 0.1e1 / t44;
    __hess_dxu_p[0] = -t1 * t45;
    real t47 = __lambda[1];
    __hess_dxu_p[1] = -t47 * t45;
    real t49 = __lambda[2];
    __hess_dxu_p[2] = -t49 * t45;
    real t54 = 0.1e1 / t44 / t43;
    __hess_p_p[0] = 0.2e1 * t1 * __state_control_derivatives[0] * t54 + 0.2e1 * t47 * __state_control_derivatives[1] * t54 + 0.2e1 * t49 * __state_control_derivatives[2] * t54;

}

integer GoddardRocket::foEqnsHessXuXuNnz ( integer const i_phase ) const {
    return 9;
return 0;
}
void GoddardRocket::foEqnsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[]) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 0;
    rows[4] = 1;
    rows[5] = 2;
    rows[6] = 0;
    rows[7] = 1;
    rows[8] = 2;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 0;
    cols[3] = 1;
    cols[4] = 1;
    cols[5] = 1;
    cols[6] = 2;
    cols[7] = 2;
    cols[8] = 2;

}

integer GoddardRocket::foEqnsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::foEqnsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::foEqnsHessXuAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void GoddardRocket::foEqnsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 2;

}

integer GoddardRocket::foEqnsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::foEqnsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::foEqnsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::foEqnsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::foEqnsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::foEqnsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::foEqnsHessDxuPNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void GoddardRocket::foEqnsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;

}

integer GoddardRocket::foEqnsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::foEqnsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::foEqnsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::foEqnsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::foEqnsHessPPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void GoddardRocket::foEqnsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 0;

}

// +-----------------------------------------------------------------------+
// |      _ _  __  __                       _             _       _        |
// |   __| (_)/ _|/ _|   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _` | | |_| |_   / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | (_| | |  _|  _| | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \__,_|_|_| |_|    \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                       |
// +-----------------------------------------------------------------------+

void GoddardRocket::pathConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    
}

void GoddardRocket::pathConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        

}

integer GoddardRocket::pathConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void GoddardRocket::pathConstraintsHess(integer const i_phase,
                                    real    const __states_controls[],
                                    real    const __state_control_derivatives[],
                                    real    const __algebraic_states_controls[],
                                    real    const __parameters[],
                                    real          __zeta,
                                    real    const __lambda[],
                                    real          __hess_xu_xu[],
                                    real          __hess_xu_dxu[],
                                    real          __hess_xu_axu[],
                                    real          __hess_xu_p[],
                                    real          __hess_dxu_dxu[],
                                    real          __hess_dxu_axu[],
                                    real          __hess_dxu_p[],
                                    real          __hess_axu_axu[],
                                    real          __hess_axu_p[],
                                    real          __hess_p_p[] ) const {
        

}
integer GoddardRocket::pathConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pathConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pathConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


// +----------------------------------------------------------------------------+
// |      _        _                             _             _       _        |
// |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                            |
// +----------------------------------------------------------------------------+


void GoddardRocket::pointConstraints ( integer const i_phase,
                     real    const __states_controls[],
                     real    const __parameters[],
                     real          __zeta,
                     real          __values[] ) const {
    
}

void GoddardRocket::pointConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_p[] ) const {
        

}

integer GoddardRocket::pointConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pointConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pointConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pointConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void GoddardRocket::pointConstraintsHess (integer const i_phase,
                                 real    const __states_controls[],
                                 real    const __parameters[],
                                 real          __zeta,
                                 real    const __lambda[],
                                 real          __hess_xu_xu[],
                                 real          __hess_xu_p[],
                                 real          __hess_p_p[] ) const {
        

 }
integer GoddardRocket::pointConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pointConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pointConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pointConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::pointConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::pointConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +-----------------------------------------------------------------------------------------+
// |  _       _                       _                       _             _       _        |
// | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                  |___/                                                                  |
// +-----------------------------------------------------------------------------------------+

void GoddardRocket::intConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    
}

void GoddardRocket::intConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        

}

integer GoddardRocket::intConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void GoddardRocket::intConstraintsHess(integer const i_phase,
                                    real    const __states_controls[],
                                    real    const __state_control_derivatives[],
                                    real    const __algebraic_states_controls[],
                                    real    const __parameters[],
                                    real          __zeta,
                                    real    const __lambda[],
                                    real          __hess_xu_xu[],
                                    real          __hess_xu_dxu[],
                                    real          __hess_xu_axu[],
                                    real          __hess_xu_p[],
                                    real          __hess_dxu_dxu[],
                                    real          __hess_dxu_axu[],
                                    real          __hess_dxu_p[],
                                    real          __hess_axu_axu[],
                                    real          __hess_axu_p[],
                                    real          __hess_p_p[] ) const {
        

}
integer GoddardRocket::intConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::intConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::intConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

//   +--------------------------------------------------------------------------------------------------+
//   |  _                           _                                        _ _ _   _                  |
//   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
//   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
//   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
//   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
//   |                                           |___/                                                  |
//   +--------------------------------------------------------------------------------------------------+

void GoddardRocket::boundaryConditions ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real       __zeta_i,
               real       __zeta_f,
               real       __values[] ) const {
        __values[0] = __initial_state_control[0];
    __values[1] = __initial_state_control[1];
    __values[2] = __initial_state_control[2];

}

void GoddardRocket::boundaryConditionsJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __zeta_i,
                             real       __zeta_f,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
        __jac_xu_init[0] = 1;
    __jac_xu_init[1] = 1;
    __jac_xu_init[2] = 1;

}

integer GoddardRocket::boundaryConditionsJacXuInitNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void GoddardRocket::boundaryConditionsJacXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;

}

integer GoddardRocket::boundaryConditionsJacXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsJacXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::boundaryConditionsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void GoddardRocket::boundaryConditionsHess ( integer const i_phase,
                                  real const __initial_state_control[],
                                  real const __final_state_control[],
                                  real const __parameters[],
                                  real       __zeta_i,
                                  real       __zeta_f,
                                  real const __lambda[],
                                  real       __hess_xu_init_xu_init[],
                                  real       __hess_xu_init_xu_fin[],
                                  real       __hess_xu_init_p[],
                                  real       __hess_xu_fin_xu_fin[],
                                  real       __hess_xu_fin_p[],
                                  real       __hess_p_p[] ) const {
        

}

integer GoddardRocket::boundaryConditionsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::boundaryConditionsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::boundaryConditionsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::boundaryConditionsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::boundaryConditionsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer GoddardRocket::boundaryConditionsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void GoddardRocket::boundaryConditionsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +--------------------------------------------------------------------------------+
// |                       _                         _             _       _        |
// |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                                |
// +--------------------------------------------------------------------------------+

void GoddardRocket::eventConstraints ( integer const i_phase,
                      real const left_state_control[],
                      real const right_state_control[],
                      real const parameters[],
                      real const __zeta_l,
                      real const __zeta_r,
                      real       __values[] ) const {
}

void GoddardRocket::eventConstraintsJac ( integer const i_phase,
                             real const left_state_control[],
                             real const right_state_control[],
                             real const parameters[],
                             real const __zeta_l,
                             real const __zeta_r,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
}

integer GoddardRocket::eventConstraintsJacXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsJacXuInitPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer GoddardRocket::eventConstraintsJacXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsJacXuFinPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer GoddardRocket::eventConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsJacPPattern ( integer const i_phase,
                               integer       rows[],
                               integer       cols[] ) const {

}

void GoddardRocket::eventConstraintsHess ( integer const i_phase,
                                  real const left_state_control[],
                                  real const right_state_control[],
                                  real const parameters[],
                                  real const __zeta_l,
                                  real const __zeta_r,
                                  real const __lambda[],
                                  real       __hess_xu_init_xu_init[],
                                  real       __hess_xu_init_xu_fin[],
                                  real       __hess_xu_init_p[],
                                  real       __hess_xu_fin_xu_fin[],
                                  real       __hess_xu_fin_p[],
                                  real       __hess_p_p[] ) const {
}

integer GoddardRocket::eventConstraintsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsHessXuInitXuInitPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer GoddardRocket::eventConstraintsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsHessXuInitXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer GoddardRocket::eventConstraintsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsHessXuInitPPattern ( integer const i_phase,
                                     integer       rows[],
                                     integer       cols[] ) const {

}

integer GoddardRocket::eventConstraintsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsHessXuFinXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer GoddardRocket::eventConstraintsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsHessXuFinPPattern ( integer const i_phase,
                                  integer       rows[],
                                  integer       cols[] ) const {

}

integer GoddardRocket::eventConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
}
void GoddardRocket::eventConstraintsHessPPPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

// +------------------------------------------------------------------------+
// |                  _                                      _              |
// |  _ __   ___  ___| |_   _ __  _ __ ___   ___ ___ ___ ___(_)_ __   __ _  |
// | | '_ \ / _ \/ __| __| | '_ \| '__/ _ \ / __/ _ Y __/ __| | '_ \ / _` | |
// | | |_) | (_) \__ \ |_  | |_) | | | (_) | (_|  __|__ \__ \ | | | | (_| | |
// | | .__/ \___/|___/\__| | .__/|_|  \___/ \___\___|___/___/_|_| |_|\__, | |
// | |_|                   |_|                                       |___/  |
// +------------------------------------------------------------------------+

integer GoddardRocket::numberOfPostProcessing( integer const i_phase ) const { return (integer) _post_processing_names.at(i_phase).size(); }
integer GoddardRocket::numberOfDifferentialPostProcessing( integer const i_phase ) const { return (integer) _differential_post_processing_names.at(i_phase).size(); }
integer GoddardRocket::numberOfIntegralPostProcessing( integer const i_phase ) const { return (integer) _integral_post_processing_names.at(i_phase).size(); }
std::string GoddardRocket::postProcessingName( integer const i_phase, integer const i ) const { return _post_processing_names.at(i_phase).at(i); }
std::string GoddardRocket::differentialPostProcessingName( integer const i_phase, integer const i ) const { return _differential_post_processing_names.at(i_phase).at(i); }
std::string GoddardRocket::integralPostProcessingName( integer const i_phase, integer const i ) const { return _integral_post_processing_names.at(i_phase).at(i); }

void GoddardRocket::postProcessing(integer const i_phase,
                                                real    const __states_controls[],
                                                real    const __parameters[],
                                                real          __zeta,
                                                real          __values[] ) const {
        __values[0] = __parameters[0] * __zeta;

}

void GoddardRocket::differentialPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
        __values[0] = __algebraic_states_controls[0];

}

void GoddardRocket::integralPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
        __values[0] = __parameters[0];

}
