#include "HorFlightMinFuel.hh"
#include <tgmath.h>

using namespace Maverick;
using namespace HorFlightMinFuelNamespace;
using namespace MaverickUtils;


HorFlightMinFuel::HorFlightMinFuel() {

    _num_p = 1;

    _dim_x = {2};
    _states_names = { {"x", "m"} };

    _dim_ax = {2};
    _algebraic_states_names = { {"V", "alpha"} };

    _dim_u = {0};
    _controls_names = { {} };

    _dim_au = {1};
    _algebraic_controls_names = { {"f"} };

    _dim_p = {1};
    _parameters_names = { {"T"} };

    _dim_poc = {0};
    _point_constraints_names = { {} };

    _dim_pc = {0};
    _path_constraints_names = { {} };

    _dim_ic = {0};
    _integral_constraints_names = { {} };

    _dim_bc = {4};
    _boundary_conditions_names = { {"init_position", "init_mass", "final_position", "final_mass"} };

    _dim_ec = {0};
    _event_constraints_names = { {} };

    _post_processing_names = {  {"t"} };
    _differential_post_processing_names = {  {"Thrust", "Lift", "Drag", "CdA", "ClA"} };
    _integral_post_processing_names = {  {} };

    _model_params = new real[NUM_MODEL_PARAMS]; // deleted automatically by parent's destructor
    _model_params_names = {"k_fm", "g", "rho", "eta_P", "x_i", "m_i", "x_f", "m_f_min", "m_f_max", "x_min", "m_min", "x_max", "m_max", "V_min", "alpha_min", "V_max", "alpha_max", "f_min", "f_max", "T_min", "T_max", "t_guess", "V_guess", "f_guess", "wmt", "wmm", "wl1", "wl2"};

}

void HorFlightMinFuel::derivedSetup(GC::GenericContainer const & gc) {
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
        throw std::runtime_error("HorFlightMinFuel: not all parameters have been set. Missing parameters are:\n" + missing_params.str());
    }

    GC::GenericContainer const * gc_mapped_objects = nullptr;
    try {
        gc_mapped_objects = &gc("MappedObjects");
    } catch (...) {
        throw std::runtime_error("Cannot find 'MappedObjects' inside 'Model' in the lua data file\n");
    }
    GC::GenericContainer const * gc_function = nullptr;

    _p_C_l = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("C_l"));
    try {
        gc_function = &( (*gc_mapped_objects)("C_l") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'C_l' in the lua data file\n");
    }
    _p_C_l->setup(*gc_function);

    _p_C_d = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("C_d"));
    try {
        gc_function = &( (*gc_mapped_objects)("C_d") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'C_d' in the lua data file\n");
    }
    _p_C_d->setup(*gc_function);

    _p_P = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("P"));
    try {
        gc_function = &( (*gc_mapped_objects)("P") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'P' in the lua data file\n");
    }
    _p_P->setup(*gc_function);


}

void HorFlightMinFuel::printDerivedInfo(std::ostream & out, InfoLevel info_level) const {
    if (info_level >= info_level_verbose) {
    out << "\n";
    _p_C_l->printInfo(out, info_level);
    out << "\n";
    _p_C_d->printInfo(out, info_level);
    out << "\n";
    _p_P->printInfo(out, info_level);
    out << "\n";
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

void HorFlightMinFuel::getStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_x_min];
    lower[1] = _model_params[MOD_PAR_INDEX_m_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_x_max];
    upper[1] = _model_params[MOD_PAR_INDEX_m_max];
    }

}

void HorFlightMinFuel::getAlgebraicStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_V_min];
    lower[1] = _model_params[MOD_PAR_INDEX_alpha_min];
    lower[2] = _model_params[MOD_PAR_INDEX_f_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_V_max];
    upper[1] = _model_params[MOD_PAR_INDEX_alpha_max];
    upper[2] = _model_params[MOD_PAR_INDEX_f_max];
    }

}

void HorFlightMinFuel::getParametersBounds(integer const i_phase,
                                                        real          lower[],
                                                        real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_T_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_T_max];
    }

}

void HorFlightMinFuel::getPathConstraintsBounds(integer const i_phase,
                                                             real    const __zeta,
                                                             real          lower[],
                                                             real          upper[] ) const {
    
    
}

void HorFlightMinFuel::getIntConstraintsBounds(integer const i_phase,
                                                            real    const __zeta_i,
                                                            real    const __zeta_f,
                                                            real          lower[],
                                                            real          upper[] ) const {
    
    
}

void HorFlightMinFuel::getBoundaryConditionsBounds(integer const i_phase,
                                                                real    const __zeta_i,
                                                                real    const __zeta_f,
                                                                real          lower[],
                                                                real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_x_i];
    lower[1] = _model_params[MOD_PAR_INDEX_m_i];
    lower[2] = _model_params[MOD_PAR_INDEX_x_f];
    lower[3] = _model_params[MOD_PAR_INDEX_m_f_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_x_i];
    upper[1] = _model_params[MOD_PAR_INDEX_m_i];
    upper[2] = _model_params[MOD_PAR_INDEX_x_f];
    upper[3] = _model_params[MOD_PAR_INDEX_m_f_max];
    }

}

void HorFlightMinFuel::getPointConstraintsBounds(integer const i_phase,
                                                              real    const __zeta,
                                                              real          lower[],
                                                              real          upper[] ) const {
    
    
}

void HorFlightMinFuel::getEventConstraintsBounds(integer const i_phase,
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


void HorFlightMinFuel::evalAtMesh(integer const i_phase,
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
    real t2 = _model_params[MOD_PAR_INDEX_x_i];
    __states_controls[0] = t2 + (_model_params[MOD_PAR_INDEX_x_f] - t2) * __zeta;
    real t8 = _model_params[MOD_PAR_INDEX_m_i];
    __states_controls[1] = t8 + (_model_params[MOD_PAR_INDEX_m_f_max] - t8) * __zeta;

    }
    if (__algebraic_states_controls) {
    __algebraic_states_controls[0] = _model_params[MOD_PAR_INDEX_V_guess];
    __algebraic_states_controls[1] = 0;
    __algebraic_states_controls[2] = _model_params[MOD_PAR_INDEX_f_guess];

    }
}

void HorFlightMinFuel::eval(integer const i_phase,
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
    __parameters[0] = _model_params[MOD_PAR_INDEX_t_guess];

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

void HorFlightMinFuel::mayer ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real     & __value ) const {
        __value = __parameters[0] * _model_params[MOD_PAR_INDEX_wmt] - _model_params[MOD_PAR_INDEX_wmm] * __final_state_control[1];

}

void HorFlightMinFuel::mayerJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
        __jac_xu_fin[0] = -_model_params[MOD_PAR_INDEX_wmm];
    __jac_p[0] = _model_params[MOD_PAR_INDEX_wmt];

}

integer HorFlightMinFuel::mayerJacXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::mayerJacXuInitPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer HorFlightMinFuel::mayerJacXuFinNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void HorFlightMinFuel::mayerJacXuFinPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 1;

}

integer HorFlightMinFuel::mayerJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void HorFlightMinFuel::mayerJacPPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

void HorFlightMinFuel::mayerHess ( integer const i_phase,
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

integer HorFlightMinFuel::mayerHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::mayerHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::mayerHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::mayerHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::mayerHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::mayerHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::mayerHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::mayerHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::mayerHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::mayerHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::mayerHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::mayerHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


//   +--------------------------------------------+
//   |  _                                         |
//   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
//   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
//   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
//   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
//   |          |___/                 |___/       |
//   +--------------------------------------------+

 void HorFlightMinFuel::lagrange ( integer const i_phase,
                           real    const __states_controls[],
                           real    const __state_control_derivatives[],
                           real    const __algebraic_states_controls[],
                           real    const __parameters[],
                           real          __zeta,
                           real        & __value ) const {
       real t5 = pow(__algebraic_states_controls[1], 0.2e1);
    __value = __parameters[0] * (_model_params[MOD_PAR_INDEX_wl1] * t5 + _model_params[MOD_PAR_INDEX_wl2]);

}

 void HorFlightMinFuel::lagrangeJac ( integer const i_phase,
                                real    const __states_controls[],
                                real    const __state_control_derivatives[],
                                real    const __algebraic_states_controls[],
                                real    const __parameters[],
                                real          __zeta,
                                real          __jac_xu[],
                                real          __jac_dxu[],
                                real          __jac_axu[],
                                real          __jac_p[] ) const {
        real t3 =  _model_params[MOD_PAR_INDEX_wl1];
    real t5 =  __algebraic_states_controls[1];
    __jac_axu[0] = 2 * __parameters[0] * t3 * t5;
    real t7 = t5 * t5;
    __jac_p[0] = t3 * t7 + _model_params[MOD_PAR_INDEX_wl2];

}

integer HorFlightMinFuel::lagrangeJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::lagrangeJacXuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer HorFlightMinFuel::lagrangeJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::lagrangeJacDxuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer HorFlightMinFuel::lagrangeJacAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void HorFlightMinFuel::lagrangeJacAxuPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 1;

}

integer HorFlightMinFuel::lagrangeJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void HorFlightMinFuel::lagrangeJacPPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

 void HorFlightMinFuel::lagrangeHess ( integer const i_phase,
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
         real t3 =  _model_params[MOD_PAR_INDEX_wl1];
    __hess_axu_axu[0] = 2 * __parameters[0] * t3 * __lambda_0;
    __hess_axu_p[0] = 2 * t3 * __algebraic_states_controls[1] * __lambda_0;

}

integer HorFlightMinFuel::lagrangeHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void HorFlightMinFuel::lagrangeHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

integer HorFlightMinFuel::lagrangeHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::lagrangeHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer HorFlightMinFuel::lagrangeHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::lagrangeHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer HorFlightMinFuel::lagrangeHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void HorFlightMinFuel::lagrangeHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer HorFlightMinFuel::lagrangeHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void HorFlightMinFuel::lagrangeHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer HorFlightMinFuel::lagrangeHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void HorFlightMinFuel::lagrangeHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer HorFlightMinFuel::lagrangeHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void HorFlightMinFuel::lagrangeHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer HorFlightMinFuel::lagrangeHessAxuAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
 }
 void HorFlightMinFuel::lagrangeHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 1;

         cols[0] = 1;

 }

 integer HorFlightMinFuel::lagrangeHessAxuPNnz ( integer const i_phase ) const {
    return 1;
return 0;
 }
 void HorFlightMinFuel::lagrangeHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;

         cols[0] = 1;

 }

 integer HorFlightMinFuel::lagrangeHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void HorFlightMinFuel::lagrangeHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

//   +-------------------------------------------------------------+
//   |   __                                 _   _                  |
//   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
//   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
//   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
//   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
//   |                       |_|                                   |
//   +-------------------------------------------------------------+

void HorFlightMinFuel::foEqns ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t3 = 0.1e1 / __parameters[0];
    real t5 = __algebraic_states_controls[0];
    __values[0] = __state_control_derivatives[0] * t3 - t5;
    real t8 = __algebraic_states_controls[2];
    __values[1] = __state_control_derivatives[1] * t3 + t8 * _model_params[MOD_PAR_INDEX_k_fm];
    real t13 = _model_params[MOD_PAR_INDEX_rho];
    real t14 = __algebraic_states_controls[1];
    real t15 = _p_C_d->funcEval(t14);
    real t17 = t5 * t5;
    real t20 = _p_P->funcEval(t8);
    real t23 = t20 * _model_params[MOD_PAR_INDEX_eta_P];
    real t24 = 0.1e1 / t5;
    real t25 = 0.1745329252e-1 * t14;
    real t26 = cos(t25);
    __values[2] = 0.5e0 * t13 * t15 * t17 - t23 * t24 * t26;
    real t29 = _p_C_l->funcEval(t14);
    real t33 = sin(t25);
    __values[3] = 0.5e0 * t13 * t29 * t17 + t23 * t24 * t33 - __states_controls[1] * _model_params[MOD_PAR_INDEX_g];

}

void HorFlightMinFuel::foEqnsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        __jac_xu[0] = -_model_params[MOD_PAR_INDEX_g];
    real t3 = __parameters[0];
    __jac_dxu[0] = 0.1e1 / t3;
    __jac_dxu[1] = __jac_dxu[0];
    __jac_axu[0] = -1;
    real t5 = _model_params[MOD_PAR_INDEX_rho];
    real t6 = __algebraic_states_controls[1];
    real t7 = _p_C_d->funcEval(t6);
    real t9 = __algebraic_states_controls[0];
    real t12 = __algebraic_states_controls[2];
    real t13 = _p_P->funcEval(t12);
    real t15 = _model_params[MOD_PAR_INDEX_eta_P];
    real t16 = t13 * t15;
    real t17 = t9 * t9;
    real t18 = 0.1e1 / t17;
    real t19 = 0.1745329252e-1 * t6;
    real t20 = cos(t19);
    __jac_axu[1] =  (0.10e1 * t5 * t7 * t9 + t16 * t18 * t20);
    real t23 = _p_C_l->funcEval(t6);
    real t27 = sin(t19);
    __jac_axu[2] =  (0.10e1 * t5 * t23 * t9 - t16 * t18 * t27);
    real t30 = _p_C_d->funcEval_D_1(t6);
    real t34 = 0.1e1 / t9;
    real t35 = t34 * t27;
    __jac_axu[3] =  (0.5e0 * t5 * t30 * t17 + 0.1745329252e-1 * t16 * t35);
    real t38 = _p_C_l->funcEval_D_1(t6);
    real t42 = t34 * t20;
    __jac_axu[4] =  (0.5e0 * t5 * t38 * t17 + 0.1745329252e-1 * t16 * t42);
    __jac_axu[5] = _model_params[MOD_PAR_INDEX_k_fm];
    real t46 = _p_P->funcEval_D_1(t12);
    real t47 = t46 * t15;
    __jac_axu[6] = - (t47 * t42);
    __jac_axu[7] =  (t47 * t35);
    real t50 = t3 * t3;
    real t51 = 0.1e1 / t50;
    __jac_p[0] = -__state_control_derivatives[0] * t51;
    __jac_p[1] = -__state_control_derivatives[1] * t51;

}

integer HorFlightMinFuel::foEqnsJacXuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void HorFlightMinFuel::foEqnsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 3;

        cols[0] = 1;

}

integer HorFlightMinFuel::foEqnsJacDxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void HorFlightMinFuel::foEqnsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;

        cols[0] = 0;
    cols[1] = 1;

}

integer HorFlightMinFuel::foEqnsJacAxuNnz ( integer const i_phase ) const {
    return 8;
return 0;
}
void HorFlightMinFuel::foEqnsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 2;
    rows[2] = 3;
    rows[3] = 2;
    rows[4] = 3;
    rows[5] = 1;
    rows[6] = 2;
    rows[7] = 3;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 0;
    cols[3] = 1;
    cols[4] = 1;
    cols[5] = 2;
    cols[6] = 2;
    cols[7] = 2;

}

integer HorFlightMinFuel::foEqnsJacPNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void HorFlightMinFuel::foEqnsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;

        cols[0] = 0;
    cols[1] = 0;

}

void HorFlightMinFuel::foEqnsHess(integer const i_phase,
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
        real t1 =  __lambda[0];
    real t2 = __parameters[0];
    real t3 = t2 * t2;
    real t4 = 0.1e1 / t3;
    __hess_dxu_p[0] = - t1 * t4;
    real t6 =  __lambda[1];
    __hess_dxu_p[1] = - t6 * t4;
    real t8 = __lambda[3];
    real t10 = _model_params[MOD_PAR_INDEX_rho];
    real t11 = __algebraic_states_controls[1];
    real t12 = _p_C_l->funcEval(t11);
    real t15 = __algebraic_states_controls[2];
    real t16 = _p_P->funcEval(t15);
    real t18 = _model_params[MOD_PAR_INDEX_eta_P];
    real t19 = t16 * t18;
    real t20 = __algebraic_states_controls[0];
    real t21 = t20 * t20;
    real t23 = 0.1e1 / t21 / t20;
    real t24 = 0.1745329252e-1 * t11;
    real t25 = sin(t24);
    real t31 = __lambda[2];
    real t32 = _p_C_d->funcEval(t11);
    real t35 = cos(t24);
    __hess_axu_axu[0] = t8 * (0.10e1 * t10 * t12 + 0.2e1 * t19 * t23 * t25) + t31 * (0.10e1 * t10 * t32 - 0.2e1 * t19 * t23 * t35);
    real t41 = _p_C_l->funcEval_D_1(t11);
    real t45 = 0.1e1 / t21;
    real t51 = _p_C_d->funcEval_D_1(t11);
    __hess_axu_axu[1] = t8 * (0.10e1 * t10 * t41 * t20 - 0.1745329252e-1 * t19 * t45 * t35) + t31 * (0.10e1 * t10 * t51 * t20 - 0.1745329252e-1 * t19 * t45 * t25);
    real t60 = _p_P->funcEval_D_1(t15);
    real t61 = t8 * t60;
    real t62 = t18 * t45;
    real t65 = t31 * t60;
    __hess_axu_axu[2] = -t61 * t62 * t25 + t65 * t62 * t35;
    __hess_axu_axu[3] = __hess_axu_axu[1];
    real t68 = _p_C_l->funcEval_D_1_1(t11);
    real t72 = 0.1e1 / t20;
    real t78 = _p_C_d->funcEval_D_1_1(t11);
    __hess_axu_axu[4] = t8 * (0.5e0 * t10 * t68 * t21 - 0.3046174198e-3 * t19 * t72 * t25) + t31 * (0.5e0 * t10 * t78 * t21 + 0.3046174198e-3 * t19 * t72 * t35);
    real t87 = t18 * t72;
    real t88 = t87 * t35;
    real t91 = t87 * t25;
    __hess_axu_axu[5] = 0.1745329252e-1 * t61 * t88 + 0.1745329252e-1 * t65 * t91;
    __hess_axu_axu[6] = __hess_axu_axu[2];
    __hess_axu_axu[7] = __hess_axu_axu[5];
    real t94 = _p_P->funcEval_D_1_1(t15);
    __hess_axu_axu[8] = -t31 * t94 * t88 + t8 * t94 * t91;
    real t102 =  (0.1e1 / t3 / t2);
    __hess_p_p[0] = 2 * t1 * __state_control_derivatives[0] * t102 + 2 * t6 * __state_control_derivatives[1] * t102;

}

integer HorFlightMinFuel::foEqnsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::foEqnsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[]) const {
        

        

}

integer HorFlightMinFuel::foEqnsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::foEqnsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::foEqnsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::foEqnsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::foEqnsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::foEqnsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::foEqnsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::foEqnsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::foEqnsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::foEqnsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::foEqnsHessDxuPNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void HorFlightMinFuel::foEqnsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;

        cols[0] = 0;
    cols[1] = 1;

}

integer HorFlightMinFuel::foEqnsHessAxuAxuNnz ( integer const i_phase ) const {
    return 9;
return 0;
}
void HorFlightMinFuel::foEqnsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
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

integer HorFlightMinFuel::foEqnsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::foEqnsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::foEqnsHessPPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void HorFlightMinFuel::foEqnsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
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

void HorFlightMinFuel::pathConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    
}

void HorFlightMinFuel::pathConstraintsJac (integer const i_phase,
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

integer HorFlightMinFuel::pathConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void HorFlightMinFuel::pathConstraintsHess(integer const i_phase,
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
integer HorFlightMinFuel::pathConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pathConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pathConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


// +----------------------------------------------------------------------------+
// |      _        _                             _             _       _        |
// |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                            |
// +----------------------------------------------------------------------------+


void HorFlightMinFuel::pointConstraints ( integer const i_phase,
                     real    const __states_controls[],
                     real    const __parameters[],
                     real          __zeta,
                     real          __values[] ) const {
    
}

void HorFlightMinFuel::pointConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_p[] ) const {
        

}

integer HorFlightMinFuel::pointConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pointConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pointConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pointConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void HorFlightMinFuel::pointConstraintsHess (integer const i_phase,
                                 real    const __states_controls[],
                                 real    const __parameters[],
                                 real          __zeta,
                                 real    const __lambda[],
                                 real          __hess_xu_xu[],
                                 real          __hess_xu_p[],
                                 real          __hess_p_p[] ) const {
        

 }
integer HorFlightMinFuel::pointConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pointConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pointConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pointConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::pointConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::pointConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +-----------------------------------------------------------------------------------------+
// |  _       _                       _                       _             _       _        |
// | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                  |___/                                                                  |
// +-----------------------------------------------------------------------------------------+

void HorFlightMinFuel::intConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    
}

void HorFlightMinFuel::intConstraintsJac (integer const i_phase,
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

integer HorFlightMinFuel::intConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void HorFlightMinFuel::intConstraintsHess(integer const i_phase,
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
integer HorFlightMinFuel::intConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::intConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::intConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

//   +--------------------------------------------------------------------------------------------------+
//   |  _                           _                                        _ _ _   _                  |
//   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
//   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
//   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
//   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
//   |                                           |___/                                                  |
//   +--------------------------------------------------------------------------------------------------+

void HorFlightMinFuel::boundaryConditions ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real       __zeta_i,
               real       __zeta_f,
               real       __values[] ) const {
        __values[0] = __initial_state_control[0];
    __values[1] = __initial_state_control[1];
    __values[2] = __final_state_control[0];
    __values[3] = __final_state_control[1];

}

void HorFlightMinFuel::boundaryConditionsJac ( integer const i_phase,
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
    __jac_xu_fin[0] = 1;
    __jac_xu_fin[1] = 1;

}

integer HorFlightMinFuel::boundaryConditionsJacXuInitNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void HorFlightMinFuel::boundaryConditionsJacXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;

        cols[0] = 0;
    cols[1] = 1;

}

integer HorFlightMinFuel::boundaryConditionsJacXuFinNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void HorFlightMinFuel::boundaryConditionsJacXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 2;
    rows[1] = 3;

        cols[0] = 0;
    cols[1] = 1;

}

integer HorFlightMinFuel::boundaryConditionsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::boundaryConditionsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void HorFlightMinFuel::boundaryConditionsHess ( integer const i_phase,
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

integer HorFlightMinFuel::boundaryConditionsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::boundaryConditionsHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::boundaryConditionsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::boundaryConditionsHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::boundaryConditionsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::boundaryConditionsHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::boundaryConditionsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::boundaryConditionsHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::boundaryConditionsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::boundaryConditionsHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer HorFlightMinFuel::boundaryConditionsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void HorFlightMinFuel::boundaryConditionsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +--------------------------------------------------------------------------------+
// |                       _                         _             _       _        |
// |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                                |
// +--------------------------------------------------------------------------------+

void HorFlightMinFuel::eventConstraints ( integer const i_phase,
                      real const left_state_control[],
                      real const right_state_control[],
                      real const parameters[],
                      real const __zeta_l,
                      real const __zeta_r,
                      real       __values[] ) const {
}

void HorFlightMinFuel::eventConstraintsJac ( integer const i_phase,
                             real const left_state_control[],
                             real const right_state_control[],
                             real const parameters[],
                             real const __zeta_l,
                             real const __zeta_r,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
}

integer HorFlightMinFuel::eventConstraintsJacXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsJacXuInitPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer HorFlightMinFuel::eventConstraintsJacXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsJacXuFinPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer HorFlightMinFuel::eventConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsJacPPattern ( integer const i_phase,
                               integer       rows[],
                               integer       cols[] ) const {

}

void HorFlightMinFuel::eventConstraintsHess ( integer const i_phase,
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

integer HorFlightMinFuel::eventConstraintsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsHessXuInitXuInitPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer HorFlightMinFuel::eventConstraintsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsHessXuInitXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer HorFlightMinFuel::eventConstraintsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsHessXuInitPPattern ( integer const i_phase,
                                     integer       rows[],
                                     integer       cols[] ) const {

}

integer HorFlightMinFuel::eventConstraintsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsHessXuFinXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer HorFlightMinFuel::eventConstraintsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsHessXuFinPPattern ( integer const i_phase,
                                  integer       rows[],
                                  integer       cols[] ) const {

}

integer HorFlightMinFuel::eventConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
}
void HorFlightMinFuel::eventConstraintsHessPPPattern ( integer const i_phase,
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

integer HorFlightMinFuel::numberOfPostProcessing( integer const i_phase ) const { return (integer) _post_processing_names.at(i_phase).size(); }
integer HorFlightMinFuel::numberOfDifferentialPostProcessing( integer const i_phase ) const { return (integer) _differential_post_processing_names.at(i_phase).size(); }
integer HorFlightMinFuel::numberOfIntegralPostProcessing( integer const i_phase ) const { return (integer) _integral_post_processing_names.at(i_phase).size(); }
std::string HorFlightMinFuel::postProcessingName( integer const i_phase, integer const i ) const { return _post_processing_names.at(i_phase).at(i); }
std::string HorFlightMinFuel::differentialPostProcessingName( integer const i_phase, integer const i ) const { return _differential_post_processing_names.at(i_phase).at(i); }
std::string HorFlightMinFuel::integralPostProcessingName( integer const i_phase, integer const i ) const { return _integral_post_processing_names.at(i_phase).at(i); }

void HorFlightMinFuel::postProcessing(integer const i_phase,
                                                real    const __states_controls[],
                                                real    const __parameters[],
                                                real          __zeta,
                                                real          __values[] ) const {
        __values[0] = __parameters[0] * __zeta;

}

void HorFlightMinFuel::differentialPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
        real t2 = _p_P->funcEval(__algebraic_states_controls[2]);
    real t6 = __algebraic_states_controls[0];
    __values[0] = t2 * _model_params[MOD_PAR_INDEX_eta_P] / t6;
    real t9 = _model_params[MOD_PAR_INDEX_rho];
    real t10 = __algebraic_states_controls[1];
    real t11 = _p_C_d->funcEval(t10);
    real t13 = t6 * t6;
    __values[1] = 0.5e0 * t9 * t11 * t13;
    real t15 = _p_C_l->funcEval(t10);
    __values[2] = 0.5e0 * t9 * t15 * t13;
    __values[3] = t11;
    __values[4] = t15;

}

void HorFlightMinFuel::integralPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
    
}
