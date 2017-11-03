#include "MinimumTimeToClimb.hh"
#include <tgmath.h>

using namespace Maverick;
using namespace MinimumTimeToClimbNamespace;
using namespace MaverickUtils;


MinimumTimeToClimb::MinimumTimeToClimb() {

    _num_p = 1;

    _dim_x = {4};
    _states_names = { {"h", "V", "fpa", "m"} };

    _dim_ax = {0};
    _algebraic_states_names = { {} };

    _dim_u = {0};
    _controls_names = { {} };

    _dim_au = {1};
    _algebraic_controls_names = { {"alpha"} };

    _dim_p = {1};
    _parameters_names = { {"T"} };

    _dim_poc = {0};
    _point_constraints_names = { {} };

    _dim_pc = {0};
    _path_constraints_names = { {} };

    _dim_ic = {0};
    _integral_constraints_names = { {} };

    _dim_bc = {7};
    _boundary_conditions_names = { {"init_altitude", "init_speed", "init_fpa", "init_mass", "final_altitude", "final_speed", "final_fpa"} };

    _dim_ec = {0};
    _event_constraints_names = { {} };

    _post_processing_names = {  {"t", "SoundSpeed"} };
    _differential_post_processing_names = {  {"r", "Mach", "CD0", "CLalpha", "eta", "Thrust", "CD", "CL", "q", "Drag", "Lift"} };
    _integral_post_processing_names = {  {} };

    _model_params = new real[NUM_MODEL_PARAMS]; // deleted automatically by parent's destructor
    _model_params_names = {"mu", "S", "g0", "Isp", "R", "h_i", "V_i", "fpa_i", "m_i", "h_f", "V_f", "fpa_f", "h_min", "V_min", "fpa_min", "m_min", "h_max", "V_max", "fpa_max", "m_max", "alpha_min", "alpha_max", "T_min", "T_max", "t_guess", "wm", "wl1", "wl"};

}

void MinimumTimeToClimb::derivedSetup(GC::GenericContainer const & gc) {
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
        throw std::runtime_error("MinimumTimeToClimb: not all parameters have been set. Missing parameters are:\n" + missing_params.str());
    }

    GC::GenericContainer const * gc_mapped_objects = nullptr;
    try {
        gc_mapped_objects = &gc("MappedObjects");
    } catch (...) {
        throw std::runtime_error("Cannot find 'MappedObjects' inside 'Model' in the lua data file\n");
    }
    GC::GenericContainer const * gc_function = nullptr;

    _p_CD0 = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("CD0"));
    try {
        gc_function = &( (*gc_mapped_objects)("CD0") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'CD0' in the lua data file\n");
    }
    _p_CD0->setup(*gc_function);

    _p_Eta = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("Eta"));
    try {
        gc_function = &( (*gc_mapped_objects)("Eta") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'Eta' in the lua data file\n");
    }
    _p_Eta->setup(*gc_function);

    _p_CLalpha = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("CLalpha"));
    try {
        gc_function = &( (*gc_mapped_objects)("CLalpha") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'CLalpha' in the lua data file\n");
    }
    _p_CLalpha->setup(*gc_function);

    _p_Rho = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("Rho"));
    try {
        gc_function = &( (*gc_mapped_objects)("Rho") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'Rho' in the lua data file\n");
    }
    _p_Rho->setup(*gc_function);

    _p_SoundSpeed = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("SoundSpeed"));
    try {
        gc_function = &( (*gc_mapped_objects)("SoundSpeed") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'SoundSpeed' in the lua data file\n");
    }
    _p_SoundSpeed->setup(*gc_function);

    _p_Thrust = std::unique_ptr<GenericFunction2AInterface> (getGenericFunction2A("Thrust"));
    try {
        gc_function = &( (*gc_mapped_objects)("Thrust") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'Thrust' in the lua data file\n");
    }
    _p_Thrust->setup(*gc_function);


}

void MinimumTimeToClimb::printDerivedInfo(std::ostream & out, InfoLevel info_level) const {
    if (info_level >= info_level_verbose) {
    out << "\n";
    _p_CD0->printInfo(out, info_level);
    out << "\n";
    _p_Eta->printInfo(out, info_level);
    out << "\n";
    _p_CLalpha->printInfo(out, info_level);
    out << "\n";
    _p_Rho->printInfo(out, info_level);
    out << "\n";
    _p_SoundSpeed->printInfo(out, info_level);
    out << "\n";
    _p_Thrust->printInfo(out, info_level);
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

integer MinimumTimeToClimb::getStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_h_min];
    lower[1] = _model_params[MOD_PAR_INDEX_V_min];
    lower[2] = _model_params[MOD_PAR_INDEX_fpa_min];
    lower[3] = _model_params[MOD_PAR_INDEX_m_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_h_max];
    upper[1] = _model_params[MOD_PAR_INDEX_V_max];
    upper[2] = _model_params[MOD_PAR_INDEX_fpa_max];
    upper[3] = _model_params[MOD_PAR_INDEX_m_max];
    }

    return 0;
}

integer MinimumTimeToClimb::getAlgebraicStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_alpha_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_alpha_max];
    }

    return 0;
}

integer MinimumTimeToClimb::getParametersBounds(integer const i_phase,
                                                        real          lower[],
                                                        real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_T_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_T_max];
    }

    return 0;
}

integer MinimumTimeToClimb::getPathConstraintsBounds(integer const i_phase,
                                                             real    const __zeta,
                                                             real          lower[],
                                                             real          upper[] ) const {
    
    
    return 0;
}

integer MinimumTimeToClimb::getIntConstraintsBounds(integer const i_phase,
                                                            real    const __zeta_i,
                                                            real    const __zeta_f,
                                                            real          lower[],
                                                            real          upper[] ) const {
    
    
    return 0;
}

integer MinimumTimeToClimb::getBoundaryConditionsBounds(integer const i_phase,
                                                                real    const __zeta_i,
                                                                real    const __zeta_f,
                                                                real          lower[],
                                                                real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_h_i];
    lower[1] = _model_params[MOD_PAR_INDEX_V_i];
    lower[2] = _model_params[MOD_PAR_INDEX_fpa_i];
    lower[3] = _model_params[MOD_PAR_INDEX_m_i];
    lower[4] = _model_params[MOD_PAR_INDEX_h_f];
    lower[5] = _model_params[MOD_PAR_INDEX_V_f];
    lower[6] = _model_params[MOD_PAR_INDEX_fpa_f];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_h_i];
    upper[1] = _model_params[MOD_PAR_INDEX_V_i];
    upper[2] = _model_params[MOD_PAR_INDEX_fpa_i];
    upper[3] = _model_params[MOD_PAR_INDEX_m_i];
    upper[4] = _model_params[MOD_PAR_INDEX_h_f];
    upper[5] = _model_params[MOD_PAR_INDEX_V_f];
    upper[6] = _model_params[MOD_PAR_INDEX_fpa_f];
    }

    return 0;
}

integer MinimumTimeToClimb::getPointConstraintsBounds(integer const i_phase,
                                                              real    const __zeta,
                                                              real          lower[],
                                                              real          upper[] ) const {
    
    
    return 0;
}

integer MinimumTimeToClimb::getEventConstraintsBounds(integer const i_phase,
                                                              real    const __zeta_i,
                                                              real    const __zeta_f,
                                                              real          lower[],
                                                              real          upper[] ) const {
    
    
    return 0;
}

// +----------------------------+
// |                            |
// |   __ _ _   _  ___ ___ ___  |
// |  / _` | | | |/ _ Y __/ __| |
// | | (_| | |_| |  __|__ \__ \ |
// |  \__, |\__,_|\___|___/___/ |
// |  |___/                     |
// +----------------------------+


void MinimumTimeToClimb::evalAtMesh(integer const i_phase,
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
    real t2 = _model_params[MOD_PAR_INDEX_V_i];
    __states_controls[1] = t2 + (_model_params[MOD_PAR_INDEX_V_f] - t2) * __zeta;
    real t8 = _model_params[MOD_PAR_INDEX_h_i];
    __states_controls[0] = t8 + (_model_params[MOD_PAR_INDEX_h_f] - t8) * __zeta;
    real t14 = _model_params[MOD_PAR_INDEX_fpa_i];
    __states_controls[2] = t14 + (_model_params[MOD_PAR_INDEX_fpa_f] - t14) * __zeta;
    __states_controls[3] = _model_params[MOD_PAR_INDEX_m_i];

    }
}

void MinimumTimeToClimb::eval(integer const i_phase,
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

integer MinimumTimeToClimb::mayer ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real     & __value ) const {
        __value = _model_params[MOD_PAR_INDEX_wm] * __parameters[0];

    return 0;
}

integer MinimumTimeToClimb::mayerJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
        __jac_p[0] = _model_params[MOD_PAR_INDEX_wm];

    return 0;
}

integer MinimumTimeToClimb::mayerJacXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerJacXuInitPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer MinimumTimeToClimb::mayerJacXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerJacXuFinPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer MinimumTimeToClimb::mayerJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void MinimumTimeToClimb::mayerJacPPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

integer MinimumTimeToClimb::mayerHess ( integer const i_phase,
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
        

    return 0;
}

integer MinimumTimeToClimb::mayerHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::mayerHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::mayerHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::mayerHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::mayerHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::mayerHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::mayerHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


//   +--------------------------------------------+
//   |  _                                         |
//   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
//   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
//   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
//   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
//   |          |___/                 |___/       |
//   +--------------------------------------------+

 integer MinimumTimeToClimb::lagrange ( integer const i_phase,
                           real    const __states_controls[],
                           real    const __state_control_derivatives[],
                           real    const __algebraic_states_controls[],
                           real    const __parameters[],
                           real          __zeta,
                           real        & __value ) const {
       real t5 = pow(__algebraic_states_controls[0], 0.2e1);
    __value = __parameters[0] * (_model_params[MOD_PAR_INDEX_wl1] * t5 + _model_params[MOD_PAR_INDEX_wl]);

   return 0;
}

 integer MinimumTimeToClimb::lagrangeJac ( integer const i_phase,
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
    real t5 =  __algebraic_states_controls[0];
    __jac_axu[0] = 2 * __parameters[0] * t3 * t5;
    real t7 = t5 * t5;
    __jac_p[0] = t3 * t7 + _model_params[MOD_PAR_INDEX_wl];

    return 0;
}

integer MinimumTimeToClimb::lagrangeJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::lagrangeJacXuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer MinimumTimeToClimb::lagrangeJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::lagrangeJacDxuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer MinimumTimeToClimb::lagrangeJacAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void MinimumTimeToClimb::lagrangeJacAxuPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

integer MinimumTimeToClimb::lagrangeJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void MinimumTimeToClimb::lagrangeJacPPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

 integer MinimumTimeToClimb::lagrangeHess ( integer const i_phase,
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
    __hess_axu_p[0] = 2 * t3 * __algebraic_states_controls[0] * __lambda_0;

     return 0;
}

integer MinimumTimeToClimb::lagrangeHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void MinimumTimeToClimb::lagrangeHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

integer MinimumTimeToClimb::lagrangeHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::lagrangeHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer MinimumTimeToClimb::lagrangeHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::lagrangeHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer MinimumTimeToClimb::lagrangeHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void MinimumTimeToClimb::lagrangeHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer MinimumTimeToClimb::lagrangeHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void MinimumTimeToClimb::lagrangeHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer MinimumTimeToClimb::lagrangeHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void MinimumTimeToClimb::lagrangeHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer MinimumTimeToClimb::lagrangeHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void MinimumTimeToClimb::lagrangeHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer MinimumTimeToClimb::lagrangeHessAxuAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
 }
 void MinimumTimeToClimb::lagrangeHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;

         cols[0] = 0;

 }

 integer MinimumTimeToClimb::lagrangeHessAxuPNnz ( integer const i_phase ) const {
    return 1;
return 0;
 }
 void MinimumTimeToClimb::lagrangeHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;

         cols[0] = 0;

 }

 integer MinimumTimeToClimb::lagrangeHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void MinimumTimeToClimb::lagrangeHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

//   +-------------------------------------------------------------+
//   |   __                                 _   _                  |
//   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
//   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
//   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
//   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
//   |                       |_|                                   |
//   +-------------------------------------------------------------+

integer MinimumTimeToClimb::foEqns ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t3 = 0.1e1 / __parameters[0];
    real t5 = __states_controls[1];
    real t6 = __states_controls[2];
    real t7 = sin(t6);
    __values[0] = __state_control_derivatives[0] * t3 - t5 * t7;
    real t11 = __states_controls[0];
    real t12 = _p_SoundSpeed->funcEval(t11);
    real t14 = t5 / t12;
    real t15 = _p_Thrust->funcEval(t14, t11);
    real t16 = __algebraic_states_controls[0];
    real t17 = cos(t16);
    real t19 = _p_Rho->funcEval(t11);
    real t20 = t5 * t5;
    real t21 = t19 * t20;
    real t23 = _model_params[MOD_PAR_INDEX_S];
    real t24 = _p_CD0->funcEval(t14);
    real t25 = _p_Eta->funcEval(t14);
    real t26 = _p_CLalpha->funcEval(t14);
    real t28 = t16 * t16;
    real t36 = 0.1e1 / __states_controls[3];
    real t39 = _model_params[MOD_PAR_INDEX_mu];
    real t43 = t11 + _model_params[MOD_PAR_INDEX_R];
    real t44 = t43 * t43;
    real t45 = 0.1e1 / t44;
    __values[1] = __state_control_derivatives[1] * t3 - (t15 * t17 - t21 * t23 * (t25 * t26 * t28 + t24) / 0.2e1) * t36 + t39 * t7 * t45;
    real t49 = sin(t16);
    real t57 = 0.1e1 / t5;
    real t59 = cos(t6);
    __values[2] = __state_control_derivatives[2] * t3 - (t15 * t49 + t21 * t23 * t26 * t16 / 0.2e1) * t36 * t57 - t59 * (t5 / t43 - t39 * t57 * t45);
    __values[3] = __state_control_derivatives[3] * t3 + t15 / _model_params[MOD_PAR_INDEX_g0] / _model_params[MOD_PAR_INDEX_Isp];

    return 0;
}

integer MinimumTimeToClimb::foEqnsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        real t1 = __states_controls[1];
    real t2 = __states_controls[0];
    real t3 = _p_SoundSpeed->funcEval(t2);
    real t4 = 0.1e1 / t3;
    real t5 = t1 * t4;
    real t6 = _p_Thrust->funcEval_D_1(t5, t2);
    real t8 = t3 * t3;
    real t9 = 0.1e1 / t8;
    real t10 = _p_SoundSpeed->funcEval_D_1(t2);
    real t11 = t9 * t10;
    real t13 = _p_Thrust->funcEval_D_2(t5, t2);
    real t14 = -t6 * t1 * t11 + t13;
    real t15 = __algebraic_states_controls[0];
    real t16 = cos(t15);
    real t18 = _p_Rho->funcEval_D_1(t2);
    real t19 = t1 * t1;
    real t20 = t18 * t19;
    real t22 = _model_params[MOD_PAR_INDEX_S];
    real t23 = _p_CD0->funcEval(t5);
    real t24 = _p_Eta->funcEval(t5);
    real t25 = _p_CLalpha->funcEval(t5);
    real t26 = t24 * t25;
    real t27 = t15 * t15;
    real t30 = t22 * (t26 * t27 + t23);
    real t33 = _p_Rho->funcEval(t2);
    real t34 = t33 * t19;
    real t35 = _p_CD0->funcEval_D_1(t5);
    real t38 = _p_Eta->funcEval_D_1(t5);
    real t44 = _p_CLalpha->funcEval_D_1(t5);
    real t45 = t24 * t44;
    real t54 = __states_controls[3];
    real t55 = 0.1e1 / t54;
    real t58 = _model_params[MOD_PAR_INDEX_mu];
    real t59 = __states_controls[2];
    real t60 = sin(t59);
    real t64 = t2 + _model_params[MOD_PAR_INDEX_R];
    real t65 = t64 * t64;
    real t67 = 0.1e1 / t65 / t64;
    __jac_xu[0] = -(t14 * t16 - t20 * t30 / 0.2e1 - t34 * t22 * (-t38 * t1 * t9 * t10 * t25 * t27 - t45 * t1 * t11 * t27 - t35 * t1 * t11) / 0.2e1) * t55 - 0.2e1 * t58 * t60 * t67;
    real t70 = sin(t15);
    real t72 = t22 * t25;
    real t73 = t72 * t15;
    real t86 = 0.1e1 / t1;
    real t88 = cos(t59);
    real t89 = 0.1e1 / t65;
    real t91 = t58 * t86;
    __jac_xu[1] = -(t14 * t70 + t20 * t73 / 0.2e1 - t33 * t19 * t1 * t22 * t44 * t9 * t10 * t15 / 0.2e1) * t55 * t86 - t88 * (-t1 * t89 + 0.2e1 * t91 * t67);
    real t98 = 0.1e1 / _model_params[MOD_PAR_INDEX_g0];
    real t102 = 0.1e1 / _model_params[MOD_PAR_INDEX_Isp];
    __jac_xu[2] = t14 * t98 * t102;
    __jac_xu[3] = -t60;
    real t103 = t6 * t4;
    real t105 = t33 * t1;
    __jac_xu[4] = -(t103 * t16 - t105 * t30 - t34 * t22 * (t38 * t4 * t25 * t27 + t45 * t4 * t27 + t35 * t4) / 0.2e1) * t55;
    real t121 = t34 * t22;
    real t129 = _p_Thrust->funcEval(t5, t2);
    real t130 = t129 * t70;
    real t133 = t130 + t34 * t73 / 0.2e1;
    real t135 = 0.1e1 / t19;
    real t137 = 0.1e1 / t64;
    __jac_xu[5] = -(t103 * t70 + t105 * t73 + t121 * t44 * t4 * t15 / 0.2e1) * t55 * t86 + t133 * t55 * t135 - t88 * (t58 * t135 * t89 + t137);
    __jac_xu[6] = t103 * t98 * t102;
    __jac_xu[7] = -t1 * t88;
    __jac_xu[8] = t58 * t88 * t89;
    __jac_xu[9] = t60 * (t1 * t137 - t91 * t89);
    real t148 = t129 * t16;
    real t152 = t54 * t54;
    real t153 = 0.1e1 / t152;
    __jac_xu[10] = (t148 - t34 * t30 / 0.2e1) * t153;
    __jac_xu[11] = t133 * t153 * t86;
    real t155 = __parameters[0];
    __jac_dxu[0] = 0.1e1 / t155;
    __jac_dxu[1] = __jac_dxu[0];
    __jac_dxu[2] = __jac_dxu[1];
    __jac_dxu[3] = __jac_dxu[2];
    __jac_axu[0] = -(-t121 * t26 * t15 - t130) * t55;
    __jac_axu[1] = -(t148 + t34 * t72 / 0.2e1) * t55 * t86;
    real t166 = t155 * t155;
    real t167 = 0.1e1 / t166;
    __jac_p[0] = -__state_control_derivatives[0] * t167;
    __jac_p[1] = -__state_control_derivatives[1] * t167;
    __jac_p[2] = -__state_control_derivatives[2] * t167;
    __jac_p[3] = -__state_control_derivatives[3] * t167;

    return 0;
}

integer MinimumTimeToClimb::foEqnsJacXuNnz ( integer const i_phase ) const {
    return 12;
return 0;
}
void MinimumTimeToClimb::foEqnsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 1;
    rows[1] = 2;
    rows[2] = 3;
    rows[3] = 0;
    rows[4] = 1;
    rows[5] = 2;
    rows[6] = 3;
    rows[7] = 0;
    rows[8] = 1;
    rows[9] = 2;
    rows[10] = 1;
    rows[11] = 2;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 0;
    cols[3] = 1;
    cols[4] = 1;
    cols[5] = 1;
    cols[6] = 1;
    cols[7] = 2;
    cols[8] = 2;
    cols[9] = 2;
    cols[10] = 3;
    cols[11] = 3;

}

integer MinimumTimeToClimb::foEqnsJacDxuNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void MinimumTimeToClimb::foEqnsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

}

integer MinimumTimeToClimb::foEqnsJacAxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void MinimumTimeToClimb::foEqnsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 1;
    rows[1] = 2;

        cols[0] = 0;
    cols[1] = 0;

}

integer MinimumTimeToClimb::foEqnsJacPNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void MinimumTimeToClimb::foEqnsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 0;
    cols[3] = 0;

}

integer MinimumTimeToClimb::foEqnsHess(integer const i_phase,
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
        real t1 = __lambda[3];
    real t2 = __states_controls[1];
    real t3 = __states_controls[0];
    real t4 = _p_SoundSpeed->funcEval(t3);
    real t5 = 0.1e1 / t4;
    real t6 = t2 * t5;
    real t7 = _p_Thrust->funcEval_D_1_1(t6, t3);
    real t9 = t4 * t4;
    real t10 = 0.1e1 / t9;
    real t11 = _p_SoundSpeed->funcEval_D_1(t3);
    real t12 = t10 * t11;
    real t14 = _p_Thrust->funcEval_D_1_2(t6, t3);
    real t15 = -t7 * t2 * t12 + t14;
    real t18 = _p_Thrust->funcEval_D_1(t6, t3);
    real t19 = t18 * t2;
    real t21 = 0.1e1 / t9 / t4;
    real t22 = t11 * t11;
    real t23 = t21 * t22;
    real t26 = _p_SoundSpeed->funcEval_D_1_1(t3);
    real t27 = t10 * t26;
    real t31 = _p_Thrust->funcEval_D_2_2(t6, t3);
    real t32 = -t14 * t2 * t12 - t15 * t2 * t12 + 0.2e1 * t19 * t23 - t19 * t27 + t31;
    real t36 = 0.1e1 / _model_params[MOD_PAR_INDEX_g0];
    real t39 = 0.1e1 / _model_params[MOD_PAR_INDEX_Isp];
    real t40 = t36 * t39;
    real t42 = __lambda[2];
    real t43 = __algebraic_states_controls[0];
    real t44 = sin(t43);
    real t46 = _p_Rho->funcEval_D_1_1(t3);
    real t47 = t2 * t2;
    real t48 = t46 * t47;
    real t50 = _model_params[MOD_PAR_INDEX_S];
    real t51 = _p_CLalpha->funcEval(t6);
    real t52 = t50 * t51;
    real t53 = t52 * t43;
    real t56 = _p_Rho->funcEval_D_1(t3);
    real t57 = t47 * t2;
    real t60 = _p_CLalpha->funcEval_D_1(t6);
    real t61 = t60 * t10;
    real t62 = t11 * t43;
    real t63 = t61 * t62;
    real t65 = _p_Rho->funcEval(t3);
    real t66 = t47 * t47;
    real t69 = _p_CLalpha->funcEval_D_1_1(t6);
    real t70 = t9 * t9;
    real t71 = 0.1e1 / t70;
    real t73 = t22 * t43;
    real t77 = t65 * t57;
    real t78 = t77 * t50;
    real t87 = __states_controls[3];
    real t88 = 0.1e1 / t87;
    real t90 = 0.1e1 / t2;
    real t92 = __states_controls[2];
    real t93 = cos(t92);
    real t96 = t3 + _model_params[MOD_PAR_INDEX_R];
    real t97 = t96 * t96;
    real t99 = 0.1e1 / t97 / t96;
    real t103 = _model_params[MOD_PAR_INDEX_mu];
    real t104 = t103 * t90;
    real t105 = t97 * t97;
    real t106 = 0.1e1 / t105;
    real t113 = __lambda[1];
    real t114 = cos(t43);
    real t116 = _p_CD0->funcEval(t6);
    real t117 = _p_Eta->funcEval(t6);
    real t118 = t117 * t51;
    real t119 = t43 * t43;
    real t121 = t118 * t119 + t116;
    real t122 = t50 * t121;
    real t125 = t56 * t47;
    real t126 = _p_CD0->funcEval_D_1(t6);
    real t127 = t126 * t2;
    real t129 = _p_Eta->funcEval_D_1(t6);
    real t130 = t129 * t2;
    real t131 = t130 * t10;
    real t133 = t11 * t51 * t119;
    real t135 = t117 * t60;
    real t136 = t135 * t2;
    real t137 = t12 * t119;
    real t140 = t50 * (-t127 * t12 - t131 * t133 - t136 * t137);
    real t142 = t65 * t47;
    real t143 = _p_CD0->funcEval_D_1_1(t6);
    real t145 = t71 * t22;
    real t150 = _p_Eta->funcEval_D_1_1(t6);
    real t154 = t22 * t51 * t119;
    real t156 = t130 * t21;
    real t168 = t117 * t69;
    real t183 = sin(t92);
    __hess_xu_xu[0] = t1 * t32 * t40 + t42 * (-(t32 * t44 + t48 * t53 / 0.2e1 - t56 * t57 * t50 * t63 + t65 * t66 * t50 * t69 * t71 * t73 / 0.2e1 + t78 * t60 * t21 * t73 - t78 * t61 * t26 * t43 / 0.2e1) * t88 * t90 - t93 * (-0.6e1 * t104 * t106 + 0.2e1 * t2 * t99)) + t113 * (-(t32 * t114 - t48 * t122 / 0.2e1 - t125 * t140 - t142 * t50 * (0.2e1 * t129 * t47 * t71 * t22 * t60 * t119 - t131 * t26 * t51 * t119 + t168 * t47 * t145 * t119 + t150 * t47 * t71 * t154 + 0.2e1 * t136 * t23 * t119 - t136 * t27 * t119 + t143 * t47 * t145 + 0.2e1 * t127 * t23 - t127 * t27 + 0.2e1 * t156 * t154) / 0.2e1) * t88 + 0.6e1 * t103 * t183 * t106);
    real t197 = t15 * t5;
    real t199 = t18 * t10;
    real t202 = t56 * t2;
    real t203 = t202 * t53;
    real t204 = t142 * t50;
    real t206 = 0.3e1 / 0.2e1 * t204 * t63;
    real t207 = t125 * t50;
    real t209 = t60 * t5 * t43;
    real t211 = t207 * t209 / 0.2e1;
    real t215 = t78 * t69 * t21 * t62 / 0.2e1;
    real t220 = _p_Thrust->funcEval_D_2(t6, t3);
    real t221 = -t19 * t12 + t220;
    real t222 = t221 * t44;
    real t227 = t222 + t125 * t53 / 0.2e1 - t78 * t63 / 0.2e1;
    real t229 = 0.1e1 / t47;
    real t230 = t227 * t88 * t229;
    real t231 = 0.1e1 / t97;
    real t232 = t103 * t229;
    real t236 = t93 * (-0.2e1 * t232 * t99 - t231);
    real t242 = t202 * t122;
    real t243 = t65 * t2;
    real t244 = t243 * t140;
    real t246 = t129 * t5;
    real t247 = t51 * t119;
    real t252 = t50 * (t135 * t5 * t119 + t126 * t5 + t246 * t247);
    real t254 = t125 * t252 / 0.2e1;
    real t256 = t2 * t11;
    real t263 = t129 * t10;
    real t276 = t142 * t50 * (-0.2e1 * t156 * t11 * t60 * t119 - t168 * t21 * t256 * t119 - t150 * t21 * t2 * t133 - t126 * t10 * t11 - t143 * t21 * t256 - t263 * t133 - t135 * t137) / 0.2e1;
    __hess_xu_xu[1] = t1 * t15 * t5 * t36 * t39 - t1 * t18 * t10 * t40 * t11 + t42 * (-(-t199 * t44 * t11 + t197 * t44 + t203 - t206 + t211 - t215) * t88 * t90 + t230 - t236) - t113 * (-t199 * t114 * t11 + t197 * t114 - t242 - t244 - t254 - t276) * t88;
    real t280 = t42 * t183;
    real t286 = t113 * t103;
    __hess_xu_xu[2] = t280 * (0.2e1 * t104 * t99 - t2 * t231) - 0.2e1 * t286 * t93 * t99;
    real t291 = t87 * t87;
    real t292 = 0.1e1 / t291;
    real t293 = t292 * t90;
    real t295 = t221 * t114;
    __hess_xu_xu[3] = t42 * t227 * t293 + t113 * (t295 - t125 * t122 / 0.2e1 - t142 * t140 / 0.2e1) * t292;
    real t307 = -t7 * t21 * t256 - t199 * t11 + t14 * t5;
    __hess_xu_xu[4] = t1 * t307 * t40 + t42 * (-(t307 * t44 + t203 - t206 + t211 - t215) * t88 * t90 + t230 - t236) - t113 * (t307 * t114 - t242 - t244 - t254 - t276) * t88;
    real t324 = t7 * t10;
    real t326 = t65 * t50;
    real t327 = t51 * t43;
    real t329 = t243 * t50;
    real t339 = t18 * t5;
    real t340 = t339 * t44;
    real t344 = t340 + t243 * t53 + t204 * t209 / 0.2e1;
    real t348 = _p_Thrust->funcEval(t6, t3);
    real t349 = t348 * t44;
    real t352 = t349 + t142 * t53 / 0.2e1;
    real t354 = 0.1e1 / t57;
    __hess_xu_xu[5] = t1 * t7 * t10 * t36 * t39 + t42 * (-(t324 * t44 + t326 * t327 + 0.2e1 * t329 * t209 + t204 * t69 * t10 * t43 / 0.2e1) * t88 * t90 + 0.2e1 * t344 * t88 * t229 - 0.2e1 * t352 * t88 * t354 + 0.2e1 * t93 * t103 * t354 * t231) - t113 * (t324 * t114 - t326 * t121 - 0.2e1 * t243 * t252 - t142 * t50 * (t168 * t10 * t119 + t150 * t10 * t247 + 0.2e1 * t263 * t60 * t119 + t143 * t10) / 0.2e1) * t88;
    real t382 = 0.1e1 / t96;
    real t386 = __lambda[0];
    __hess_xu_xu[6] = t280 * (t232 * t231 + t382) - t386 * t93;
    real t390 = t42 * t352;
    real t393 = t339 * t114;
    real t399 = t113 * (t393 - t243 * t122 - t142 * t252 / 0.2e1) * t292;
    __hess_xu_xu[7] = -t390 * t292 * t229 + t42 * t344 * t293 + t399;
    __hess_xu_xu[8] = __hess_xu_xu[2];
    __hess_xu_xu[9] = __hess_xu_xu[6];
    __hess_xu_xu[10] = t42 * t93 * (-t104 * t231 + t2 * t382) - t286 * t183 * t231 + t386 * t2 * t183;
    __hess_xu_xu[11] = __hess_xu_xu[3];
    __hess_xu_xu[12] = t42 * (-t352 * t292 * t229 + t344 * t292 * t90) + t399;
    real t416 = 0.1e1 / t291 / t87;
    real t419 = t348 * t114;
    __hess_xu_xu[13] = -0.2e1 * t390 * t416 * t90 - 0.2e1 * t113 * (t419 - t142 * t122 / 0.2e1) * t416;
    real t433 = t88 * t90;
    real t435 = t118 * t43;
    real t441 = t50 * t117;
    __hess_xu_axu[0] = -t42 * (t295 + t125 * t52 / 0.2e1 - t78 * t61 * t11 / 0.2e1) * t433 - t113 * (t77 * t50 * t129 * t12 * t327 + t77 * t441 * t63 - t207 * t435 - t222) * t88;
    real t458 = t42 * (t419 + t142 * t52 / 0.2e1);
    __hess_xu_axu[1] = -t42 * (t393 + t243 * t52 + t142 * t50 * t60 * t5 / 0.2e1) * t433 + t458 * t88 * t229 - t113 * (-t204 * t135 * t5 * t43 - t204 * t246 * t327 - 0.2e1 * t329 * t435 - t340) * t88;
    __hess_xu_axu[2] = t458 * t293 + t113 * (-t204 * t435 - t349) * t292;
    real t476 = __parameters[0];
    real t477 = t476 * t476;
    real t478 = 0.1e1 / t477;
    __hess_dxu_p[0] = -t386 * t478;
    __hess_dxu_p[1] = -t113 * t478;
    __hess_dxu_p[2] = -t42 * t478;
    __hess_dxu_p[3] = -t1 * t478;
    __hess_axu_axu[0] = t42 * t348 * t44 * t88 * t90 - t113 * (-t142 * t441 * t51 - t419) * t88;
    real t495 = 0.1e1 / t477 / t476;
    __hess_p_p[0] = 0.2e1 * t1 * __state_control_derivatives[3] * t495 + 0.2e1 * t113 * __state_control_derivatives[1] * t495 + 0.2e1 * t386 * __state_control_derivatives[0] * t495 + 0.2e1 * t42 * __state_control_derivatives[2] * t495;

    return 0;
}

integer MinimumTimeToClimb::foEqnsHessXuXuNnz ( integer const i_phase ) const {
    return 14;
return 0;
}
void MinimumTimeToClimb::foEqnsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[]) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;
    rows[4] = 0;
    rows[5] = 1;
    rows[6] = 2;
    rows[7] = 3;
    rows[8] = 0;
    rows[9] = 1;
    rows[10] = 2;
    rows[11] = 0;
    rows[12] = 1;
    rows[13] = 3;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 0;
    cols[3] = 0;
    cols[4] = 1;
    cols[5] = 1;
    cols[6] = 1;
    cols[7] = 1;
    cols[8] = 2;
    cols[9] = 2;
    cols[10] = 2;
    cols[11] = 3;
    cols[12] = 3;
    cols[13] = 3;

}

integer MinimumTimeToClimb::foEqnsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::foEqnsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::foEqnsHessXuAxuNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void MinimumTimeToClimb::foEqnsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 3;

}

integer MinimumTimeToClimb::foEqnsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::foEqnsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::foEqnsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::foEqnsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::foEqnsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::foEqnsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::foEqnsHessDxuPNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void MinimumTimeToClimb::foEqnsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;
    rows[3] = 0;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

}

integer MinimumTimeToClimb::foEqnsHessAxuAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void MinimumTimeToClimb::foEqnsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 0;

}

integer MinimumTimeToClimb::foEqnsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::foEqnsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::foEqnsHessPPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void MinimumTimeToClimb::foEqnsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
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

integer MinimumTimeToClimb::pathConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    
    return 0;
}

integer MinimumTimeToClimb::pathConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        

    return 0;
}

integer MinimumTimeToClimb::pathConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHess(integer const i_phase,
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
        

    return 0;
}
integer MinimumTimeToClimb::pathConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pathConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pathConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


// +----------------------------------------------------------------------------+
// |      _        _                             _             _       _        |
// |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                            |
// +----------------------------------------------------------------------------+


integer MinimumTimeToClimb::pointConstraints ( integer const i_phase,
                     real    const __states_controls[],
                     real    const __parameters[],
                     real          __zeta,
                     real          __values[] ) const {
    
    return 0;
}

integer MinimumTimeToClimb::pointConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_p[] ) const {
        

    return 0;
}

integer MinimumTimeToClimb::pointConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pointConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pointConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pointConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pointConstraintsHess (integer const i_phase,
                                 real    const __states_controls[],
                                 real    const __parameters[],
                                 real          __zeta,
                                 real    const __lambda[],
                                 real          __hess_xu_xu[],
                                 real          __hess_xu_p[],
                                 real          __hess_p_p[] ) const {
        

    return 0;
 }
integer MinimumTimeToClimb::pointConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pointConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pointConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pointConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::pointConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::pointConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +-----------------------------------------------------------------------------------------+
// |  _       _                       _                       _             _       _        |
// | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                  |___/                                                                  |
// +-----------------------------------------------------------------------------------------+

integer MinimumTimeToClimb::intConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    
    return 0;
}

integer MinimumTimeToClimb::intConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        

    return 0;
}

integer MinimumTimeToClimb::intConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHess(integer const i_phase,
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
        

    return 0;
}
integer MinimumTimeToClimb::intConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::intConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::intConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

//   +--------------------------------------------------------------------------------------------------+
//   |  _                           _                                        _ _ _   _                  |
//   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
//   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
//   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
//   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
//   |                                           |___/                                                  |
//   +--------------------------------------------------------------------------------------------------+

integer MinimumTimeToClimb::boundaryConditions ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real       __zeta_i,
               real       __zeta_f,
               real       __values[] ) const {
        __values[0] = __initial_state_control[0];
    __values[1] = __initial_state_control[1];
    __values[2] = __initial_state_control[2];
    __values[3] = __initial_state_control[3];
    __values[4] = __final_state_control[0];
    __values[5] = __final_state_control[1];
    __values[6] = __final_state_control[2];

    return 0;
}

integer MinimumTimeToClimb::boundaryConditionsJac ( integer const i_phase,
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
    __jac_xu_init[3] = 1;
    __jac_xu_fin[0] = 1;
    __jac_xu_fin[1] = 1;
    __jac_xu_fin[2] = 1;

    return 0;
}

integer MinimumTimeToClimb::boundaryConditionsJacXuInitNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsJacXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

}

integer MinimumTimeToClimb::boundaryConditionsJacXuFinNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsJacXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 4;
    rows[1] = 5;
    rows[2] = 6;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;

}

integer MinimumTimeToClimb::boundaryConditionsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::boundaryConditionsHess ( integer const i_phase,
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
        

    return 0;
}

integer MinimumTimeToClimb::boundaryConditionsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::boundaryConditionsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::boundaryConditionsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::boundaryConditionsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::boundaryConditionsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer MinimumTimeToClimb::boundaryConditionsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void MinimumTimeToClimb::boundaryConditionsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +--------------------------------------------------------------------------------+
// |                       _                         _             _       _        |
// |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                                |
// +--------------------------------------------------------------------------------+

integer MinimumTimeToClimb::eventConstraints ( integer const i_phase,
                      real const left_state_control[],
                      real const right_state_control[],
                      real const parameters[],
                      real const __zeta_l,
                      real const __zeta_r,
                      real       __values[] ) const {
    return 0;
}

integer MinimumTimeToClimb::eventConstraintsJac ( integer const i_phase,
                             real const left_state_control[],
                             real const right_state_control[],
                             real const parameters[],
                             real const __zeta_l,
                             real const __zeta_r,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
    return 0;
}

integer MinimumTimeToClimb::eventConstraintsJacXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsJacXuInitPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsJacXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsJacXuFinPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsJacPPattern ( integer const i_phase,
                               integer       rows[],
                               integer       cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsHess ( integer const i_phase,
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
    return 0;
}

integer MinimumTimeToClimb::eventConstraintsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsHessXuInitXuInitPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsHessXuInitXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsHessXuInitPPattern ( integer const i_phase,
                                     integer       rows[],
                                     integer       cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsHessXuFinXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsHessXuFinPPattern ( integer const i_phase,
                                  integer       rows[],
                                  integer       cols[] ) const {

}

integer MinimumTimeToClimb::eventConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
}
void MinimumTimeToClimb::eventConstraintsHessPPPattern ( integer const i_phase,
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

integer MinimumTimeToClimb::numberOfPostProcessing( integer const i_phase ) const { return (integer) _post_processing_names.at(i_phase).size(); }
integer MinimumTimeToClimb::numberOfDifferentialPostProcessing( integer const i_phase ) const { return (integer) _differential_post_processing_names.at(i_phase).size(); }
integer MinimumTimeToClimb::numberOfIntegralPostProcessing( integer const i_phase ) const { return (integer) _integral_post_processing_names.at(i_phase).size(); }
std::string MinimumTimeToClimb::postProcessingName( integer const i_phase, integer const i ) const { return _post_processing_names.at(i_phase).at(i); }
std::string MinimumTimeToClimb::differentialPostProcessingName( integer const i_phase, integer const i ) const { return _differential_post_processing_names.at(i_phase).at(i); }
std::string MinimumTimeToClimb::integralPostProcessingName( integer const i_phase, integer const i ) const { return _integral_post_processing_names.at(i_phase).at(i); }

void MinimumTimeToClimb::postProcessing(integer const i_phase,
                                                real    const __states_controls[],
                                                real    const __parameters[],
                                                real          __zeta,
                                                real          __values[] ) const {
        __values[0] = __parameters[0] * __zeta;
    __values[1] = _p_SoundSpeed->funcEval(__states_controls[0]);

}

void MinimumTimeToClimb::differentialPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
        real t1 = __states_controls[0];
    __values[0] = t1 + _model_params[MOD_PAR_INDEX_R];
    real t4 = __states_controls[1];
    real t5 = _p_SoundSpeed->funcEval(t1);
    __values[1] = t4 / t5;
    __values[2] = _p_CD0->funcEval(__values[1]);
    __values[3] = _p_CLalpha->funcEval(__values[1]);
    __values[4] = _p_Eta->funcEval(__values[1]);
    __values[5] = _p_Thrust->funcEval(__values[1], t1);
    real t8 = __algebraic_states_controls[0];
    real t9 = t8 * t8;
    __values[6] = __values[4] * __values[3] * t9 + __values[2];
    __values[7] = __values[3] * t8;
    real t11 = _p_Rho->funcEval(t1);
    real t12 = t4 * t4;
    real t13 = t11 * t12;
    __values[8] = t13 / 0.2e1;
    real t15 = _model_params[MOD_PAR_INDEX_S];
    __values[9] = t13 * t15 * __values[6] / 0.2e1;
    __values[10] = t13 * t15 * __values[3] * t8 / 0.2e1;

}

void MinimumTimeToClimb::integralPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
    
}
