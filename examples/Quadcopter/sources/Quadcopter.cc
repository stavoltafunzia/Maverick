#include "Quadcopter.hh"
#include <tgmath.h>

using namespace Maverick;
using namespace QuadcopterNamespace;
using namespace MaverickUtils;


Quadcopter::Quadcopter() {

    _num_p = 1;

    _dim_x = {10};
    _states_names = { {"x", "y", "z", "v_x", "v_y", "v_z", "phi", "mu", "psi", "thrust"} };

    _dim_ax = {0};
    _algebraic_states_names = { {} };

    _dim_u = {0};
    _controls_names = { {} };

    _dim_au = {4};
    _algebraic_controls_names = { {"phi_dot", "mu_dot", "psi_dot", "thrust_dot"} };

    _dim_p = {1};
    _parameters_names = { {"T"} };

    _dim_poc = {0};
    _point_constraints_names = { {} };

    _dim_pc = {1};
    _path_constraints_names = { {"MinimumHeight"} };

    _dim_ic = {0};
    _integral_constraints_names = { {} };

    _dim_bc = {18};
    _boundary_conditions_names = { {"x_i", "y_i", "z_i", "v_x_i", "v_y_i", "v_z_i", "psi_i", "phi_i", "mu_i", "x_f", "y_f", "z_f", "v_x_f", "v_y_f", "v_z_f", "psi_f", "phi_f", "mu_f"} };

    _dim_ec = {0};
    _event_constraints_names = { {} };

    _post_processing_names = {  {"vxg", "vyg", "vzg", "Dx", "Dy", "Dz", "ground_profile", "t"} };
    _differential_post_processing_names = {  {"omega_x", "omega_y", "omega_z"} };
    _integral_post_processing_names = {  {} };

    _model_params = new real[NUM_MODEL_PARAMS]; // deleted automatically by parent's destructor
    _model_params_names = {"x_min", "y_min", "z_min", "v_x_min", "v_y_min", "v_z_min", "phi_min", "mu_min", "psi_min", "thrust_min", "x_max", "y_max", "z_max", "v_x_max", "v_y_max", "v_z_max", "phi_max", "mu_max", "psi_max", "thrust_max", "phi_dot_min", "mu_dot_min", "psi_dot_min", "thrust_dot_min", "phi_dot_max", "mu_dot_max", "psi_dot_max", "thrust_dot_max", "T_min", "T_max", "x_i", "y_i", "z_i", "psi_i", "mu_i", "phi_i", "v_x_i", "v_y_i", "v_z_i", "x_f", "y_f", "z_f", "psi_f", "mu_f", "phi_f", "v_x_f", "v_y_f", "v_z_f", "time_guess", "thrust_guess", "wlt", "wlu", "wmt", "psi_dot_rel_imp", "g", "rho_aria", "m", "CxA", "CyA", "CzA", "V0", "l", "Id", "Iz", "thrust_0"};

}

void Quadcopter::derivedSetup(GC::GenericContainer const & gc) {
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
        throw std::runtime_error("Quadcopter: not all parameters have been set. Missing parameters are:\n" + missing_params.str());
    }

    GC::GenericContainer const * gc_mapped_objects = nullptr;
    try {
        gc_mapped_objects = &gc("MappedObjects");
    } catch (...) {
        throw std::runtime_error("Cannot find 'MappedObjects' inside 'Model' in the lua data file\n");
    }
    GC::GenericContainer const * gc_function = nullptr;

    _p_MinimumHeight = std::unique_ptr<GenericFunction2AInterface> (getGenericFunction2A("MinimumHeight"));
    try {
        gc_function = &( (*gc_mapped_objects)("MinimumHeight") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'MinimumHeight' in the lua data file\n");
    }
    _p_MinimumHeight->setup(*gc_function);

    _p_RegularizedAbsoluteValue = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RegularizedAbsoluteValue"));
    try {
        gc_function = &( (*gc_mapped_objects)("RegularizedAbsoluteValue") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RegularizedAbsoluteValue' in the lua data file\n");
    }
    _p_RegularizedAbsoluteValue->setup(*gc_function);

    _p_ThrustFactor = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("ThrustFactor"));
    try {
        gc_function = &( (*gc_mapped_objects)("ThrustFactor") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'ThrustFactor' in the lua data file\n");
    }
    _p_ThrustFactor->setup(*gc_function);


}

void Quadcopter::printDerivedInfo(std::ostream & out, InfoLevel info_level) const {
    if (info_level >= info_level_verbose) {
    out << "\n";
    _p_MinimumHeight->printInfo(out, info_level);
    out << "\n";
    _p_RegularizedAbsoluteValue->printInfo(out, info_level);
    out << "\n";
    _p_ThrustFactor->printInfo(out, info_level);
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

void Quadcopter::getStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_x_min];
    lower[1] = _model_params[MOD_PAR_INDEX_y_min];
    lower[2] = _model_params[MOD_PAR_INDEX_z_min];
    lower[3] = _model_params[MOD_PAR_INDEX_v_x_min];
    lower[4] = _model_params[MOD_PAR_INDEX_v_y_min];
    lower[5] = _model_params[MOD_PAR_INDEX_v_z_min];
    lower[6] = _model_params[MOD_PAR_INDEX_phi_min];
    lower[7] = _model_params[MOD_PAR_INDEX_mu_min];
    lower[8] = _model_params[MOD_PAR_INDEX_psi_min];
    lower[9] = _model_params[MOD_PAR_INDEX_thrust_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_x_max];
    upper[1] = _model_params[MOD_PAR_INDEX_y_max];
    upper[2] = _model_params[MOD_PAR_INDEX_z_max];
    upper[3] = _model_params[MOD_PAR_INDEX_v_x_max];
    upper[4] = _model_params[MOD_PAR_INDEX_v_y_max];
    upper[5] = _model_params[MOD_PAR_INDEX_v_z_max];
    upper[6] = _model_params[MOD_PAR_INDEX_phi_max];
    upper[7] = _model_params[MOD_PAR_INDEX_mu_max];
    upper[8] = _model_params[MOD_PAR_INDEX_psi_max];
    upper[9] = _model_params[MOD_PAR_INDEX_thrust_max];
    }

}

void Quadcopter::getAlgebraicStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = -_model_params[MOD_PAR_INDEX_phi_dot_max];
    lower[1] = -_model_params[MOD_PAR_INDEX_mu_dot_max];
    lower[2] = -_model_params[MOD_PAR_INDEX_psi_dot_max];
    lower[3] = -_model_params[MOD_PAR_INDEX_thrust_dot_max];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_phi_dot_max];
    upper[1] = _model_params[MOD_PAR_INDEX_mu_dot_max];
    upper[2] = _model_params[MOD_PAR_INDEX_psi_dot_max];
    upper[3] = _model_params[MOD_PAR_INDEX_thrust_dot_max];
    }

}

void Quadcopter::getParametersBounds(integer const i_phase,
                                                        real          lower[],
                                                        real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_T_min];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_T_max];
    }

}

void Quadcopter::getPathConstraintsBounds(integer const i_phase,
                                                             real    const __zeta,
                                                             real          lower[],
                                                             real          upper[] ) const {
    {
    lower[0] = 0;
    }

    {
    upper[0] = 0.1e6;
    }

}

void Quadcopter::getIntConstraintsBounds(integer const i_phase,
                                                            real    const __zeta_i,
                                                            real    const __zeta_f,
                                                            real          lower[],
                                                            real          upper[] ) const {
    
    
}

void Quadcopter::getBoundaryConditionsBounds(integer const i_phase,
                                                                real    const __zeta_i,
                                                                real    const __zeta_f,
                                                                real          lower[],
                                                                real          upper[] ) const {
    {
    lower[0] = _model_params[MOD_PAR_INDEX_x_i];
    lower[1] = _model_params[MOD_PAR_INDEX_y_i];
    lower[2] = _model_params[MOD_PAR_INDEX_z_i];
    lower[3] = _model_params[MOD_PAR_INDEX_v_x_i];
    lower[4] = _model_params[MOD_PAR_INDEX_v_y_i];
    lower[5] = _model_params[MOD_PAR_INDEX_v_z_i];
    lower[6] = _model_params[MOD_PAR_INDEX_psi_i];
    lower[7] = _model_params[MOD_PAR_INDEX_phi_i];
    lower[8] = _model_params[MOD_PAR_INDEX_mu_i];
    lower[9] = _model_params[MOD_PAR_INDEX_x_f];
    lower[10] = _model_params[MOD_PAR_INDEX_y_f];
    lower[11] = _model_params[MOD_PAR_INDEX_z_f];
    lower[12] = _model_params[MOD_PAR_INDEX_v_x_f];
    lower[13] = _model_params[MOD_PAR_INDEX_v_y_f];
    lower[14] = _model_params[MOD_PAR_INDEX_v_z_f];
    lower[15] = _model_params[MOD_PAR_INDEX_psi_f];
    lower[16] = _model_params[MOD_PAR_INDEX_phi_f];
    lower[17] = _model_params[MOD_PAR_INDEX_mu_f];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_x_i];
    upper[1] = _model_params[MOD_PAR_INDEX_y_i];
    upper[2] = _model_params[MOD_PAR_INDEX_z_i];
    upper[3] = _model_params[MOD_PAR_INDEX_v_x_i];
    upper[4] = _model_params[MOD_PAR_INDEX_v_y_i];
    upper[5] = _model_params[MOD_PAR_INDEX_v_z_i];
    upper[6] = _model_params[MOD_PAR_INDEX_psi_i];
    upper[7] = _model_params[MOD_PAR_INDEX_phi_i];
    upper[8] = _model_params[MOD_PAR_INDEX_mu_i];
    upper[9] = _model_params[MOD_PAR_INDEX_x_f];
    upper[10] = _model_params[MOD_PAR_INDEX_y_f];
    upper[11] = _model_params[MOD_PAR_INDEX_z_f];
    upper[12] = _model_params[MOD_PAR_INDEX_v_x_f];
    upper[13] = _model_params[MOD_PAR_INDEX_v_y_f];
    upper[14] = _model_params[MOD_PAR_INDEX_v_z_f];
    upper[15] = _model_params[MOD_PAR_INDEX_psi_f];
    upper[16] = _model_params[MOD_PAR_INDEX_phi_f];
    upper[17] = _model_params[MOD_PAR_INDEX_mu_f];
    }

}

void Quadcopter::getPointConstraintsBounds(integer const i_phase,
                                                              real    const __zeta,
                                                              real          lower[],
                                                              real          upper[] ) const {
    
    
}

void Quadcopter::getEventConstraintsBounds(integer const i_phase,
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


void Quadcopter::evalAtMesh(integer const i_phase,
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
    __states_controls[9] = _model_params[MOD_PAR_INDEX_thrust_guess];
    real t3 = _model_params[MOD_PAR_INDEX_x_i];
    real t6 = _model_params[MOD_PAR_INDEX_x_f] - t3;
    __states_controls[0] = t6 * __zeta + t3;
    real t9 = _model_params[MOD_PAR_INDEX_y_i];
    real t12 = _model_params[MOD_PAR_INDEX_y_f] - t9;
    __states_controls[1] = t12 * __zeta + t9;
    real t15 = _model_params[MOD_PAR_INDEX_z_i];
    real t18 = _model_params[MOD_PAR_INDEX_z_f] - t15;
    __states_controls[2] = t18 * __zeta + t15;
    real t22 = 0.1e1 / _model_params[MOD_PAR_INDEX_time_guess];
    __states_controls[3] = t6 * t22;
    __states_controls[4] = t12 * t22;
    __states_controls[5] = t18 * t22;

    }
    if (__algebraic_states_controls) {
    

    }
}

void Quadcopter::eval(integer const i_phase,
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
    __parameters[0] = _model_params[MOD_PAR_INDEX_time_guess];

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

void Quadcopter::mayer ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real     & __value ) const {
        __value = _model_params[MOD_PAR_INDEX_wmt] * __parameters[0];

}

void Quadcopter::mayerJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
        __jac_p[0] = _model_params[MOD_PAR_INDEX_wmt];

}

integer Quadcopter::mayerJacXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerJacXuInitPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer Quadcopter::mayerJacXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerJacXuFinPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer Quadcopter::mayerJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void Quadcopter::mayerJacPPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

void Quadcopter::mayerHess ( integer const i_phase,
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

integer Quadcopter::mayerHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::mayerHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::mayerHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::mayerHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::mayerHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::mayerHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::mayerHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


//   +--------------------------------------------+
//   |  _                                         |
//   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
//   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
//   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
//   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
//   |          |___/                 |___/       |
//   +--------------------------------------------+

 void Quadcopter::lagrange ( integer const i_phase,
                           real    const __states_controls[],
                           real    const __state_control_derivatives[],
                           real    const __algebraic_states_controls[],
                           real    const __parameters[],
                           real          __zeta,
                           real        & __value ) const {
       real t8 = pow(__algebraic_states_controls[0], 0.2e1);
    real t10 = pow(__algebraic_states_controls[1], 0.2e1);
    real t12 = pow(__algebraic_states_controls[2], 0.2e1);
    real t17 = pow(__algebraic_states_controls[3], 0.2e1);
    real t20 = pow(_model_params[MOD_PAR_INDEX_thrust_0], 0.2e1);
    __value = _model_params[MOD_PAR_INDEX_wlt] * __parameters[0] + _model_params[MOD_PAR_INDEX_wlu] * (t8 + t10 + t12 * _model_params[MOD_PAR_INDEX_psi_dot_rel_imp] + t17 / t20);

}

 void Quadcopter::lagrangeJac ( integer const i_phase,
                                real    const __states_controls[],
                                real    const __state_control_derivatives[],
                                real    const __algebraic_states_controls[],
                                real    const __parameters[],
                                real          __zeta,
                                real          __jac_xu[],
                                real          __jac_dxu[],
                                real          __jac_axu[],
                                real          __jac_p[] ) const {
        real t2 =  _model_params[MOD_PAR_INDEX_wlu];
    __jac_axu[0] = 2 * t2 * __algebraic_states_controls[0];
    __jac_axu[1] = 2 * t2 * __algebraic_states_controls[1];
    __jac_axu[2] = 2 * t2 * __algebraic_states_controls[2] * _model_params[MOD_PAR_INDEX_psi_dot_rel_imp];
    real t16 =  pow( _model_params[MOD_PAR_INDEX_thrust_0],  2);
    __jac_axu[3] = 2 * t2 * __algebraic_states_controls[3] / t16;
    __jac_p[0] = _model_params[MOD_PAR_INDEX_wlt];

}

integer Quadcopter::lagrangeJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::lagrangeJacXuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer Quadcopter::lagrangeJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::lagrangeJacDxuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer Quadcopter::lagrangeJacAxuNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void Quadcopter::lagrangeJacAxuPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

}

integer Quadcopter::lagrangeJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void Quadcopter::lagrangeJacPPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;

}

 void Quadcopter::lagrangeHess ( integer const i_phase,
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
         real t2 =  _model_params[MOD_PAR_INDEX_wlu];
    __hess_axu_axu[0] = 2 * t2 * __lambda_0;
    __hess_axu_axu[1] = __hess_axu_axu[0];
    __hess_axu_axu[2] = 2 * t2 * _model_params[MOD_PAR_INDEX_psi_dot_rel_imp] * __lambda_0;
    real t10 =  pow( _model_params[MOD_PAR_INDEX_thrust_0],  2);
    __hess_axu_axu[3] = 2 * t2 / t10 * __lambda_0;

}

integer Quadcopter::lagrangeHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void Quadcopter::lagrangeHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

integer Quadcopter::lagrangeHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::lagrangeHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer Quadcopter::lagrangeHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::lagrangeHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer Quadcopter::lagrangeHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void Quadcopter::lagrangeHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer Quadcopter::lagrangeHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void Quadcopter::lagrangeHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer Quadcopter::lagrangeHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void Quadcopter::lagrangeHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer Quadcopter::lagrangeHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void Quadcopter::lagrangeHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer Quadcopter::lagrangeHessAxuAxuNnz ( integer const i_phase ) const {
    return 4;
return 0;
 }
 void Quadcopter::lagrangeHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;

         cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

 }

 integer Quadcopter::lagrangeHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
 }
 void Quadcopter::lagrangeHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer Quadcopter::lagrangeHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void Quadcopter::lagrangeHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

//   +-------------------------------------------------------------+
//   |   __                                 _   _                  |
//   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
//   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
//   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
//   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
//   |                       |_|                                   |
//   +-------------------------------------------------------------+

void Quadcopter::foEqns ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t3 = 0.1e1 / __parameters[0];
    real t5 = __states_controls[5];
    real t6 = __states_controls[8];
    real t7 = sin(t6);
    real t8 = t5 * t7;
    real t9 = __states_controls[7];
    real t10 = cos(t9);
    real t11 = __states_controls[6];
    real t12 = sin(t11);
    real t13 = t10 * t12;
    real t15 = __states_controls[3];
    real t16 = t15 * t7;
    real t17 = sin(t9);
    real t18 = t12 * t17;
    real t20 = cos(t6);
    real t21 = t5 * t20;
    real t23 = t15 * t20;
    real t25 = __states_controls[4];
    real t27 = cos(t11);
    __values[0] = t25 * t7 * t27 - t23 * t10 - t8 * t13 + t16 * t18 - t21 * t17 + __state_control_derivatives[0] * t3;
    __values[1] = -t25 * t20 * t27 - t16 * t10 + t21 * t13 - t8 * t17 - t23 * t18 + __state_control_derivatives[1] * t3;
    __values[2] = -t5 * t10 * t27 + t15 * t27 * t17 - t25 * t12 + __state_control_derivatives[2] * t3;
    real t44 = t27 * t10;
    real t45 = __state_control_derivatives[8];
    real t46 = t45 * t3;
    real t49 = __state_control_derivatives[6] * t3;
    real t53 = _model_params[MOD_PAR_INDEX_m];
    real t54 = (t49 * t17 + t44 * t46) * t53;
    real t57 = __state_control_derivatives[7] * t3;
    real t61 = (t12 * t45 * t3 + t57) * t53;
    real t66 = t17 * t27;
    real t68 = _model_params[MOD_PAR_INDEX_g];
    real t69 = t68 * t53;
    real t72 = _model_params[MOD_PAR_INDEX_rho_aria];
    real t77 = _model_params[MOD_PAR_INDEX_V0];
    real t78 = 0.1e1 / t77;
    real t80 = _p_RegularizedAbsoluteValue->funcEval(t15 * t78);
    __values[3] = -t54 * t25 + t61 * t5 + t53 * __state_control_derivatives[3] * t3 - t66 * t69 + 0.5e0 * t72 * _model_params[MOD_PAR_INDEX_CxA] * t15 * t80 * t77;
    real t89 = (t49 * t10 - t46 * t66) * t53;
    real t100 = _p_RegularizedAbsoluteValue->funcEval(t25 * t78);
    __values[4] = t54 * t15 - t89 * t5 + t53 * __state_control_derivatives[4] * t3 + t12 * t68 * t53 + 0.5e0 * t72 * _model_params[MOD_PAR_INDEX_CyA] * t25 * t100 * t77;
    real t115 = _p_RegularizedAbsoluteValue->funcEval(t5 * t78);
    real t122 = _p_ThrustFactor->funcEval(__states_controls[2]);
    __values[5] = -t61 * t15 + t89 * t25 + t53 * __state_control_derivatives[5] * t3 + t44 * t69 + 0.5e0 * t72 * _model_params[MOD_PAR_INDEX_CzA] * t5 * t115 * t77 - __states_controls[9] * t122;
    __values[6] = __algebraic_states_controls[0] - t49;
    __values[7] = __algebraic_states_controls[1] - t57;
    __values[8] = __algebraic_states_controls[2] - t46;
    __values[9] = -__state_control_derivatives[9] * t3 + __algebraic_states_controls[3];

}

void Quadcopter::foEqnsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        real t2 = __states_controls[2];
    real t3 = _p_ThrustFactor->funcEval_D_1(t2);
    __jac_xu[0] = -__states_controls[9] * t3;
    real t5 = __states_controls[8];
    real t6 = sin(t5);
    real t7 = __states_controls[6];
    real t8 = sin(t7);
    real t10 = __states_controls[7];
    real t11 = sin(t10);
    real t13 = cos(t5);
    real t14 = cos(t10);
    real t15 = t13 * t14;
    __jac_xu[1] = t6 * t8 * t11 - t15;
    real t18 = t6 * t14;
    __jac_xu[2] = -t13 * t8 * t11 - t18;
    real t19 = cos(t7);
    __jac_xu[3] = t19 * t11;
    real t21 = _model_params[MOD_PAR_INDEX_rho_aria];
    real t24 = t21 * _model_params[MOD_PAR_INDEX_CxA];
    real t25 = __states_controls[3];
    real t27 = _model_params[MOD_PAR_INDEX_V0];
    real t28 = 0.1e1 / t27;
    real t29 = t25 * t28;
    real t30 = _p_RegularizedAbsoluteValue->funcEval(t29);
    real t34 = _p_RegularizedAbsoluteValue->funcEval_D_1(t29);
    __jac_xu[4] = 0.5e0 * t24 * t30 * t27 + 0.5e0 * t24 * t25 * t34;
    real t38 = t14 * t19;
    real t39 = __state_control_derivatives[8];
    real t40 = __parameters[0];
    real t41 = 0.1e1 / t40;
    real t42 = t39 * t41;
    real t44 = __state_control_derivatives[6];
    real t45 = t44 * t41;
    real t47 = t45 * t11 + t38 * t42;
    real t49 = _model_params[MOD_PAR_INDEX_m];
    __jac_xu[5] = t47 * t49;
    real t50 = __state_control_derivatives[7];
    real t52 = t8 * t39;
    real t53 = t52 * t41;
    real t55 = (t50 * t41 + t53) * t49;
    __jac_xu[6] = -t55;
    __jac_xu[7] = t6 * t19;
    __jac_xu[8] = -t13 * t19;
    __jac_xu[9] = -t8;
    __jac_xu[10] = -__jac_xu[5];
    real t59 = t21 * _model_params[MOD_PAR_INDEX_CyA];
    real t60 = __states_controls[4];
    real t61 = t60 * t28;
    real t62 = _p_RegularizedAbsoluteValue->funcEval(t61);
    real t66 = _p_RegularizedAbsoluteValue->funcEval_D_1(t61);
    __jac_xu[11] = 0.5e0 * t59 * t62 * t27 + 0.5e0 * t59 * t60 * t66;
    real t72 = t45 * t14 - t42 * __jac_xu[3];
    __jac_xu[12] = t72 * t49;
    __jac_xu[13] = -t13 * t11 - t18 * t8;
    __jac_xu[14] = -t6 * t11 + t15 * t8;
    __jac_xu[15] = -t38;
    __jac_xu[16] = t55;
    __jac_xu[17] = -__jac_xu[12];
    real t79 = t21 * _model_params[MOD_PAR_INDEX_CzA];
    real t80 = __states_controls[5];
    real t81 = t80 * t28;
    real t82 = _p_RegularizedAbsoluteValue->funcEval(t81);
    real t86 = _p_RegularizedAbsoluteValue->funcEval_D_1(t81);
    __jac_xu[18] = 0.5e0 * t79 * t82 * t27 + 0.5e0 * t79 * t80 * t86;
    real t90 = t80 * t6;
    real t92 = t25 * t6;
    real t94 = t60 * t6;
    __jac_xu[19] = -t90 * t38 - t94 * t8 + t92 * __jac_xu[3];
    real t96 = t80 * t13;
    real t98 = t25 * t13;
    real t100 = t60 * t13;
    __jac_xu[20] = t100 * t8 + t96 * t38 - t98 * __jac_xu[3];
    __jac_xu[21] = -t25 * t8 * t11 + t80 * t14 * t8 - t60 * t19;
    real t107 = t8 * t14;
    real t108 = t107 * t39;
    real t109 = t41 * t49;
    real t110 = t109 * t60;
    real t112 = t19 * t39;
    real t113 = t109 * t80;
    real t115 = t11 * t8;
    real t117 = _model_params[MOD_PAR_INDEX_g];
    real t118 = t117 * t49;
    __jac_xu[22] = t108 * t110 + t112 * t113 + t115 * t118;
    real t120 = t109 * t25;
    real t122 = t11 * t49;
    real t123 = t122 * t80;
    __jac_xu[23] = t19 * t117 * t49 - t108 * t120 - t53 * t123;
    real t128 = t122 * t60;
    __jac_xu[24] = -t107 * t118 - t112 * t120 + t53 * t128;
    __jac_xu[25] = t92 * t107 + t98 * t11 + t90 * t115 - t96 * t14;
    __jac_xu[26] = -t98 * t107 + t92 * t11 - t96 * t115 - t90 * t14;
    __jac_xu[27] = t80 * t11 * t19 + t25 * t19 * t14;
    real t143 = t72 * t49;
    __jac_xu[28] = -t38 * t118 - t143 * t60;
    real t148 = -t47 * t49;
    __jac_xu[29] = t143 * t25 - t148 * t80;
    __jac_xu[30] = -__jac_xu[3] * t118 + t148 * t60;
    __jac_xu[31] = t100 * t19 - t96 * t107 + t90 * t11 + t98 * t115 + t92 * t14;
    __jac_xu[32] = -t90 * t107 - t96 * t11 + t92 * t115 - t98 * t14 + t94 * t19;
    real t162 = _p_ThrustFactor->funcEval(t2);
    __jac_xu[33] = -t162;
    __jac_dxu[0] = t41;
    __jac_dxu[1] = __jac_dxu[0];
    __jac_dxu[2] = __jac_dxu[1];
    __jac_dxu[3] = t49 * __jac_dxu[2];
    __jac_dxu[4] = __jac_dxu[3];
    __jac_dxu[5] = __jac_dxu[4];
    real t163 = __jac_dxu[2] * t11;
    real t164 = t49 * t60;
    __jac_dxu[6] = -t163 * t164;
    real t166 = t49 * t25;
    real t168 = __jac_dxu[2] * t14;
    real t169 = t49 * t80;
    __jac_dxu[7] = t163 * t166 - t168 * t169;
    __jac_dxu[8] = t168 * t164;
    __jac_dxu[9] = -__jac_dxu[2];
    __jac_dxu[10] = t113;
    __jac_dxu[11] = -t120;
    __jac_dxu[12] = __jac_dxu[9];
    real t172 = t8 * __jac_dxu[2];
    __jac_dxu[13] = -t38 * t110 + t172 * t169;
    real t175 = __jac_dxu[2] * t19;
    __jac_dxu[14] = t38 * t120 + t175 * t123;
    __jac_dxu[15] = -t175 * t128 - t172 * t166;
    __jac_dxu[16] = __jac_dxu[12];
    __jac_dxu[17] = __jac_dxu[16];
    __jac_axu[0] = 1;
    __jac_axu[1] = 1;
    __jac_axu[2] = 1;
    __jac_axu[3] = 1;
    real t180 = t40 * t40;
    real t181 = 0.1e1 / t180;
    __jac_p[0] = -__state_control_derivatives[0] * t181;
    __jac_p[1] = -__state_control_derivatives[1] * t181;
    __jac_p[2] = -__state_control_derivatives[2] * t181;
    real t187 = t39 * t181;
    real t189 = t44 * t181;
    real t192 = (-t189 * t11 - t38 * t187) * t49;
    real t194 = t50 * t181;
    real t197 = (-t52 * t181 - t194) * t49;
    __jac_p[3] = -t49 * __state_control_derivatives[3] * t181 - t192 * t60 + t197 * t80;
    real t206 = (-t189 * t14 + t187 * __jac_xu[3]) * t49;
    __jac_p[4] = -t49 * __state_control_derivatives[4] * t181 + t192 * t25 - t206 * t80;
    __jac_p[5] = -t49 * __state_control_derivatives[5] * t181 - t197 * t25 + t206 * t60;
    __jac_p[6] = t44 * t181;
    __jac_p[7] = t194;
    __jac_p[8] = t187;
    __jac_p[9] = __state_control_derivatives[9] * t181;

}

integer Quadcopter::foEqnsJacXuNnz ( integer const i_phase ) const {
    return 34;
return 0;
}
void Quadcopter::foEqnsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 5;
    rows[1] = 0;
    rows[2] = 1;
    rows[3] = 2;
    rows[4] = 3;
    rows[5] = 4;
    rows[6] = 5;
    rows[7] = 0;
    rows[8] = 1;
    rows[9] = 2;
    rows[10] = 3;
    rows[11] = 4;
    rows[12] = 5;
    rows[13] = 0;
    rows[14] = 1;
    rows[15] = 2;
    rows[16] = 3;
    rows[17] = 4;
    rows[18] = 5;
    rows[19] = 0;
    rows[20] = 1;
    rows[21] = 2;
    rows[22] = 3;
    rows[23] = 4;
    rows[24] = 5;
    rows[25] = 0;
    rows[26] = 1;
    rows[27] = 2;
    rows[28] = 3;
    rows[29] = 4;
    rows[30] = 5;
    rows[31] = 0;
    rows[32] = 1;
    rows[33] = 5;

        cols[0] = 2;
    cols[1] = 3;
    cols[2] = 3;
    cols[3] = 3;
    cols[4] = 3;
    cols[5] = 3;
    cols[6] = 3;
    cols[7] = 4;
    cols[8] = 4;
    cols[9] = 4;
    cols[10] = 4;
    cols[11] = 4;
    cols[12] = 4;
    cols[13] = 5;
    cols[14] = 5;
    cols[15] = 5;
    cols[16] = 5;
    cols[17] = 5;
    cols[18] = 5;
    cols[19] = 6;
    cols[20] = 6;
    cols[21] = 6;
    cols[22] = 6;
    cols[23] = 6;
    cols[24] = 6;
    cols[25] = 7;
    cols[26] = 7;
    cols[27] = 7;
    cols[28] = 7;
    cols[29] = 7;
    cols[30] = 7;
    cols[31] = 8;
    cols[32] = 8;
    cols[33] = 9;

}

integer Quadcopter::foEqnsJacDxuNnz ( integer const i_phase ) const {
    return 18;
return 0;
}
void Quadcopter::foEqnsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;
    rows[4] = 4;
    rows[5] = 5;
    rows[6] = 3;
    rows[7] = 4;
    rows[8] = 5;
    rows[9] = 6;
    rows[10] = 3;
    rows[11] = 5;
    rows[12] = 7;
    rows[13] = 3;
    rows[14] = 4;
    rows[15] = 5;
    rows[16] = 8;
    rows[17] = 9;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;
    cols[4] = 4;
    cols[5] = 5;
    cols[6] = 6;
    cols[7] = 6;
    cols[8] = 6;
    cols[9] = 6;
    cols[10] = 7;
    cols[11] = 7;
    cols[12] = 7;
    cols[13] = 8;
    cols[14] = 8;
    cols[15] = 8;
    cols[16] = 8;
    cols[17] = 9;

}

integer Quadcopter::foEqnsJacAxuNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void Quadcopter::foEqnsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 6;
    rows[1] = 7;
    rows[2] = 8;
    rows[3] = 9;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

}

integer Quadcopter::foEqnsJacPNnz ( integer const i_phase ) const {
    return 10;
return 0;
}
void Quadcopter::foEqnsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;
    rows[4] = 4;
    rows[5] = 5;
    rows[6] = 6;
    rows[7] = 7;
    rows[8] = 8;
    rows[9] = 9;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 0;
    cols[3] = 0;
    cols[4] = 0;
    cols[5] = 0;
    cols[6] = 0;
    cols[7] = 0;
    cols[8] = 0;
    cols[9] = 0;

}

void Quadcopter::foEqnsHess(integer const i_phase,
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
        real t1 = __lambda[5];
    real t4 = __states_controls[2];
    real t5 = _p_ThrustFactor->funcEval_D_1_1(t4);
    __hess_xu_xu[0] = -t1 * __states_controls[9] * t5;
    real t7 = _p_ThrustFactor->funcEval_D_1(t4);
    __hess_xu_xu[1] = -t1 * t7;
    real t9 = __lambda[3];
    real t11 = _model_params[MOD_PAR_INDEX_rho_aria];
    real t14 = t11 * _model_params[MOD_PAR_INDEX_CxA];
    real t15 = __states_controls[3];
    real t18 = 0.1e1 / _model_params[MOD_PAR_INDEX_V0];
    real t19 = t15 * t18;
    real t20 = _p_RegularizedAbsoluteValue->funcEval_D_1(t19);
    real t23 = _p_RegularizedAbsoluteValue->funcEval_D_1_1(t19);
    __hess_xu_xu[2] = t9 * (0.10e1 * t14 * t20 + 0.5e0 * t14 * t15 * t23 * t18);
    real t29 = __states_controls[6];
    real t30 = cos(t29);
    real t31 = t1 * t30;
    real t32 = __state_control_derivatives[8];
    real t33 = __parameters[0];
    real t34 = 0.1e1 / t33;
    real t35 = t32 * t34;
    real t37 = _model_params[MOD_PAR_INDEX_m];
    real t38 = t35 * t37;
    real t40 = __lambda[4];
    real t41 = sin(t29);
    real t43 = __states_controls[7];
    real t44 = cos(t43);
    real t47 = __lambda[2];
    real t49 = sin(t43);
    real t51 = __lambda[1];
    real t52 = __states_controls[8];
    real t53 = cos(t52);
    real t54 = t51 * t53;
    real t55 = t30 * t49;
    real t57 = __lambda[0];
    real t58 = sin(t52);
    real t59 = t57 * t58;
    __hess_xu_xu[3] = -t40 * t41 * t44 * t38 - t47 * t41 * t49 - t31 * t38 - t54 * t55 + t59 * t55;
    real t62 = __state_control_derivatives[6];
    real t63 = t62 * t34;
    real t65 = -t35 * t55 + t63 * t44;
    real t68 = t47 * t44;
    real t70 = t53 * t44;
    real t73 = -t70 * t41 + t58 * t49;
    real t75 = t58 * t44;
    real t78 = t75 * t41 + t53 * t49;
    __hess_xu_xu[4] = t40 * t65 * t37 + t68 * t30 + t51 * t73 + t57 * t78;
    real t82 = t58 * t41 * t49 - t70;
    real t86 = t53 * t41 * t49 + t75;
    __hess_xu_xu[5] = t51 * t82 + t57 * t86;
    real t90 = t11 * _model_params[MOD_PAR_INDEX_CyA];
    real t91 = __states_controls[4];
    real t92 = t91 * t18;
    real t93 = _p_RegularizedAbsoluteValue->funcEval_D_1(t92);
    real t96 = _p_RegularizedAbsoluteValue->funcEval_D_1_1(t92);
    __hess_xu_xu[6] = t40 * (0.10e1 * t90 * t93 + 0.5e0 * t90 * t91 * t96 * t18);
    real t104 = t41 * t49;
    real t105 = t104 * t37;
    real t107 = t9 * t41;
    real t110 = t47 * t30;
    __hess_xu_xu[7] = t1 * t32 * t34 * t105 + t107 * t44 * t38 + t54 * t41 - t59 * t41 - t110;
    real t113 = t30 * t44;
    real t116 = -t113 * t35 - t63 * t49;
    __hess_xu_xu[8] = t1 * t116 * t37 - t9 * t65 * t37;
    __hess_xu_xu[9] = t51 * t58 * t30 + t57 * t53 * t30;
    real t127 = t11 * _model_params[MOD_PAR_INDEX_CzA];
    real t128 = __states_controls[5];
    real t129 = t128 * t18;
    real t130 = _p_RegularizedAbsoluteValue->funcEval_D_1(t129);
    real t133 = _p_RegularizedAbsoluteValue->funcEval_D_1_1(t129);
    __hess_xu_xu[10] = t1 * (0.10e1 * t127 * t130 + 0.5e0 * t127 * t128 * t133 * t18);
    real t142 = t9 * t30;
    __hess_xu_xu[11] = -t40 * t32 * t34 * t105 + t54 * t113 - t59 * t113 + t142 * t38 + t68 * t41;
    __hess_xu_xu[12] = -t40 * t116 * t37 + t110 * t49 - t51 * t86 + t57 * t82;
    __hess_xu_xu[13] = -t51 * t78 + t57 * t73;
    __hess_xu_xu[14] = __hess_xu_xu[3];
    __hess_xu_xu[15] = __hess_xu_xu[7];
    __hess_xu_xu[16] = __hess_xu_xu[11];
    real t156 = t41 * t32;
    real t157 = t34 * t37;
    real t158 = t157 * t15;
    real t160 = t35 * t30;
    real t161 = t49 * t37;
    real t162 = t161 * t91;
    real t165 = _model_params[MOD_PAR_INDEX_g];
    real t166 = t165 * t37;
    real t167 = t113 * t166;
    real t170 = t113 * t32;
    real t172 = t161 * t128;
    real t178 = t157 * t91;
    real t182 = t55 * t166;
    real t186 = t128 * t44 * t30;
    real t188 = t15 * t30 * t49;
    real t192 = t128 * t53;
    real t193 = t44 * t41;
    real t194 = t192 * t193;
    real t195 = t15 * t53;
    real t196 = t195 * t104;
    real t197 = t91 * t53;
    real t198 = t197 * t30;
    real t201 = t128 * t58;
    real t202 = t201 * t193;
    real t203 = t15 * t58;
    real t204 = t203 * t104;
    real t205 = t91 * t58;
    real t206 = t205 * t30;
    __hess_xu_xu[17] = t1 * (t156 * t158 + t160 * t162 - t167) + t40 * (-t41 * t165 * t37 - t170 * t158 - t160 * t172) + t9 * (-t156 * t157 * t128 + t170 * t178 + t182) + t47 * (t91 * t41 + t186 - t188) + t51 * (-t194 + t196 + t198) + t57 * (t202 - t204 - t206);
    real t209 = t193 * t32;
    real t216 = t35 * t41;
    real t217 = t44 * t37;
    real t218 = t217 * t128;
    __hess_xu_xu[18] = t1 * (t104 * t166 + t209 * t178) + t40 * (t104 * t32 * t158 - t216 * t218) + t9 * (-t216 * t162 + t193 * t166) + t47 * (-t128 * t49 * t41 - t15 * t41 * t44) + t51 * (-t195 * t113 - t192 * t55) + t57 * (t203 * t113 + t201 * t55);
    __hess_xu_xu[19] = t51 * (-t201 * t113 + t203 * t55 - t205 * t41) + t57 * (-t192 * t113 + t195 * t55 - t197 * t41);
    __hess_xu_xu[20] = __hess_xu_xu[4];
    __hess_xu_xu[21] = __hess_xu_xu[8];
    __hess_xu_xu[22] = __hess_xu_xu[12];
    __hess_xu_xu[23] = __hess_xu_xu[18];
    real t251 = -t65 * t37;
    real t255 = t116 * t37;
    real t265 = t201 * t49;
    real t266 = t203 * t44;
    real t269 = t192 * t49;
    real t270 = t195 * t44;
    __hess_xu_xu[24] = t1 * (t251 * t91 - t167) + t40 * (-t251 * t128 + t255 * t15) + t9 * (-t255 * t91 + t182) + t47 * (t186 - t188) + t51 * (-t194 + t196 + t265 + t266) + t57 * (t202 - t204 + t269 + t270);
    __hess_xu_xu[25] = t51 * (t201 * t104 - t192 * t44 + t203 * t193 + t195 * t49) + t57 * (t192 * t104 + t195 * t193 + t201 * t44 - t203 * t49);
    __hess_xu_xu[26] = __hess_xu_xu[5];
    __hess_xu_xu[27] = __hess_xu_xu[9];
    __hess_xu_xu[28] = __hess_xu_xu[13];
    __hess_xu_xu[29] = __hess_xu_xu[19];
    __hess_xu_xu[30] = __hess_xu_xu[25];
    __hess_xu_xu[31] = t51 * (-t194 + t196 + t265 + t266 + t198) + t57 * (t202 - t204 + t269 + t270 - t206);
    __hess_xu_xu[32] = __hess_xu_xu[1];
    real t289 = t40 * t34;
    __hess_xu_dxu[0] = t289 * t161;
    real t290 = t1 * t37;
    __hess_xu_dxu[1] = -t290 * t34;
    real t295 = t44 * t34;
    real t296 = t295 * t37;
    __hess_xu_dxu[2] = -t1 * t41 * t157 + t40 * t30 * t296;
    real t298 = t1 * t34;
    real t300 = t9 * t34;
    __hess_xu_dxu[3] = -t300 * t161 + t298 * t217;
    real t302 = t55 * t37;
    __hess_xu_dxu[4] = -t142 * t296 - t298 * t302;
    __hess_xu_dxu[5] = -t289 * t217;
    real t306 = t9 * t37;
    __hess_xu_dxu[6] = t306 * t34;
    __hess_xu_dxu[7] = t107 * t157 + t289 * t302;
    real t309 = t34 * t30;
    real t310 = t37 * t15;
    real t312 = t34 * t41;
    real t321 = t37 * t128;
    __hess_xu_dxu[8] = t1 * (t312 * t162 - t309 * t310) + t40 * (-t193 * t158 - t312 * t172) + t9 * (t193 * t178 + t309 * t321);
    real t331 = t217 * t91;
    __hess_xu_dxu[9] = -t298 * t162 + t40 * (t34 * t49 * t321 + t295 * t310) - t300 * t331;
    __hess_xu_dxu[10] = -t31 * t44 * t178 + t40 * (-t55 * t158 + t309 * t218) + t300 * t30 * t162;
    real t341 = __state_control_derivatives[7];
    real t342 = t33 * t33;
    real t343 = 0.1e1 / t342;
    real t345 = t156 * t343;
    real t346 = -t341 * t343 - t345;
    real t349 = t32 * t343;
    real t351 = t62 * t343;
    real t353 = -t113 * t349 - t351 * t49;
    __hess_xu_p[0] = -t1 * t346 * t37 + t40 * t353 * t37;
    real t358 = t349 * t55 - t351 * t44;
    __hess_xu_p[1] = t1 * t358 * t37 - t9 * t353 * t37;
    __hess_xu_p[2] = t9 * t346 * t37 - t40 * t358 * t37;
    real t367 = t30 * t32;
    real t368 = t343 * t37;
    real t369 = t368 * t15;
    real t378 = t368 * t91;
    __hess_xu_p[3] = t1 * (-t345 * t162 + t367 * t369) + t40 * (t345 * t172 + t209 * t369) + t9 * (-t367 * t368 * t128 - t209 * t378);
    real t384 = -t353;
    real t386 = t37 * t91;
    __hess_xu_p[4] = t1 * t384 * t386 + t40 * (-t384 * t37 * t128 + t358 * t37 * t15) - t9 * t358 * t386;
    __hess_dxu_p[0] = -t57 * t343;
    __hess_dxu_p[1] = -t51 * t343;
    __hess_dxu_p[2] = -t47 * t343;
    __hess_dxu_p[3] = -t306 * t343;
    __hess_dxu_p[4] = -t40 * t37 * t343;
    __hess_dxu_p[5] = -t290 * t343;
    real t403 = __lambda[6];
    real t405 = t1 * t343;
    real t413 = t9 * t343;
    __hess_dxu_p[6] = t403 * t343 - t405 * t331 + t40 * (-t343 * t49 * t310 + t343 * t44 * t321) + t413 * t162;
    real t415 = __lambda[7];
    __hess_dxu_p[7] = t405 * t310 - t413 * t321 + t415 * t343;
    real t419 = __lambda[8];
    real t421 = t41 * t343;
    real t423 = t343 * t30;
    __hess_dxu_p[8] = t419 * t343 + t1 * (t423 * t162 + t421 * t310) + t40 * (-t113 * t369 - t423 * t172) + t9 * (t113 * t378 - t421 * t321);
    real t435 = __lambda[9];
    __hess_dxu_p[9] = t435 * t343;
    real t439 = 0.1e1 / t342 / t33;
    real t455 = (0.2e1 * t156 * t439 + 0.2e1 * t341 * t439) * t37;
    real t457 = t32 * t439;
    real t459 = t62 * t439;
    real t463 = (0.2e1 * t459 * t44 - 0.2e1 * t457 * t55) * t37;
    real t475 = (0.2e1 * t113 * t457 + 0.2e1 * t459 * t49) * t37;
    __hess_p_p[0] = -0.2e1 * t435 * __state_control_derivatives[9] * t439 - 0.2e1 * t419 * t32 * t439 - 0.2e1 * t415 * t341 * t439 - 0.2e1 * t403 * t62 * t439 + t1 * (0.2e1 * t37 * __state_control_derivatives[5] * t439 - t455 * t15 + t463 * t91) + t40 * (0.2e1 * t37 * __state_control_derivatives[4] * t439 - t463 * t128 + t475 * t15) + t9 * (0.2e1 * t37 * __state_control_derivatives[3] * t439 + t455 * t128 - t475 * t91) + 0.2e1 * t47 * __state_control_derivatives[2] * t439 + 0.2e1 * t51 * __state_control_derivatives[1] * t439 + 0.2e1 * t57 * __state_control_derivatives[0] * t439;

}

integer Quadcopter::foEqnsHessXuXuNnz ( integer const i_phase ) const {
    return 33;
return 0;
}
void Quadcopter::foEqnsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[]) const {
        rows[0] = 2;
    rows[1] = 9;
    rows[2] = 3;
    rows[3] = 6;
    rows[4] = 7;
    rows[5] = 8;
    rows[6] = 4;
    rows[7] = 6;
    rows[8] = 7;
    rows[9] = 8;
    rows[10] = 5;
    rows[11] = 6;
    rows[12] = 7;
    rows[13] = 8;
    rows[14] = 3;
    rows[15] = 4;
    rows[16] = 5;
    rows[17] = 6;
    rows[18] = 7;
    rows[19] = 8;
    rows[20] = 3;
    rows[21] = 4;
    rows[22] = 5;
    rows[23] = 6;
    rows[24] = 7;
    rows[25] = 8;
    rows[26] = 3;
    rows[27] = 4;
    rows[28] = 5;
    rows[29] = 6;
    rows[30] = 7;
    rows[31] = 8;
    rows[32] = 2;

        cols[0] = 2;
    cols[1] = 2;
    cols[2] = 3;
    cols[3] = 3;
    cols[4] = 3;
    cols[5] = 3;
    cols[6] = 4;
    cols[7] = 4;
    cols[8] = 4;
    cols[9] = 4;
    cols[10] = 5;
    cols[11] = 5;
    cols[12] = 5;
    cols[13] = 5;
    cols[14] = 6;
    cols[15] = 6;
    cols[16] = 6;
    cols[17] = 6;
    cols[18] = 6;
    cols[19] = 6;
    cols[20] = 7;
    cols[21] = 7;
    cols[22] = 7;
    cols[23] = 7;
    cols[24] = 7;
    cols[25] = 7;
    cols[26] = 8;
    cols[27] = 8;
    cols[28] = 8;
    cols[29] = 8;
    cols[30] = 8;
    cols[31] = 8;
    cols[32] = 9;

}

integer Quadcopter::foEqnsHessXuDxuNnz ( integer const i_phase ) const {
    return 11;
return 0;
}
void Quadcopter::foEqnsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 6;
    rows[1] = 7;
    rows[2] = 8;
    rows[3] = 6;
    rows[4] = 8;
    rows[5] = 6;
    rows[6] = 7;
    rows[7] = 8;
    rows[8] = 8;
    rows[9] = 6;
    rows[10] = 8;

        cols[0] = 3;
    cols[1] = 3;
    cols[2] = 3;
    cols[3] = 4;
    cols[4] = 4;
    cols[5] = 5;
    cols[6] = 5;
    cols[7] = 5;
    cols[8] = 6;
    cols[9] = 7;
    cols[10] = 7;

}

integer Quadcopter::foEqnsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::foEqnsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::foEqnsHessXuPNnz ( integer const i_phase ) const {
    return 5;
return 0;
}
void Quadcopter::foEqnsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;
    rows[3] = 0;
    rows[4] = 0;

        cols[0] = 3;
    cols[1] = 4;
    cols[2] = 5;
    cols[3] = 6;
    cols[4] = 7;

}

integer Quadcopter::foEqnsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::foEqnsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::foEqnsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::foEqnsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::foEqnsHessDxuPNnz ( integer const i_phase ) const {
    return 10;
return 0;
}
void Quadcopter::foEqnsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;
    rows[3] = 0;
    rows[4] = 0;
    rows[5] = 0;
    rows[6] = 0;
    rows[7] = 0;
    rows[8] = 0;
    rows[9] = 0;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;
    cols[4] = 4;
    cols[5] = 5;
    cols[6] = 6;
    cols[7] = 7;
    cols[8] = 8;
    cols[9] = 9;

}

integer Quadcopter::foEqnsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::foEqnsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::foEqnsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::foEqnsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::foEqnsHessPPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void Quadcopter::foEqnsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
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

void Quadcopter::pathConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t4 = _p_MinimumHeight->funcEval(__states_controls[0], __states_controls[1]);
    __values[0] = __states_controls[2] - t4;

}

void Quadcopter::pathConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        real t1 = __states_controls[0];
    real t2 = __states_controls[1];
    real t3 = _p_MinimumHeight->funcEval_D_1(t1, t2);
    __jac_xu[0] = -t3;
    real t4 = _p_MinimumHeight->funcEval_D_2(t1, t2);
    __jac_xu[1] = -t4;
    __jac_xu[2] = 0.1e1;

}

integer Quadcopter::pathConstraintsJacXuNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void Quadcopter::pathConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;

}

integer Quadcopter::pathConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void Quadcopter::pathConstraintsHess(integer const i_phase,
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
    real t2 = __states_controls[0];
    real t3 = __states_controls[1];
    real t4 = _p_MinimumHeight->funcEval_D_1_1(t2, t3);
    __hess_xu_xu[0] = -t1 * t4;
    real t6 = _p_MinimumHeight->funcEval_D_1_2(t2, t3);
    __hess_xu_xu[1] = -t1 * t6;
    __hess_xu_xu[2] = __hess_xu_xu[1];
    real t8 = _p_MinimumHeight->funcEval_D_2_2(t2, t3);
    __hess_xu_xu[3] = -t1 * t8;

}
integer Quadcopter::pathConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void Quadcopter::pathConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 0;
    rows[3] = 1;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 1;
    cols[3] = 1;

}

integer Quadcopter::pathConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pathConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pathConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


// +----------------------------------------------------------------------------+
// |      _        _                             _             _       _        |
// |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                            |
// +----------------------------------------------------------------------------+


void Quadcopter::pointConstraints ( integer const i_phase,
                     real    const __states_controls[],
                     real    const __parameters[],
                     real          __zeta,
                     real          __values[] ) const {
    
}

void Quadcopter::pointConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_p[] ) const {
        

}

integer Quadcopter::pointConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pointConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pointConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pointConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void Quadcopter::pointConstraintsHess (integer const i_phase,
                                 real    const __states_controls[],
                                 real    const __parameters[],
                                 real          __zeta,
                                 real    const __lambda[],
                                 real          __hess_xu_xu[],
                                 real          __hess_xu_p[],
                                 real          __hess_p_p[] ) const {
        

 }
integer Quadcopter::pointConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pointConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pointConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pointConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::pointConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::pointConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +-----------------------------------------------------------------------------------------+
// |  _       _                       _                       _             _       _        |
// | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                  |___/                                                                  |
// +-----------------------------------------------------------------------------------------+

void Quadcopter::intConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    
}

void Quadcopter::intConstraintsJac (integer const i_phase,
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

integer Quadcopter::intConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void Quadcopter::intConstraintsHess(integer const i_phase,
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
integer Quadcopter::intConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::intConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::intConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

//   +--------------------------------------------------------------------------------------------------+
//   |  _                           _                                        _ _ _   _                  |
//   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
//   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
//   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
//   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
//   |                                           |___/                                                  |
//   +--------------------------------------------------------------------------------------------------+

void Quadcopter::boundaryConditions ( integer const i_phase,
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
    __values[4] = __initial_state_control[4];
    __values[5] = __initial_state_control[5];
    __values[6] = __initial_state_control[8];
    __values[7] = __initial_state_control[6];
    __values[8] = __initial_state_control[7];
    __values[9] = __final_state_control[0];
    __values[10] = __final_state_control[1];
    __values[11] = __final_state_control[2];
    __values[12] = __final_state_control[3];
    __values[13] = __final_state_control[4];
    __values[14] = __final_state_control[5];
    __values[15] = __final_state_control[8];
    __values[16] = __final_state_control[6];
    __values[17] = __final_state_control[7];

}

void Quadcopter::boundaryConditionsJac ( integer const i_phase,
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
    __jac_xu_init[4] = 1;
    __jac_xu_init[5] = 1;
    __jac_xu_init[6] = 1;
    __jac_xu_init[7] = 1;
    __jac_xu_init[8] = 1;
    __jac_xu_fin[0] = 1;
    __jac_xu_fin[1] = 1;
    __jac_xu_fin[2] = 1;
    __jac_xu_fin[3] = 1;
    __jac_xu_fin[4] = 1;
    __jac_xu_fin[5] = 1;
    __jac_xu_fin[6] = 1;
    __jac_xu_fin[7] = 1;
    __jac_xu_fin[8] = 1;

}

integer Quadcopter::boundaryConditionsJacXuInitNnz ( integer const i_phase ) const {
    return 9;
return 0;
}
void Quadcopter::boundaryConditionsJacXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;
    rows[4] = 4;
    rows[5] = 5;
    rows[6] = 7;
    rows[7] = 8;
    rows[8] = 6;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;
    cols[4] = 4;
    cols[5] = 5;
    cols[6] = 6;
    cols[7] = 7;
    cols[8] = 8;

}

integer Quadcopter::boundaryConditionsJacXuFinNnz ( integer const i_phase ) const {
    return 9;
return 0;
}
void Quadcopter::boundaryConditionsJacXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 9;
    rows[1] = 10;
    rows[2] = 11;
    rows[3] = 12;
    rows[4] = 13;
    rows[5] = 14;
    rows[6] = 16;
    rows[7] = 17;
    rows[8] = 15;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;
    cols[4] = 4;
    cols[5] = 5;
    cols[6] = 6;
    cols[7] = 7;
    cols[8] = 8;

}

integer Quadcopter::boundaryConditionsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::boundaryConditionsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void Quadcopter::boundaryConditionsHess ( integer const i_phase,
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

integer Quadcopter::boundaryConditionsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::boundaryConditionsHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::boundaryConditionsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::boundaryConditionsHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::boundaryConditionsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::boundaryConditionsHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::boundaryConditionsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::boundaryConditionsHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::boundaryConditionsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::boundaryConditionsHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer Quadcopter::boundaryConditionsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void Quadcopter::boundaryConditionsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +--------------------------------------------------------------------------------+
// |                       _                         _             _       _        |
// |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                                |
// +--------------------------------------------------------------------------------+

void Quadcopter::eventConstraints ( integer const i_phase,
                      real const left_state_control[],
                      real const right_state_control[],
                      real const parameters[],
                      real const __zeta_l,
                      real const __zeta_r,
                      real       __values[] ) const {
}

void Quadcopter::eventConstraintsJac ( integer const i_phase,
                             real const left_state_control[],
                             real const right_state_control[],
                             real const parameters[],
                             real const __zeta_l,
                             real const __zeta_r,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
}

integer Quadcopter::eventConstraintsJacXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsJacXuInitPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer Quadcopter::eventConstraintsJacXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsJacXuFinPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer Quadcopter::eventConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsJacPPattern ( integer const i_phase,
                               integer       rows[],
                               integer       cols[] ) const {

}

void Quadcopter::eventConstraintsHess ( integer const i_phase,
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

integer Quadcopter::eventConstraintsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsHessXuInitXuInitPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer Quadcopter::eventConstraintsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsHessXuInitXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer Quadcopter::eventConstraintsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsHessXuInitPPattern ( integer const i_phase,
                                     integer       rows[],
                                     integer       cols[] ) const {

}

integer Quadcopter::eventConstraintsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsHessXuFinXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer Quadcopter::eventConstraintsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsHessXuFinPPattern ( integer const i_phase,
                                  integer       rows[],
                                  integer       cols[] ) const {

}

integer Quadcopter::eventConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
}
void Quadcopter::eventConstraintsHessPPPattern ( integer const i_phase,
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

integer Quadcopter::numberOfPostProcessing( integer const i_phase ) const { return (integer) _post_processing_names.at(i_phase).size(); }
integer Quadcopter::numberOfDifferentialPostProcessing( integer const i_phase ) const { return (integer) _differential_post_processing_names.at(i_phase).size(); }
integer Quadcopter::numberOfIntegralPostProcessing( integer const i_phase ) const { return (integer) _integral_post_processing_names.at(i_phase).size(); }
std::string Quadcopter::postProcessingName( integer const i_phase, integer const i ) const { return _post_processing_names.at(i_phase).at(i); }
std::string Quadcopter::differentialPostProcessingName( integer const i_phase, integer const i ) const { return _differential_post_processing_names.at(i_phase).at(i); }
std::string Quadcopter::integralPostProcessingName( integer const i_phase, integer const i ) const { return _integral_post_processing_names.at(i_phase).at(i); }

void Quadcopter::postProcessing(integer const i_phase,
                                                real    const __states_controls[],
                                                real    const __parameters[],
                                                real          __zeta,
                                                real          __values[] ) const {
        real t1 = __states_controls[5];
    real t2 = __states_controls[8];
    real t3 = sin(t2);
    real t4 = t1 * t3;
    real t5 = __states_controls[7];
    real t6 = cos(t5);
    real t7 = __states_controls[6];
    real t8 = sin(t7);
    real t9 = t6 * t8;
    real t11 = __states_controls[3];
    real t12 = t11 * t3;
    real t13 = sin(t5);
    real t14 = t8 * t13;
    real t16 = cos(t2);
    real t17 = t1 * t16;
    real t19 = t11 * t16;
    real t21 = __states_controls[4];
    real t23 = cos(t7);
    __values[0] = -t21 * t3 * t23 - t12 * t14 + t17 * t13 + t19 * t6 + t4 * t9;
    __values[1] = t21 * t16 * t23 + t12 * t6 + t4 * t13 + t19 * t14 - t17 * t9;
    __values[2] = t1 * t6 * t23 - t11 * t23 * t13 + t21 * t8;
    real t37 = _model_params[MOD_PAR_INDEX_rho_aria];
    real t42 = _model_params[MOD_PAR_INDEX_V0];
    real t43 = 0.1e1 / t42;
    real t45 = _p_RegularizedAbsoluteValue->funcEval(t11 * t43);
    __values[3] = -0.5e0 * t37 * _model_params[MOD_PAR_INDEX_CxA] * t11 * t45 * t42;
    real t54 = _p_RegularizedAbsoluteValue->funcEval(t21 * t43);
    __values[4] = -0.5e0 * t37 * _model_params[MOD_PAR_INDEX_CyA] * t21 * t54 * t42;
    real t63 = _p_RegularizedAbsoluteValue->funcEval(t1 * t43);
    __values[5] = -0.5e0 * t37 * _model_params[MOD_PAR_INDEX_CzA] * t1 * t63 * t42;
    __values[6] = _p_MinimumHeight->funcEval(__states_controls[0], __states_controls[1]);
    __values[7] = __parameters[0] * __zeta;

}

void Quadcopter::differentialPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
        real t1 = __state_control_derivatives[8];
    real t3 = 0.1e1 / __parameters[0];
    real t4 = t1 * t3;
    real t5 = __states_controls[6];
    real t6 = cos(t5);
    real t7 = __states_controls[7];
    real t8 = sin(t7);
    real t12 = __state_control_derivatives[6] * t3;
    real t13 = cos(t7);
    __values[0] = -t4 * t6 * t8 + t12 * t13;
    real t17 = sin(t5);
    __values[1] = t17 * t1 * t3 + __state_control_derivatives[7] * t3;
    __values[2] = t6 * t13 * t4 + t12 * t8;

}

void Quadcopter::integralPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
    
}
