#include "SingleMassPointE.hh"
#include <tgmath.h>

using namespace Maverick;
using namespace SingleMassPointENamespace;
using namespace MaverickUtils;


SingleMassPointE::SingleMassPointE() {

    _num_p = 1;

    _dim_x = {4};
    _states_names = { {"n", "alpha", "V", "lambda"} };

    _dim_ax = {0};
    _algebraic_states_names = { {} };

    _dim_u = {0};
    _controls_names = { {} };

    _dim_au = {2};
    _algebraic_controls_names = { {"ux", "uy"} };

    _dim_p = {1};
    _parameters_names = { {"beta"} };

    _dim_poc = {0};
    _point_constraints_names = { {} };

    _dim_pc = {2};
    _path_constraints_names = { {"power_constraint", "AdherenceEllipse"} };

    _dim_ic = {1};
    _integral_constraints_names = { {"EnergyUsed"} };

    _dim_bc = {5};
    _boundary_conditions_names = { {"init_lat_pos", "init_orientation", "init_speed", "sideslip", "final_speed"} };

    _dim_ec = {0};
    _event_constraints_names = { {} };

    _post_processing_names = {  {"xR", "yR", "xRL", "yRL", "xRR", "yRR", "xV", "yV", "theta", "Fy", "Fy", "RoadLeftWidth", "RoadRightWidth"} };
    _differential_post_processing_names = {  {"ax", "ay", "PowerUsed", "Omega", "Fx", "PowerUsed"} };
    _integral_post_processing_names = {  {"t", "EnergyUsed"} };

    _model_params = new real[NUM_MODEL_PARAMS]; // deleted automatically by parent's destructor
    _model_params_names = {"g", "m", "DX", "DY", "Klambda", "Pmax", "Fx_max", "Omega_max", "V0", "road_width", "w_reg", "Vmax", "Pmin", "Vf", "Emin", "Emax"};

}

void SingleMassPointE::derivedSetup(GC::GenericContainer const & gc) {
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
        throw std::runtime_error("SingleMassPointE: not all parameters have been set. Missing parameters are:\n" + missing_params.str());
    }

    GC::GenericContainer const * gc_mapped_objects = nullptr;
    try {
        gc_mapped_objects = &gc("MappedObjects");
    } catch (...) {
        throw std::runtime_error("Cannot find 'MappedObjects' inside 'Model' in the lua data file\n");
    }
    GC::GenericContainer const * gc_function = nullptr;

    _p_RoadXR = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadXR"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadXR") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadXR' in the lua data file\n");
    }
    _p_RoadXR->setup(*gc_function);

    _p_RoadRightWidth = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadRightWidth"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadRightWidth") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadRightWidth' in the lua data file\n");
    }
    _p_RoadRightWidth->setup(*gc_function);

    _p_RegularizedPositive = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RegularizedPositive"));
    try {
        gc_function = &( (*gc_mapped_objects)("RegularizedPositive") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RegularizedPositive' in the lua data file\n");
    }
    _p_RegularizedPositive->setup(*gc_function);

    _p_RoadXL = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadXL"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadXL") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadXL' in the lua data file\n");
    }
    _p_RoadXL->setup(*gc_function);

    _p_RoadYL = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadYL"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadYL") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadYL' in the lua data file\n");
    }
    _p_RoadYL->setup(*gc_function);

    _p_RoadHeading = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadHeading"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadHeading") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadHeading' in the lua data file\n");
    }
    _p_RoadHeading->setup(*gc_function);

    _p_RoadY = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadY"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadY") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadY' in the lua data file\n");
    }
    _p_RoadY->setup(*gc_function);

    _p_RoadCurvature = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadCurvature"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadCurvature") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadCurvature' in the lua data file\n");
    }
    _p_RoadCurvature->setup(*gc_function);

    _p_RoadLeftWidth = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadLeftWidth"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadLeftWidth") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadLeftWidth' in the lua data file\n");
    }
    _p_RoadLeftWidth->setup(*gc_function);

    _p_RoadX = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadX"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadX") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadX' in the lua data file\n");
    }
    _p_RoadX->setup(*gc_function);

    _p_RoadYR = std::unique_ptr<GenericFunction1AInterface> (getGenericFunction1A("RoadYR"));
    try {
        gc_function = &( (*gc_mapped_objects)("RoadYR") );
    } catch (...) {
        throw std::runtime_error("Cannot find mapped object named 'RoadYR' in the lua data file\n");
    }
    _p_RoadYR->setup(*gc_function);


}

void SingleMassPointE::printDerivedInfo(std::ostream & out, InfoLevel info_level) const {
    if (info_level >= info_level_verbose) {
    out << "\n";
    _p_RoadXR->printInfo(out, info_level);
    out << "\n";
    _p_RoadRightWidth->printInfo(out, info_level);
    out << "\n";
    _p_RegularizedPositive->printInfo(out, info_level);
    out << "\n";
    _p_RoadXL->printInfo(out, info_level);
    out << "\n";
    _p_RoadYL->printInfo(out, info_level);
    out << "\n";
    _p_RoadHeading->printInfo(out, info_level);
    out << "\n";
    _p_RoadY->printInfo(out, info_level);
    out << "\n";
    _p_RoadCurvature->printInfo(out, info_level);
    out << "\n";
    _p_RoadLeftWidth->printInfo(out, info_level);
    out << "\n";
    _p_RoadX->printInfo(out, info_level);
    out << "\n";
    _p_RoadYR->printInfo(out, info_level);
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

void SingleMassPointE::getStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    real t1 = _p_RoadLeftWidth->funcEval(__zeta);
    lower[0] = -t1;
    lower[1] = -0.1e1;
    lower[2] = 0.0e0;
    lower[3] = -0.5e0;
    }

    {
    upper[0] = _p_RoadRightWidth->funcEval(__zeta);
    upper[1] = 0.1e1;
    upper[2] = _model_params[MOD_PAR_INDEX_Vmax];
    upper[3] = 0.105e2;
    }

}

void SingleMassPointE::getAlgebraicStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = -1;
    lower[1] = -1;
    }

    {
    upper[0] = 1;
    upper[1] = 1;
    }

}

void SingleMassPointE::getParametersBounds(integer const i_phase,
                                                        real          lower[],
                                                        real          upper[] ) const {
    {
    lower[0] = -0.5e0;
    }

    {
    upper[0] = 0.5e0;
    }

}

void SingleMassPointE::getPathConstraintsBounds(integer const i_phase,
                                                             real    const __zeta,
                                                             real          lower[],
                                                             real          upper[] ) const {
    {
    lower[0] = -_model_params[MOD_PAR_INDEX_Pmin];
    lower[1] = -1;
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_Pmax];
    upper[1] = 1;
    }

}

void SingleMassPointE::getIntConstraintsBounds(integer const i_phase,
                                                            real    const __zeta_i,
                                                            real    const __zeta_f,
                                                            real          lower[],
                                                            real          upper[] ) const {
    {
    lower[0] = -_model_params[MOD_PAR_INDEX_Emin];
    }

    {
    upper[0] = _model_params[MOD_PAR_INDEX_Emax];
    }

}

void SingleMassPointE::getBoundaryConditionsBounds(integer const i_phase,
                                                                real    const __zeta_i,
                                                                real    const __zeta_f,
                                                                real          lower[],
                                                                real          upper[] ) const {
    {
    lower[0] = 0;
    lower[1] = 0;
    lower[2] = _model_params[MOD_PAR_INDEX_V0];
    lower[3] = 0;
    lower[4] = _model_params[MOD_PAR_INDEX_Vf];
    }

    {
    upper[0] = 0;
    upper[1] = 0;
    upper[2] = _model_params[MOD_PAR_INDEX_V0];
    upper[3] = 0;
    upper[4] = _model_params[MOD_PAR_INDEX_Vf];
    }

}

void SingleMassPointE::getPointConstraintsBounds(integer const i_phase,
                                                              real    const __zeta,
                                                              real          lower[],
                                                              real          upper[] ) const {
    
    
}

void SingleMassPointE::getEventConstraintsBounds(integer const i_phase,
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


void SingleMassPointE::evalAtMesh(integer const i_phase,
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
    __states_controls[2] = _model_params[MOD_PAR_INDEX_V0];

    }
    if (__algebraic_states_controls) {
    

    }
}

void SingleMassPointE::eval(integer const i_phase,
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

void SingleMassPointE::mayer ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real     & __value ) const {
        __value = 0;

}

void SingleMassPointE::mayerJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
        

}

integer SingleMassPointE::mayerJacXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerJacXuInitPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer SingleMassPointE::mayerJacXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerJacXuFinPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer SingleMassPointE::mayerJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerJacPPattern ( integer const i_phase, integer cols[] ) const {
        

}

void SingleMassPointE::mayerHess ( integer const i_phase,
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

integer SingleMassPointE::mayerHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::mayerHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::mayerHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::mayerHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::mayerHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::mayerHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::mayerHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}


//   +--------------------------------------------+
//   |  _                                         |
//   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
//   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
//   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
//   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
//   |          |___/                 |___/       |
//   +--------------------------------------------+

 void SingleMassPointE::lagrange ( integer const i_phase,
                           real    const __states_controls[],
                           real    const __state_control_derivatives[],
                           real    const __algebraic_states_controls[],
                           real    const __parameters[],
                           real          __zeta,
                           real        & __value ) const {
       real t3 = __states_controls[3];
    real t4 = cos(t3);
    real t5 = __states_controls[1];
    real t6 = cos(t5);
    real t8 = sin(t3);
    real t9 = sin(t5);
    real t15 = _p_RoadCurvature->funcEval(__zeta);
    real t21 = pow(__algebraic_states_controls[0], 0.2e1);
    real t23 = pow(__algebraic_states_controls[1], 0.2e1);
    __value = -0.1e1 / __states_controls[2] / (t4 * t6 + t8 * t9) * (__states_controls[0] * t15 - 0.1e1) * (0.1e1 + _model_params[MOD_PAR_INDEX_w_reg] * (t21 + t23));

}

 void SingleMassPointE::lagrangeJac ( integer const i_phase,
                                real    const __states_controls[],
                                real    const __state_control_derivatives[],
                                real    const __algebraic_states_controls[],
                                real    const __parameters[],
                                real          __zeta,
                                real          __jac_xu[],
                                real          __jac_dxu[],
                                real          __jac_axu[],
                                real          __jac_p[] ) const {
        real t1 = __states_controls[2];
    real t2 = 0.1e1 / t1;
    real t3 = __states_controls[3];
    real t4 = cos(t3);
    real t5 = __states_controls[1];
    real t6 = cos(t5);
    real t8 = sin(t3);
    real t9 = sin(t5);
    real t11 = t4 * t6 + t8 * t9;
    real t12 = 0.1e1 / t11;
    real t13 = t2 * t12;
    real t14 = _p_RoadCurvature->funcEval(__zeta);
    real t16 =  _model_params[MOD_PAR_INDEX_w_reg];
    real t17 = __algebraic_states_controls[0];
    real t18 =  (t17 * t17);
    real t19 = __algebraic_states_controls[1];
    real t20 =  (t19 * t19);
    real t23 = 1 + t16 * (t18 + t20);
    __jac_xu[0] = -t13 * t14 *  t23;
    real t26 = t11 * t11;
    real t28 = t2 / t26;
    real t31 = __states_controls[0] * t14 - 0.1e1;
    real t32 = t31 *  t23;
    real t35 = -t4 * t9 + t8 * t6;
    __jac_xu[1] = t28 * t32 * t35;
    real t37 = t1 * t1;
    __jac_xu[2] = 0.1e1 / t37 * t12 * t32;
    __jac_xu[3] = -t28 * t32 * t35;
    real t42 = t31 *  t16;
    __jac_axu[0] = -0.2e1 * t13 * t42 * t17;
    __jac_axu[1] = -0.2e1 * t13 * t42 * t19;

}

integer SingleMassPointE::lagrangeJacXuNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void SingleMassPointE::lagrangeJacXuPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

}

integer SingleMassPointE::lagrangeJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::lagrangeJacDxuPattern ( integer const i_phase, integer cols[] ) const {
        

}

integer SingleMassPointE::lagrangeJacAxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void SingleMassPointE::lagrangeJacAxuPattern ( integer const i_phase, integer cols[] ) const {
        cols[0] = 0;
    cols[1] = 1;

}

integer SingleMassPointE::lagrangeJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::lagrangeJacPPattern ( integer const i_phase, integer cols[] ) const {
        

}

 void SingleMassPointE::lagrangeHess ( integer const i_phase,
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
         real t1 = __states_controls[2];
    real t2 = 0.1e1 / t1;
    real t3 = __states_controls[3];
    real t4 = cos(t3);
    real t5 = __states_controls[1];
    real t6 = cos(t5);
    real t8 = sin(t3);
    real t9 = sin(t5);
    real t11 = t4 * t6 + t8 * t9;
    real t12 = t11 * t11;
    real t13 = 0.1e1 / t12;
    real t14 = t2 * t13;
    real t15 = _p_RoadCurvature->funcEval(__zeta);
    real t16 = t14 * t15;
    real t18 =  _model_params[MOD_PAR_INDEX_w_reg];
    real t19 =  __algebraic_states_controls[0];
    real t20 =  ( t19 *  t19);
    real t21 =  __algebraic_states_controls[1];
    real t22 =  ( t21 *  t21);
    real t25 = 1 + t18 * (t20 + t22);
    real t26 = t25 * __lambda_0;
    real t29 = -t4 * t9 + t8 * t6;
    real t30 =  t26 * t29;
    __hess_xu_xu[0] = t16 * t30;
    real t31 = t1 * t1;
    real t32 = 0.1e1 / t31;
    real t33 = 0.1e1 / t11;
    real t34 = t32 * t33;
    __hess_xu_xu[1] = t34 * t15 *  t25 *  __lambda_0;
    real t37 = -t29;
    real t38 =  t26 * t37;
    __hess_xu_xu[2] = t16 * t38;
    __hess_xu_xu[3] = __hess_xu_xu[0];
    real t44 = __states_controls[0] * t15 - 0.1e1;
    real t45 = t2 / t12 / t11 * t44;
    real t46 = t29 * t29;
    real t50 = t14 * t44;
    real t53 = -t50 *  t26 * t11;
    __hess_xu_xu[4] = -0.2e1 * t45 *  t26 * t46 + t53;
    real t55 = t32 * t13 * t44;
    __hess_xu_xu[5] = -t55 * t30;
    real t61 = t2 * t33;
    real t63 = t44 *  t25 *  __lambda_0;
    __hess_xu_xu[6] = -0.2e1 * t45 *  t26 * t29 * t37 + t61 * t63;
    __hess_xu_xu[7] = __hess_xu_xu[1];
    __hess_xu_xu[8] = __hess_xu_xu[5];
    __hess_xu_xu[9] = -0.2e1 / t31 / t1 * t33 * t63;
    __hess_xu_xu[10] = -t55 * t38;
    __hess_xu_xu[11] = __hess_xu_xu[2];
    __hess_xu_xu[12] = __hess_xu_xu[6];
    __hess_xu_xu[13] = __hess_xu_xu[10];
    real t71 = t37 * t37;
    __hess_xu_xu[14] = -0.2e1 * t45 *  t26 * t71 + t53;
    real t75 = t61 * t15;
    real t76 = t18 * t19;
    real t77 = t76 * __lambda_0;
    __hess_xu_axu[0] = -0.2e1 * t75 *  t77;
    real t80 = t18 * t21;
    real t81 = t80 * __lambda_0;
    __hess_xu_axu[1] = -0.2e1 * t75 *  t81;
    real t84 =  __lambda_0 * t29;
    __hess_xu_axu[2] = 0.2e1 * t50 *  t76 * t84;
    __hess_xu_axu[3] = 0.2e1 * t50 *  t80 * t84;
    real t89 = t34 * t44;
    __hess_xu_axu[4] = 0.2e1 * t89 *  t77;
    __hess_xu_axu[5] = 0.2e1 * t89 *  t81;
    real t92 =  __lambda_0 * t37;
    __hess_xu_axu[6] = 0.2e1 * t50 *  t76 * t92;
    __hess_xu_axu[7] = 0.2e1 * t50 *  t80 * t92;
    __hess_axu_axu[0] = -0.2e1 * t61 * t44 *  t18 *  __lambda_0;
    __hess_axu_axu[1] = __hess_axu_axu[0];

}

integer SingleMassPointE::lagrangeHessXuXuNnz ( integer const i_phase ) const {
    return 15;
return 0;
}
 void SingleMassPointE::lagrangeHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
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
    rows[10] = 3;
    rows[11] = 0;
    rows[12] = 1;
    rows[13] = 2;
    rows[14] = 3;

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
    cols[10] = 2;
    cols[11] = 3;
    cols[12] = 3;
    cols[13] = 3;
    cols[14] = 3;

 }

integer SingleMassPointE::lagrangeHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::lagrangeHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     

     

}

integer SingleMassPointE::lagrangeHessXuAxuNnz ( integer const i_phase ) const {
    return 8;
return 0;
}
void SingleMassPointE::lagrangeHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     rows[0] = 0;
    rows[1] = 1;
    rows[2] = 0;
    rows[3] = 1;
    rows[4] = 0;
    rows[5] = 1;
    rows[6] = 0;
    rows[7] = 1;

     cols[0] = 0;
    cols[1] = 0;
    cols[2] = 1;
    cols[3] = 1;
    cols[4] = 2;
    cols[5] = 2;
    cols[6] = 3;
    cols[7] = 3;

}

integer SingleMassPointE::lagrangeHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void SingleMassPointE::lagrangeHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer SingleMassPointE::lagrangeHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void SingleMassPointE::lagrangeHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer SingleMassPointE::lagrangeHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void SingleMassPointE::lagrangeHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer SingleMassPointE::lagrangeHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void SingleMassPointE::lagrangeHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer SingleMassPointE::lagrangeHessAxuAxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
 }
 void SingleMassPointE::lagrangeHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         rows[0] = 0;
    rows[1] = 1;

         cols[0] = 0;
    cols[1] = 1;

 }

 integer SingleMassPointE::lagrangeHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
 }
 void SingleMassPointE::lagrangeHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

 integer SingleMassPointE::lagrangeHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
 void SingleMassPointE::lagrangeHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
         

         

 }

//   +-------------------------------------------------------------+
//   |   __                                 _   _                  |
//   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
//   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
//   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
//   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
//   |                       |_|                                   |
//   +-------------------------------------------------------------+

void SingleMassPointE::foEqns ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t2 = __states_controls[2];
    real t4 = __states_controls[3];
    real t5 = cos(t4);
    real t6 = __states_controls[1];
    real t7 = cos(t6);
    real t8 = t5 * t7;
    real t9 = sin(t4);
    real t10 = sin(t6);
    real t11 = t9 * t10;
    real t12 = t8 + t11;
    real t14 = _p_RoadCurvature->funcEval(__zeta);
    real t16 = __states_controls[0] * t14 - 0.1e1;
    real t17 = 0.1e1 / t16;
    real t18 = t12 * t17;
    __values[0] = -__state_control_derivatives[0] * t2 * t18 - t2 * (t5 * t10 - t9 * t7);
    real t33 = _model_params[MOD_PAR_INDEX_Omega_max];
    real t34 = __algebraic_states_controls[1];
    real t35 = t33 * t34;
    __values[1] = -__state_control_derivatives[1] * t2 * t18 - (t14 * t5 * t2 * t7 + t14 * t2 * t11 + t35 * t16) * t17;
    real t39 = t2 * t2;
    real t41 = t5 * t5;
    real t43 = -t10 * t41 + t8 * t9 + t10;
    real t45 = __state_control_derivatives[3];
    real t48 = __state_control_derivatives[2];
    real t58 = _model_params[MOD_PAR_INDEX_g];
    __values[2] = (t39 * t43 * t45 - t5 * t2 * t12 * t48 - t16 * (-t35 * t2 * t9 + __algebraic_states_controls[0] * _model_params[MOD_PAR_INDEX_Fx_max] * t58)) * t17;
    __values[3] = (t2 * t43 * t48 + t5 * t39 * t12 * t45 - t16 * (-t5 * t34 * t33 * t2 + _model_params[MOD_PAR_INDEX_Klambda] * t4 * t58)) * t17;

}

void SingleMassPointE::foEqnsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        real t1 = __state_control_derivatives[0];
    real t2 = __states_controls[2];
    real t3 = t1 * t2;
    real t4 = __states_controls[3];
    real t5 = cos(t4);
    real t6 = __states_controls[1];
    real t7 = cos(t6);
    real t8 = t5 * t7;
    real t9 = sin(t4);
    real t10 = sin(t6);
    real t11 = t9 * t10;
    real t12 = t8 + t11;
    real t14 = _p_RoadCurvature->funcEval(__zeta);
    real t16 = __states_controls[0] * t14 - 0.1e1;
    real t17 = t16 * t16;
    real t18 = 0.1e1 / t17;
    real t20 = t12 * t18 * t14;
    __jac_xu[0] = t3 * t20;
    real t21 = __state_control_derivatives[1];
    real t22 = t21 * t2;
    real t25 = _model_params[MOD_PAR_INDEX_Omega_max];
    real t26 = __algebraic_states_controls[1];
    real t27 = t25 * t26;
    real t28 = 0.1e1 / t16;
    real t31 = t14 * t5;
    real t34 = t14 * t2;
    __jac_xu[1] = t22 * t20 - t27 * t14 * t28 + (t31 * t2 * t7 + t34 * t11 + t27 * t16) * t18 * t14;
    real t40 = t2 * t9;
    real t41 = t27 * t40;
    real t44 = _model_params[MOD_PAR_INDEX_Fx_max];
    real t47 = _model_params[MOD_PAR_INDEX_g];
    real t49 = __algebraic_states_controls[0] * t44 * t47 - t41;
    real t52 = t2 * t2;
    real t54 = t5 * t5;
    real t56 = -t10 * t54 + t8 * t9 + t10;
    real t57 = t52 * t56;
    real t58 = __state_control_derivatives[3];
    real t60 = t5 * t2;
    real t61 = __state_control_derivatives[2];
    real t62 = t12 * t61;
    __jac_xu[2] = -t14 * t49 * t28 - (-t16 * t49 + t57 * t58 - t60 * t62) * t18 * t14;
    real t69 = t25 * t2;
    real t72 = _model_params[MOD_PAR_INDEX_Klambda];
    real t75 = -t5 * t26 * t69 + t72 * t4 * t47;
    real t78 = t2 * t56;
    real t80 = t5 * t52;
    real t81 = t12 * t58;
    __jac_xu[3] = -t14 * t75 * t28 - (-t16 * t75 + t78 * t61 + t80 * t81) * t18 * t14;
    real t87 = t5 * t10;
    real t88 = t9 * t7;
    real t89 = -t87 + t88;
    real t90 = t89 * t28;
    real t92 = t2 * t12;
    __jac_xu[4] = -t3 * t90 - t92;
    real t97 = -t31 * t2 * t10 + t34 * t88;
    __jac_xu[5] = -t22 * t90 - t97 * t28;
    real t99 = t87 * t9;
    real t100 = t7 * t54;
    real t101 = -t99 - t100 + t7;
    __jac_xu[6] = (t52 * t101 * t58 - t60 * t89 * t61) * t28;
    __jac_xu[7] = (t2 * t101 * t61 + t80 * t89 * t58) * t28;
    __jac_xu[8] = -t1 * t12 * t28 - t87 + t88;
    __jac_xu[9] = -t21 * t12 * t28 - (t14 * t9 * t10 + t31 * t7) * t28;
    real t125 = t16 * t26;
    __jac_xu[10] = (-t5 * t12 * t61 + t125 * t25 * t9 + 0.2e1 * t78 * t58) * t28;
    __jac_xu[11] = (t16 * t5 * t27 + t56 * t61 + 0.2e1 * t60 * t81) * t28;
    real t135 = -t89;
    real t136 = t135 * t28;
    __jac_xu[12] = t2 * t12 - t3 * t136;
    __jac_xu[13] = -t22 * t136 + t97 * t28;
    real t143 = t9 * t9;
    real t146 = -t143 * t7 + t100 + 0.2e1 * t99;
    real t153 = t5 * t25 * t2;
    __jac_xu[14] = (-t60 * t135 * t61 + t52 * t146 * t58 + t125 * t153 + t40 * t62) * t28;
    __jac_xu[15] = (t2 * t146 * t61 - t9 * t52 * t81 + t80 * t135 * t58 - t16 * (t72 * t47 + t41)) * t28;
    __jac_dxu[0] = -t92 * t28;
    __jac_dxu[1] = __jac_dxu[0];
    real t167 = t12 * t28;
    __jac_dxu[2] = -t60 * t167;
    __jac_dxu[3] = t78 * t28;
    __jac_dxu[4] = t57 * t28;
    __jac_dxu[5] = t80 * t167;
    __jac_axu[0] = -t44 * t47;
    __jac_axu[1] = -t25;
    __jac_axu[2] = t69 * t9;
    __jac_axu[3] = t153;

}

integer SingleMassPointE::foEqnsJacXuNnz ( integer const i_phase ) const {
    return 16;
return 0;
}
void SingleMassPointE::foEqnsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
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
    rows[11] = 3;
    rows[12] = 0;
    rows[13] = 1;
    rows[14] = 2;
    rows[15] = 3;

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
    cols[11] = 2;
    cols[12] = 3;
    cols[13] = 3;
    cols[14] = 3;
    cols[15] = 3;

}

integer SingleMassPointE::foEqnsJacDxuNnz ( integer const i_phase ) const {
    return 6;
return 0;
}
void SingleMassPointE::foEqnsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;
    rows[4] = 2;
    rows[5] = 3;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 2;
    cols[4] = 3;
    cols[5] = 3;

}

integer SingleMassPointE::foEqnsJacAxuNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void SingleMassPointE::foEqnsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 2;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 1;
    cols[3] = 1;

}

integer SingleMassPointE::foEqnsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void SingleMassPointE::foEqnsHess(integer const i_phase,
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
    real t2 = _p_RoadCurvature->funcEval(__zeta);
    real t3 = t2 * t2;
    real t5 = __states_controls[3];
    real t6 = cos(t5);
    real t7 = __algebraic_states_controls[1];
    real t10 = _model_params[MOD_PAR_INDEX_Omega_max];
    real t11 = __states_controls[2];
    real t12 = t10 * t11;
    real t15 = _model_params[MOD_PAR_INDEX_Klambda];
    real t18 = _model_params[MOD_PAR_INDEX_g];
    real t20 = -t6 * t7 * t12 + t15 * t5 * t18;
    real t23 = __states_controls[0] * t2 - 0.1e1;
    real t24 = t23 * t23;
    real t25 = 0.1e1 / t24;
    real t29 = __states_controls[1];
    real t30 = cos(t29);
    real t31 = t6 * t30;
    real t32 = sin(t5);
    real t33 = t31 * t32;
    real t34 = sin(t29);
    real t35 = t6 * t6;
    real t36 = t34 * t35;
    real t37 = t33 - t36 + t34;
    real t38 = t11 * t37;
    real t39 = __state_control_derivatives[2];
    real t41 = t11 * t11;
    real t42 = t6 * t41;
    real t43 = t32 * t34;
    real t44 = t31 + t43;
    real t45 = __state_control_derivatives[3];
    real t46 = t44 * t45;
    real t47 = t42 * t46;
    real t52 = 0.1e1 / t24 / t23;
    real t53 = t52 * t3;
    real t56 = __lambda[2];
    real t58 = t7 * t10;
    real t59 = t11 * t32;
    real t60 = t58 * t59;
    real t66 = __algebraic_states_controls[0] * _model_params[MOD_PAR_INDEX_Fx_max] * t18 - t60;
    real t72 = t6 * t11;
    real t73 = t44 * t39;
    real t74 = t72 * t73;
    real t80 = __lambda[1];
    real t81 = __state_control_derivatives[1];
    real t82 = t81 * t11;
    real t84 = t44 * t52 * t3;
    real t88 = t2 * t6;
    real t90 = t88 * t11 * t30;
    real t91 = t2 * t11;
    real t92 = t91 * t43;
    real t100 = __lambda[0];
    real t101 = __state_control_derivatives[0];
    real t102 = t100 * t101;
    real t103 = t102 * t11;
    __hess_xu_xu[0] = 0.2e1 * t1 * t3 * t20 * t25 + 0.2e1 * t1 * (-t23 * t20 + t38 * t39 + t47) * t53 + 0.2e1 * t56 * t3 * t66 * t25 + 0.2e1 * t56 * (t41 * t37 * t45 - t23 * t66 - t74) * t53 + t80 * (-0.2e1 * t82 * t84 + 0.2e1 * t58 * t3 * t25 - 0.2e1 * (t58 * t23 + t90 + t92) * t52 * t3) - 0.2e1 * t103 * t84;
    real t106 = t6 * t34;
    real t107 = t106 * t32;
    real t108 = t30 * t35;
    real t109 = -t107 - t108 + t30;
    real t110 = t11 * t109;
    real t112 = t32 * t30;
    real t113 = -t106 + t112;
    real t114 = t113 * t45;
    real t118 = t25 * t2;
    real t122 = t113 * t39;
    real t128 = t113 * t25 * t2;
    real t133 = -t88 * t11 * t34 + t91 * t112;
    __hess_xu_xu[1] = -t1 * (t110 * t39 + t42 * t114) * t118 - t56 * (t41 * t109 * t45 - t72 * t122) * t118 + t80 * (t133 * t25 * t2 + t82 * t128) + t103 * t128;
    real t139 = t1 * t2;
    real t141 = 0.1e1 / t23;
    real t148 = t23 * t6 * t58;
    real t153 = t56 * t2 * t7;
    real t154 = t10 * t32;
    real t161 = t23 * t7;
    real t162 = t161 * t154;
    real t169 = t2 * t32;
    real t177 = t44 * t25 * t2;
    __hess_xu_xu[2] = t139 * t6 * t58 * t141 - t1 * (t37 * t39 + 0.2e1 * t72 * t46 + t148) * t118 + t153 * t154 * t141 - t56 * (-t6 * t44 * t39 + 0.2e1 * t38 * t45 + t162) * t118 + t80 * (t81 * t44 * t118 + (t169 * t34 + t88 * t30) * t25 * t2) + t102 * t177;
    real t180 = t15 * t18 + t60;
    real t183 = t32 * t32;
    real t186 = -t183 * t30 + 0.2e1 * t107 + t108;
    real t187 = t11 * t186;
    real t189 = t32 * t41;
    real t191 = -t113;
    real t192 = t191 * t45;
    real t204 = t191 * t39;
    real t207 = t161 * t12 * t6;
    real t212 = t191 * t25 * t2;
    __hess_xu_xu[3] = -t139 * t180 * t141 - t1 * (-t23 * t180 + t187 * t39 - t189 * t46 + t42 * t192) * t118 + t153 * t12 * t6 * t141 - t56 * (t41 * t186 * t45 - t72 * t204 + t59 * t73 + t207) * t118 + t80 * (-t133 * t25 * t2 + t82 * t212) + t103 * t212;
    __hess_xu_xu[4] = __hess_xu_xu[1];
    real t220 = -t37;
    real t223 = -t44;
    real t225 = t42 * t223 * t45;
    real t232 = t72 * t223 * t39;
    real t236 = t223 * t141;
    real t238 = -t90 - t92;
    real t241 = t80 * (-t238 * t141 - t82 * t236);
    real t242 = t101 * t11;
    real t244 = t11 * t113;
    real t246 = t100 * (-t242 * t236 - t244);
    __hess_xu_xu[5] = t1 * (t11 * t220 * t39 + t225) * t141 + t56 * (t41 * t220 * t45 - t232) * t141 + t241 + t246;
    real t264 = t169 * t30 - t88 * t34;
    __hess_xu_xu[6] = t1 * (t109 * t39 + 0.2e1 * t72 * t114) * t141 + t56 * (-t6 * t113 * t39 + 0.2e1 * t110 * t45) * t141 + t80 * (-t81 * t113 * t141 - t264 * t141) + t100 * (-t101 * t113 * t141 - t31 - t43);
    real t272 = t34 * t183;
    real t274 = t272 - t36 + 0.2e1 * t33;
    real t287 = t44 * t141;
    real t294 = t11 * t191;
    __hess_xu_xu[7] = t1 * (t11 * t274 * t39 - t189 * t114 + t47) * t141 + t56 * (t41 * t274 * t45 + t59 * t122 - t74) * t141 + t80 * (t238 * t141 - t82 * t287) + t100 * (-t242 * t287 - t294);
    __hess_xu_xu[8] = __hess_xu_xu[2];
    __hess_xu_xu[9] = __hess_xu_xu[6];
    real t297 = t1 * t6;
    __hess_xu_xu[10] = 0.2e1 * t56 * t37 * t45 * t141 + 0.2e1 * t297 * t46 * t141;
    __hess_xu_xu[11] = t1 * (t186 * t39 + 0.2e1 * t72 * t192 - 0.2e1 * t59 * t46 - t162) * t141 + t56 * (-t6 * t191 * t39 + t32 * t44 * t39 + 0.2e1 * t187 * t45 + t148) * t141 + t80 * (-t81 * t191 * t141 + t264 * t141) + t100 * (-t101 * t191 * t141 + t31 + t43);
    __hess_xu_xu[12] = __hess_xu_xu[3];
    __hess_xu_xu[13] = __hess_xu_xu[7];
    __hess_xu_xu[14] = __hess_xu_xu[11];
    real t334 = -0.4e1 * t33 - 0.2e1 * t272 + 0.2e1 * t36;
    __hess_xu_xu[15] = t1 * (t11 * t334 * t39 - 0.2e1 * t189 * t192 - t207 + t225 - t47) * t141 + t56 * (-t161 * t12 * t32 + t41 * t334 * t45 + 0.2e1 * t59 * t204 - t232 + t74) * t141 + t241 + t246;
    real t351 = t100 * t11;
    __hess_xu_dxu[0] = t351 * t177;
    real t352 = t80 * t11;
    __hess_xu_dxu[1] = t352 * t177;
    real t353 = t1 * t11;
    real t355 = t37 * t25 * t2;
    real t357 = t56 * t6;
    __hess_xu_dxu[2] = t357 * t11 * t177 - t353 * t355;
    real t362 = t56 * t41;
    __hess_xu_dxu[3] = -t297 * t41 * t177 - t362 * t355;
    real t364 = t113 * t141;
    __hess_xu_dxu[4] = -t351 * t364;
    __hess_xu_dxu[5] = -t352 * t364;
    real t367 = t109 * t141;
    __hess_xu_dxu[6] = -t357 * t244 * t141 + t353 * t367;
    __hess_xu_dxu[7] = t297 * t41 * t113 * t141 + t362 * t367;
    __hess_xu_dxu[8] = -t100 * t44 * t141;
    __hess_xu_dxu[9] = -t80 * t44 * t141;
    __hess_xu_dxu[10] = t1 * t37 * t141 - t357 * t287;
    real t383 = t11 * t44 * t141;
    __hess_xu_dxu[11] = 0.2e1 * t56 * t11 * t37 * t141 + 0.2e1 * t297 * t383;
    real t389 = t191 * t141;
    __hess_xu_dxu[12] = -t351 * t389;
    __hess_xu_dxu[13] = -t352 * t389;
    real t392 = t186 * t141;
    __hess_xu_dxu[14] = -t357 * t294 * t141 + t56 * t32 * t383 + t353 * t392;
    real t398 = t1 * t32;
    __hess_xu_dxu[15] = t297 * t41 * t191 * t141 - t398 * t41 * t44 * t141 + t362 * t392;
    real t407 = t56 * t10;
    __hess_xu_axu[0] = t297 * t10 + t407 * t32;
    __hess_xu_axu[1] = -t398 * t12 + t407 * t72;

}

integer SingleMassPointE::foEqnsHessXuXuNnz ( integer const i_phase ) const {
    return 16;
return 0;
}
void SingleMassPointE::foEqnsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[]) const {
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
    rows[11] = 3;
    rows[12] = 0;
    rows[13] = 1;
    rows[14] = 2;
    rows[15] = 3;

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
    cols[11] = 2;
    cols[12] = 3;
    cols[13] = 3;
    cols[14] = 3;
    cols[15] = 3;

}

integer SingleMassPointE::foEqnsHessXuDxuNnz ( integer const i_phase ) const {
    return 16;
return 0;
}
void SingleMassPointE::foEqnsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
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
    rows[11] = 3;
    rows[12] = 0;
    rows[13] = 1;
    rows[14] = 2;
    rows[15] = 3;

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
    cols[11] = 2;
    cols[12] = 3;
    cols[13] = 3;
    cols[14] = 3;
    cols[15] = 3;

}

integer SingleMassPointE::foEqnsHessXuAxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void SingleMassPointE::foEqnsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 1;
    rows[1] = 1;

        cols[0] = 2;
    cols[1] = 3;

}

integer SingleMassPointE::foEqnsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::foEqnsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::foEqnsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::foEqnsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::foEqnsHessAxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::foEqnsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::foEqnsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::foEqnsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +-----------------------------------------------------------------------+
// |      _ _  __  __                       _             _       _        |
// |   __| (_)/ _|/ _|   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _` | | |_| |_   / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | (_| | |  _|  _| | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \__,_|_|_| |_|    \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                       |
// +-----------------------------------------------------------------------+

void SingleMassPointE::pathConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t2 = __algebraic_states_controls[0];
    real t5 = _model_params[MOD_PAR_INDEX_Fx_max];
    __values[0] = __states_controls[2] * t2 * t5 * _model_params[MOD_PAR_INDEX_m] * _model_params[MOD_PAR_INDEX_g];
    real t12 = t2 * t5;
    real t13 = __parameters[0];
    real t14 = cos(t13);
    real t19 = _model_params[MOD_PAR_INDEX_Klambda] * __states_controls[3];
    real t20 = sin(t13);
    real t23 = pow(t12 * t14 + t19 * t20, 0.2e1);
    real t26 = pow(_model_params[MOD_PAR_INDEX_DX], 0.2e1);
    real t32 = pow(-t12 * t20 + t19 * t14, 0.2e1);
    real t35 = pow(_model_params[MOD_PAR_INDEX_DY], 0.2e1);
    __values[1] = t23 / t26 + t32 / t35;

}

void SingleMassPointE::pathConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        real t3 = _model_params[MOD_PAR_INDEX_Fx_max];
    real t4 = __algebraic_states_controls[0] * t3;
    real t9 = _model_params[MOD_PAR_INDEX_m] * _model_params[MOD_PAR_INDEX_g];
    __jac_xu[0] = t4 * t9;
    real t10 = __parameters[0];
    real t11 = cos(t10);
    real t14 = _model_params[MOD_PAR_INDEX_Klambda];
    real t16 = t14 * __states_controls[3];
    real t17 = sin(t10);
    real t19 = t4 * t11 + t16 * t17;
    real t22 = pow(_model_params[MOD_PAR_INDEX_DX], 0.2e1);
    real t24 = t19 / t22;
    real t29 = t16 * t11 - t4 * t17;
    real t32 = pow(_model_params[MOD_PAR_INDEX_DY], 0.2e1);
    real t34 = t29 / t32;
    __jac_xu[1] = 0.2e1 * t34 * t14 * t11 + 0.2e1 * t24 * t14 * t17;
    __jac_axu[0] = __states_controls[2] * t3 * t9;
    __jac_axu[1] = 0.2e1 * t24 * t3 * t11 - 0.2e1 * t34 * t3 * t17;
    __jac_p[0] = -0.2e1 * t34 * t19 + 0.2e1 * t24 * t29;

}

integer SingleMassPointE::pathConstraintsJacXuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void SingleMassPointE::pathConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;

        cols[0] = 2;
    cols[1] = 3;

}

integer SingleMassPointE::pathConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pathConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pathConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void SingleMassPointE::pathConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;

        cols[0] = 0;
    cols[1] = 0;

}

integer SingleMassPointE::pathConstraintsJacPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::pathConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 1;

        cols[0] = 0;

}

void SingleMassPointE::pathConstraintsHess(integer const i_phase,
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
        real t1 = __lambda[1];
    real t3 = _model_params[MOD_PAR_INDEX_Klambda];
    real t4 = t3 * t3;
    real t5 = __parameters[0];
    real t6 = sin(t5);
    real t7 = t6 * t6;
    real t11 = pow(_model_params[MOD_PAR_INDEX_DX], 0.2e1);
    real t12 = 0.1e1 / t11;
    real t14 = cos(t5);
    real t15 = t14 * t14;
    real t19 = pow(_model_params[MOD_PAR_INDEX_DY], 0.2e1);
    real t20 = 0.1e1 / t19;
    __hess_xu_xu[0] = t1 * (0.2e1 * t4 * t7 * t12 + 0.2e1 * t4 * t15 * t20);
    real t26 = _model_params[MOD_PAR_INDEX_Fx_max];
    __hess_xu_axu[0] = __lambda[0] * t26 * _model_params[MOD_PAR_INDEX_m] * _model_params[MOD_PAR_INDEX_g];
    real t33 = t3 * t6;
    real t37 = t3 * t14;
    __hess_xu_axu[1] = t1 * (0.2e1 * t33 * t12 * t26 * t14 - 0.2e1 * t37 * t20 * t26 * t6);
    real t44 = __algebraic_states_controls[0] * t26;
    real t47 = t3 * __states_controls[3];
    real t49 = t47 * t14 - t44 * t6;
    real t50 = t12 * t49;
    real t54 = t44 * t14 + t47 * t6;
    real t55 = t54 * t12;
    real t57 = -t54;
    real t58 = t20 * t57;
    real t60 = t49 * t20;
    __hess_xu_p[0] = t1 * (0.2e1 * t33 * t50 - 0.2e1 * t60 * t33 + 0.2e1 * t55 * t37 + 0.2e1 * t37 * t58);
    real t64 = t26 * t26;
    __hess_axu_axu[0] = t1 * (0.2e1 * t64 * t15 * t12 + 0.2e1 * t64 * t7 * t20);
    real t71 = t26 * t14;
    real t73 = t26 * t6;
    __hess_axu_p[0] = t1 * (0.2e1 * t71 * t50 - 0.2e1 * t55 * t73 - 0.2e1 * t73 * t58 - 0.2e1 * t60 * t71);
    real t79 = t49 * t49;
    real t82 = t57 * t57;
    __hess_p_p[0] = t1 * (0.2e1 * t79 * t12 + 0.2e1 * t82 * t20 - 0.2e1 * t60 * t49 + 0.2e1 * t55 * t57);

}
integer SingleMassPointE::pathConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::pathConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 3;

        cols[0] = 3;

}

integer SingleMassPointE::pathConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pathConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pathConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 2;
return 0;
}
void SingleMassPointE::pathConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;

        cols[0] = 2;
    cols[1] = 3;

}

integer SingleMassPointE::pathConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::pathConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 3;

}

integer SingleMassPointE::pathConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pathConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pathConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pathConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pathConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pathConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pathConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::pathConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 0;

}

integer SingleMassPointE::pathConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::pathConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 0;

}

integer SingleMassPointE::pathConstraintsHessPPNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::pathConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 0;

}


// +----------------------------------------------------------------------------+
// |      _        _                             _             _       _        |
// |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                            |
// +----------------------------------------------------------------------------+


void SingleMassPointE::pointConstraints ( integer const i_phase,
                     real    const __states_controls[],
                     real    const __parameters[],
                     real          __zeta,
                     real          __values[] ) const {
    
}

void SingleMassPointE::pointConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_p[] ) const {
        

}

integer SingleMassPointE::pointConstraintsJacXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pointConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pointConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pointConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void SingleMassPointE::pointConstraintsHess (integer const i_phase,
                                 real    const __states_controls[],
                                 real    const __parameters[],
                                 real          __zeta,
                                 real    const __lambda[],
                                 real          __hess_xu_xu[],
                                 real          __hess_xu_p[],
                                 real          __hess_p_p[] ) const {
        

 }
integer SingleMassPointE::pointConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pointConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pointConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pointConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::pointConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::pointConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +-----------------------------------------------------------------------------------------+
// |  _       _                       _                       _             _       _        |
// | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                  |___/                                                                  |
// +-----------------------------------------------------------------------------------------+

void SingleMassPointE::intConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
        real t2 = _p_RegularizedPositive->funcEval(__algebraic_states_controls[0]);
    real t11 = __states_controls[3];
    real t12 = cos(t11);
    real t13 = __states_controls[1];
    real t14 = cos(t13);
    real t16 = sin(t11);
    real t17 = sin(t13);
    real t23 = _p_RoadCurvature->funcEval(__zeta);
    __values[0] = -t2 * _model_params[MOD_PAR_INDEX_Fx_max] * _model_params[MOD_PAR_INDEX_m] * _model_params[MOD_PAR_INDEX_g] / (t12 * t14 + t16 * t17) * (__states_controls[0] * t23 - 0.1e1);

}

void SingleMassPointE::intConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
        real t1 = __algebraic_states_controls[0];
    real t2 = _p_RegularizedPositive->funcEval(t1);
    real t4 = _model_params[MOD_PAR_INDEX_Fx_max];
    real t7 = _model_params[MOD_PAR_INDEX_m];
    real t8 = t2 * t4 * t7;
    real t10 = _model_params[MOD_PAR_INDEX_g];
    real t11 = __states_controls[3];
    real t12 = cos(t11);
    real t13 = __states_controls[1];
    real t14 = cos(t13);
    real t16 = sin(t11);
    real t17 = sin(t13);
    real t19 = t12 * t14 + t16 * t17;
    real t21 = t10 / t19;
    real t22 = _p_RoadCurvature->funcEval(__zeta);
    __jac_xu[0] = -t8 * t21 * t22;
    real t25 = t19 * t19;
    real t27 = t10 / t25;
    real t30 = __states_controls[0] * t22 - 0.1e1;
    real t33 = -t12 * t17 + t16 * t14;
    __jac_xu[1] = t8 * t27 * t30 * t33;
    __jac_xu[2] = -t8 * t27 * t30 * t33;
    real t39 = _p_RegularizedPositive->funcEval_D_1(t1);
    __jac_axu[0] = -t39 * t4 * t7 * t21 * t30;

}

integer SingleMassPointE::intConstraintsJacXuNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void SingleMassPointE::intConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 3;

}

integer SingleMassPointE::intConstraintsJacDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::intConstraintsJacAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::intConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 0;

}

integer SingleMassPointE::intConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void SingleMassPointE::intConstraintsHess(integer const i_phase,
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
    real t2 = __algebraic_states_controls[0];
    real t3 = _p_RegularizedPositive->funcEval(t2);
    real t4 = t1 * t3;
    real t6 = _model_params[MOD_PAR_INDEX_Fx_max];
    real t8 = _model_params[MOD_PAR_INDEX_m];
    real t9 = t6 * t8;
    real t10 = t4 * t9;
    real t12 = _model_params[MOD_PAR_INDEX_g];
    real t13 = __states_controls[3];
    real t14 = cos(t13);
    real t15 = __states_controls[1];
    real t16 = cos(t15);
    real t18 = sin(t13);
    real t19 = sin(t15);
    real t21 = t14 * t16 + t18 * t19;
    real t22 = t21 * t21;
    real t24 = t12 / t22;
    real t25 = _p_RoadCurvature->funcEval(__zeta);
    real t28 = -t14 * t19 + t18 * t16;
    __hess_xu_xu[0] = t10 * t24 * t25 * t28;
    real t31 = -t28;
    __hess_xu_xu[1] = t10 * t24 * t25 * t31;
    __hess_xu_xu[2] = __hess_xu_xu[0];
    real t36 = t12 / t22 / t21;
    real t39 = __states_controls[0] * t25 - 0.1e1;
    real t40 = t28 * t28;
    real t48 = -t10 * t24 * t39 * t21;
    __hess_xu_xu[3] = -0.2e1 * t10 * t36 * t39 * t40 + t48;
    real t49 = t39 * t28;
    real t55 = t8 * t12;
    real t56 = 0.1e1 / t21;
    real t58 = t55 * t56 * t39;
    __hess_xu_xu[4] = -0.2e1 * t10 * t36 * t49 * t31 + t4 * t6 * t58;
    __hess_xu_xu[5] = __hess_xu_xu[1];
    __hess_xu_xu[6] = __hess_xu_xu[4];
    real t60 = t31 * t31;
    __hess_xu_xu[7] = -0.2e1 * t10 * t36 * t39 * t60 + t48;
    real t65 = _p_RegularizedPositive->funcEval_D_1(t2);
    real t66 = t1 * t65;
    __hess_xu_axu[0] = -t66 * t6 * t55 * t56 * t25;
    real t71 = t66 * t9;
    __hess_xu_axu[1] = t71 * t24 * t49;
    __hess_xu_axu[2] = t71 * t24 * t39 * t31;
    real t75 = _p_RegularizedPositive->funcEval_D_1_1(t2);
    __hess_axu_axu[0] = -t1 * t75 * t6 * t58;

}
integer SingleMassPointE::intConstraintsHessXuXuNnz ( integer const i_phase ) const {
    return 8;
return 0;
}
void SingleMassPointE::intConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 1;
    rows[1] = 3;
    rows[2] = 0;
    rows[3] = 1;
    rows[4] = 3;
    rows[5] = 0;
    rows[6] = 1;
    rows[7] = 3;

        cols[0] = 0;
    cols[1] = 0;
    cols[2] = 1;
    cols[3] = 1;
    cols[4] = 1;
    cols[5] = 3;
    cols[6] = 3;
    cols[7] = 3;

}

integer SingleMassPointE::intConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::intConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    return 3;
return 0;
}
void SingleMassPointE::intConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 0;
    rows[2] = 0;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 3;

}

integer SingleMassPointE::intConstraintsHessXuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::intConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::intConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::intConstraintsHessDxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::intConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::intConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;

        cols[0] = 0;

}

integer SingleMassPointE::intConstraintsHessAxuPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::intConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::intConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

//   +--------------------------------------------------------------------------------------------------+
//   |  _                           _                                        _ _ _   _                  |
//   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
//   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
//   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
//   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
//   |                                           |___/                                                  |
//   +--------------------------------------------------------------------------------------------------+

void SingleMassPointE::boundaryConditions ( integer const i_phase,
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
    __values[4] = __final_state_control[2];

}

void SingleMassPointE::boundaryConditionsJac ( integer const i_phase,
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

}

integer SingleMassPointE::boundaryConditionsJacXuInitNnz ( integer const i_phase ) const {
    return 4;
return 0;
}
void SingleMassPointE::boundaryConditionsJacXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 0;
    rows[1] = 1;
    rows[2] = 2;
    rows[3] = 3;

        cols[0] = 0;
    cols[1] = 1;
    cols[2] = 2;
    cols[3] = 3;

}

integer SingleMassPointE::boundaryConditionsJacXuFinNnz ( integer const i_phase ) const {
    return 1;
return 0;
}
void SingleMassPointE::boundaryConditionsJacXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        rows[0] = 4;

        cols[0] = 2;

}

integer SingleMassPointE::boundaryConditionsJacPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::boundaryConditionsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

void SingleMassPointE::boundaryConditionsHess ( integer const i_phase,
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

integer SingleMassPointE::boundaryConditionsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::boundaryConditionsHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::boundaryConditionsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::boundaryConditionsHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::boundaryConditionsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::boundaryConditionsHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::boundaryConditionsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::boundaryConditionsHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::boundaryConditionsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::boundaryConditionsHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

integer SingleMassPointE::boundaryConditionsHessPPNnz ( integer const i_phase ) const {
    return 0;
return 0;
}
void SingleMassPointE::boundaryConditionsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
        

        

}

// +--------------------------------------------------------------------------------+
// |                       _                         _             _       _        |
// |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                                |
// +--------------------------------------------------------------------------------+

void SingleMassPointE::eventConstraints ( integer const i_phase,
                      real const left_state_control[],
                      real const right_state_control[],
                      real const parameters[],
                      real const __zeta_l,
                      real const __zeta_r,
                      real       __values[] ) const {
}

void SingleMassPointE::eventConstraintsJac ( integer const i_phase,
                             real const left_state_control[],
                             real const right_state_control[],
                             real const parameters[],
                             real const __zeta_l,
                             real const __zeta_r,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
}

integer SingleMassPointE::eventConstraintsJacXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsJacXuInitPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer SingleMassPointE::eventConstraintsJacXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsJacXuFinPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer SingleMassPointE::eventConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsJacPPattern ( integer const i_phase,
                               integer       rows[],
                               integer       cols[] ) const {

}

void SingleMassPointE::eventConstraintsHess ( integer const i_phase,
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

integer SingleMassPointE::eventConstraintsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsHessXuInitXuInitPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer SingleMassPointE::eventConstraintsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsHessXuInitXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer SingleMassPointE::eventConstraintsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsHessXuInitPPattern ( integer const i_phase,
                                     integer       rows[],
                                     integer       cols[] ) const {

}

integer SingleMassPointE::eventConstraintsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsHessXuFinXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer SingleMassPointE::eventConstraintsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsHessXuFinPPattern ( integer const i_phase,
                                  integer       rows[],
                                  integer       cols[] ) const {

}

integer SingleMassPointE::eventConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
}
void SingleMassPointE::eventConstraintsHessPPPattern ( integer const i_phase,
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

integer SingleMassPointE::numberOfPostProcessing( integer const i_phase ) const { return (integer) _post_processing_names.at(i_phase).size(); }
integer SingleMassPointE::numberOfDifferentialPostProcessing( integer const i_phase ) const { return (integer) _differential_post_processing_names.at(i_phase).size(); }
integer SingleMassPointE::numberOfIntegralPostProcessing( integer const i_phase ) const { return (integer) _integral_post_processing_names.at(i_phase).size(); }
std::string SingleMassPointE::postProcessingName( integer const i_phase, integer const i ) const { return _post_processing_names.at(i_phase).at(i); }
std::string SingleMassPointE::differentialPostProcessingName( integer const i_phase, integer const i ) const { return _differential_post_processing_names.at(i_phase).at(i); }
std::string SingleMassPointE::integralPostProcessingName( integer const i_phase, integer const i ) const { return _integral_post_processing_names.at(i_phase).at(i); }

void SingleMassPointE::postProcessing(integer const i_phase,
                                                real    const __states_controls[],
                                                real    const __parameters[],
                                                real          __zeta,
                                                real          __values[] ) const {
        __values[0] = _p_RoadX->funcEval(__zeta);
    __values[1] = _p_RoadY->funcEval(__zeta);
    __values[2] = _p_RoadXL->funcEval(__zeta);
    __values[3] = _p_RoadYL->funcEval(__zeta);
    __values[4] = _p_RoadXR->funcEval(__zeta);
    __values[5] = _p_RoadYR->funcEval(__zeta);
    real t1 = __states_controls[0];
    real t2 = _p_RoadHeading->funcEval(__zeta);
    real t3 = sin(t2);
    __values[6] = -t1 * t3 + __values[0];
    real t5 = cos(t2);
    __values[7] = t1 * t5 + __values[1];
    __values[8] = t2;
    real t8 = _model_params[MOD_PAR_INDEX_Klambda];
    real t9 = __states_controls[3];
    __values[9] = t8 * t9;
    __values[10] = t8 * t9 * _model_params[MOD_PAR_INDEX_g];
    __values[11] = _p_RoadLeftWidth->funcEval(__zeta);
    __values[12] = _p_RoadRightWidth->funcEval(__zeta);

}

void SingleMassPointE::differentialPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
        real t2 = __states_controls[2];
    real t4 = __states_controls[3];
    real t5 = cos(t4);
    real t6 = __states_controls[1];
    real t7 = cos(t6);
    real t9 = sin(t4);
    real t10 = sin(t6);
    real t12 = t9 * t10 + t5 * t7;
    real t14 = _p_RoadCurvature->funcEval(__zeta);
    real t16 = __states_controls[0] * t14 - 0.1e1;
    __values[0] = -__state_control_derivatives[2] * t2 * t12 / t16;
    real t20 = __algebraic_states_controls[1];
    real t22 = _model_params[MOD_PAR_INDEX_Omega_max];
    __values[1] = t20 * t22 * t2;
    real t24 = __algebraic_states_controls[0];
    real t27 = _model_params[MOD_PAR_INDEX_Fx_max];
    real t29 = _model_params[MOD_PAR_INDEX_m];
    real t32 = _model_params[MOD_PAR_INDEX_g];
    __values[2] = t2 * t24 * t27 * t29 * t32;
    __values[3] = t20 * t22;
    __values[4] = t24 * t27 * t32;
    real t35 = _p_RegularizedPositive->funcEval(t24);
    __values[5] = -t35 * t27 * t29 * t32 / t12 * t16;

}

void SingleMassPointE::integralPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
        real t3 = __states_controls[3];
    real t4 = cos(t3);
    real t5 = __states_controls[1];
    real t6 = cos(t5);
    real t8 = sin(t3);
    real t9 = sin(t5);
    real t12 = 0.1e1 / (t4 * t6 + t8 * t9);
    real t15 = _p_RoadCurvature->funcEval(__zeta);
    real t17 = __states_controls[0] * t15 - 0.1e1;
    __values[0] = -0.1e1 / __states_controls[2] * t12 * t17;
    real t20 = _p_RegularizedPositive->funcEval(__algebraic_states_controls[0]);
    __values[1] = -t20 * _model_params[MOD_PAR_INDEX_Fx_max] * _model_params[MOD_PAR_INDEX_m] * _model_params[MOD_PAR_INDEX_g] * t12 * t17;

}
