#include "_MAVERICK_ENTER_CLASS_NAME.hh"
#include <tgmath.h>

using namespace Maverick;
using namespace _MAVERICK_ENTER_CLASS_NAMENamespace;
_MAVERICK_ENTER_EXT_OBJECTS_NAMESPACE

_MAVERICK_ENTER_CLASS_NAME::_MAVERICK_ENTER_CLASS_NAME() {

    _num_p = _MAVERICK_ENTER_NUM_PHASES;

    _MAVERICK_ENTER_STATE_DIM_VECTOR
    _MAVERICK_ENTER_STATE_NAMES_VECTOR

    _MAVERICK_ENTER_ALG_STATE_DIM_VECTOR
    _MAVERICK_ENTER_ALG_STATE_NAMES_VECTOR

    _MAVERICK_ENTER_CONTROL_DIM_VECTOR
    _MAVERICK_ENTER_CONTROL_NAMES_VECTOR

    _MAVERICK_ENTER_ALG_CONTROL_DIM_VECTOR
    _MAVERICK_ENTER_ALG_CONTROL_NAMES_VECTOR

    _MAVERICK_ENTER_PARAM_DIM_VECTOR
    _MAVERICK_ENTER_PARAM_NAMES_VECTOR

    _MAVERICK_ENTER_STATE_CONSTRAINTS_DIM_VECTOR
    _MAVERICK_ENTER_STATE_CONSTRAINTS_NAMES_VECTOR

    _MAVERICK_ENTER_PATH_CONSTR_DIM_VECTOR
    _MAVERICK_ENTER_PATH_CONSTR_NAMES_VECTOR

    _MAVERICK_ENTER_INT_CONSTR_DIM_VECTOR
    _MAVERICK_ENTER_INT_CONSTR_NAMES_VECTOR

    _MAVERICK_ENTER_BC_DIM_VECTOR
    _MAVERICK_ENTER_BC_NAMES_VECTOR

    _MAVERICK_ENTER_EVENT_CONSTRAINT_DIM_VECTOR
    _MAVERICK_ENTER_EVENT_CONSTRAINT_NAMES_VECTOR

    _post_processing_names = { _MAVERICK_ENTER_POST_PROC_NAMES };
    _differential_post_processing_names = { _MAVERICK_ENTER_DIFFERENTIAL_POST_PROC_NAMES };
    _integral_post_processing_names = { _MAVERICK_ENTER_INTEGRAL_POST_PROC_NAMES };

    _model_params = new real[NUM_MODEL_PARAMS]; // deleted automatically by parent's destructor
    _model_params_names = {_MAVERICK_ENTER_PARAMS_NAMES};

}

void _MAVERICK_ENTER_CLASS_NAME::derivedSetup(GC::GenericContainer const & gc) {
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
        throw std::runtime_error("_MAVERICK_ENTER_CLASS_NAME: not all parameters have been set. Missing parameters are:\n" + missing_params.str());
    }

_MAVERICK_ENTER_EXT_OBJECTS_SETUP
}

void _MAVERICK_ENTER_CLASS_NAME::printDerivedInfo(std::ostream & out, InfoLevel info_level) const {
    if (info_level >= info_level_verbose) {
_MAVERICK_ENTER_DERIVED_INFO
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

void _MAVERICK_ENTER_CLASS_NAME::getStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    _MAVERICK_ENTER_STATE_CONTROL_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_STATE_CONTROL_BOUNDS_UPPER_VECTOR
}

void _MAVERICK_ENTER_CLASS_NAME::getAlgebraicStatesControlsBounds(integer const i_phase,
                                                            real    const __zeta,
                                                            real          lower[],
                                                            real          upper[] ) const {
    _MAVERICK_ENTER_ALG_STATE_CONTROL_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_ALG_STATE_CONTROL_BOUNDS_UPPER_VECTOR
}

void _MAVERICK_ENTER_CLASS_NAME::getParametersBounds(integer const i_phase,
                                                        real          lower[],
                                                        real          upper[] ) const {
    _MAVERICK_ENTER_PARAMETER_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_PARAMETER_BOUNDS_UPPER_VECTOR
}

void _MAVERICK_ENTER_CLASS_NAME::getPathConstraintsBounds(integer const i_phase,
                                                             real    const __zeta,
                                                             real          lower[],
                                                             real          upper[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_PATH_CONSTR_BOUNDS_UPPER_VECTOR
}

void _MAVERICK_ENTER_CLASS_NAME::getIntConstraintsBounds(integer const i_phase,
                                                            real    const __zeta_i,
                                                            real    const __zeta_f,
                                                            real          lower[],
                                                            real          upper[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_INT_CONSTR_BOUNDS_UPPER_VECTOR
}

void _MAVERICK_ENTER_CLASS_NAME::getBoundaryConditionsBounds(integer const i_phase,
                                                                real    const __zeta_i,
                                                                real    const __zeta_f,
                                                                real          lower[],
                                                                real          upper[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_BOUNDS_UPPER_VECTOR
}

void _MAVERICK_ENTER_CLASS_NAME::getPointConstraintsBounds(integer const i_phase,
                                                              real    const __zeta,
                                                              real          lower[],
                                                              real          upper[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_STATE_CONSTRAINTS_BOUNDS_UPPER_VECTOR
}

void _MAVERICK_ENTER_CLASS_NAME::getEventConstraintsBounds(integer const i_phase,
                                                              real    const __zeta_i,
                                                              real    const __zeta_f,
                                                              real          lower[],
                                                              real          upper[] ) const {
    _MAVERICK_ENTER_EVENT_CONSTRAINTS_BOUNDS_LOWER_VECTOR
    _MAVERICK_ENTER_EVENT_CONSTRAINTS_BOUNDS_UPPER_VECTOR
}

_MAVERICK_ENTER_GUESS

//   +-----------------------------------+
//   |                                   |
//   |  _ __ ___   __ _ _   _  ___ _ __  |
//   | | '_ ` _ \ / _` | | | |/ _ \ '__| |
//   | | | | | | | (_| | |_| |  __/ |    |
//   | |_| |_| |_|\__,_|\__, |\___|_|    |
//   |                  |___/            |
//   +-----------------------------------+

void _MAVERICK_ENTER_CLASS_NAME::mayer ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real     & __value ) const {
    _MAVERICK_ENTER_MAYER_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::mayerJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
    _MAVERICK_ENTER_MAYER_J_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerJacXuInitNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_J_XUINIT_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerJacXuInitPattern ( integer const i_phase, integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_J_XUINIT_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerJacXuFinNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_J_XUFIN_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerJacXuFinPattern ( integer const i_phase, integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_J_XUFIN_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerJacPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_J_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerJacPPattern ( integer const i_phase, integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_J_P_COL_PATTERN
}

void _MAVERICK_ENTER_CLASS_NAME::mayerHess ( integer const i_phase,
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
    _MAVERICK_ENTER_MAYER_H_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerHessXuInitXuInitNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_H_XUINIT_XUINIT_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_H_XUINIT_XUINIT_ROW_PATTERN
    _MAVERICK_ENTER_MAYER_H_XUINIT_XUINIT_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerHessXuInitXuFinNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_H_XUINIT_XUFIN_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_H_XUINIT_XUFIN_ROW_PATTERN
    _MAVERICK_ENTER_MAYER_H_XUINIT_XUFIN_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerHessXuInitPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_H_XUINIT_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_H_XUINIT_P_ROW_PATTERN
    _MAVERICK_ENTER_MAYER_H_XUINIT_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerHessXuFinXuFinNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_H_XUFIN_XUFIN_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_H_XUFIN_XUFIN_ROW_PATTERN
    _MAVERICK_ENTER_MAYER_H_XUFIN_XUFIN_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerHessXuFinPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_H_XUFIN_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_H_XUFIN_P_ROW_PATTERN
    _MAVERICK_ENTER_MAYER_H_XUFIN_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::mayerHessPPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_MAYER_H_P_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::mayerHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_MAYER_H_P_P_ROW_PATTERN
    _MAVERICK_ENTER_MAYER_H_P_P_COL_PATTERN
}


//   +--------------------------------------------+
//   |  _                                         |
//   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
//   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
//   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
//   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
//   |          |___/                 |___/       |
//   +--------------------------------------------+

 void _MAVERICK_ENTER_CLASS_NAME::lagrange ( integer const i_phase,
                           real    const __states_controls[],
                           real    const __state_control_derivatives[],
                           real    const __algebraic_states_controls[],
                           real    const __parameters[],
                           real          __zeta,
                           real        & __value ) const {
   _MAVERICK_ENTER_LAGRANGE_BODY
}

 void _MAVERICK_ENTER_CLASS_NAME::lagrangeJac ( integer const i_phase,
                                real    const __states_controls[],
                                real    const __state_control_derivatives[],
                                real    const __algebraic_states_controls[],
                                real    const __parameters[],
                                real          __zeta,
                                real          __jac_xu[],
                                real          __jac_dxu[],
                                real          __jac_axu[],
                                real          __jac_p[] ) const {
    _MAVERICK_ENTER_LAGRANGE_J_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeJacXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_J_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::lagrangeJacXuPattern ( integer const i_phase, integer cols[] ) const {
    _MAVERICK_ENTER_LAGRANGE_J_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeJacDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_J_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::lagrangeJacDxuPattern ( integer const i_phase, integer cols[] ) const {
    _MAVERICK_ENTER_LAGRANGE_J_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeJacAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_J_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::lagrangeJacAxuPattern ( integer const i_phase, integer cols[] ) const {
    _MAVERICK_ENTER_LAGRANGE_J_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeJacPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_J_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::lagrangeJacPPattern ( integer const i_phase, integer cols[] ) const {
    _MAVERICK_ENTER_LAGRANGE_J_P_COL_PATTERN
}

 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHess ( integer const i_phase,
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
     _MAVERICK_ENTER_LAGRANGE_H_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_XU_XU_NNZ
}
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_XU_XU_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_XU_XU_COL_PATTERN
 }

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_XU_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
 _MAVERICK_ENTER_LAGRANGE_H_XU_DXU_ROW_PATTERN
 _MAVERICK_ENTER_LAGRANGE_H_XU_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_XU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
 _MAVERICK_ENTER_LAGRANGE_H_XU_AXU_ROW_PATTERN
 _MAVERICK_ENTER_LAGRANGE_H_XU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_XU_P_NNZ
}
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_XU_P_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_XU_P_COL_PATTERN
 }

 integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessDxuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_DXU_DXU_NNZ
}
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_DXU_DXU_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_DXU_DXU_COL_PATTERN
 }

 integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessDxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_DXU_AXU_NNZ
}
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_DXU_AXU_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_DXU_AXU_COL_PATTERN
 }

 integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessDxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_DXU_P_NNZ
}
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_DXU_P_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_DXU_P_COL_PATTERN
 }

 integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessAxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_AXU_AXU_NNZ
 }
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_AXU_AXU_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_AXU_AXU_COL_PATTERN
 }

 integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessAxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_AXU_P_NNZ
 }
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_AXU_P_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_AXU_P_COL_PATTERN
 }

 integer _MAVERICK_ENTER_CLASS_NAME::lagrangeHessPPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_LAGRANGE_H_P_P_NNZ
}
 void _MAVERICK_ENTER_CLASS_NAME::lagrangeHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
     _MAVERICK_ENTER_LAGRANGE_H_P_P_ROW_PATTERN
     _MAVERICK_ENTER_LAGRANGE_H_P_P_COL_PATTERN
 }

//   +-------------------------------------------------------------+
//   |   __                                 _   _                  |
//   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
//   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
//   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
//   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
//   |                       |_|                                   |
//   +-------------------------------------------------------------+

void _MAVERICK_ENTER_CLASS_NAME::foEqns ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    _MAVERICK_ENTER_FO_EQNS_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::foEqnsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
    _MAVERICK_ENTER_FO_EQNS_J_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsJacXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_J_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_J_XU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_J_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsJacDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_J_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_J_DXU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_J_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsJacAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_J_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_J_AXU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_J_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsJacPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_J_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_J_P_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_J_P_COL_PATTERN
}

void _MAVERICK_ENTER_CLASS_NAME::foEqnsHess(integer const i_phase,
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
    _MAVERICK_ENTER_FO_EQNS_H_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[]) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_XU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_XU_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_DXU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_XU_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_XU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_XU_P_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_XU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessDxuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_DXU_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_DXU_DXU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_DXU_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessDxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_DXU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_DXU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_DXU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessDxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_DXU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_DXU_P_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_DXU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessAxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_AXU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_AXU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_AXU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessAxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_AXU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_AXU_P_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_AXU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::foEqnsHessPPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_FO_EQNS_H_P_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::foEqnsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_FO_EQNS_H_P_P_ROW_PATTERN
    _MAVERICK_ENTER_FO_EQNS_H_P_P_COL_PATTERN
}

// +-----------------------------------------------------------------------+
// |      _ _  __  __                       _             _       _        |
// |   __| (_)/ _|/ _|   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _` | | |_| |_   / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | (_| | |  _|  _| | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \__,_|_|_| |_|    \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                       |
// +-----------------------------------------------------------------------+

void _MAVERICK_ENTER_CLASS_NAME::pathConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_XU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_J_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_DXU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_J_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_AXU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_J_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_J_P_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_J_P_COL_PATTERN
}

void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHess(integer const i_phase,
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
    _MAVERICK_ENTER_PATH_CONSTR_H_BODY
}
integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_XU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_DXU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_P_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_XU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_DXU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessDxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_P_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_DXU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_AXU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_AXU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_AXU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessAxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_AXU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_AXU_P_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_AXU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessPPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_P_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pathConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_PATH_CONSTR_H_P_P_ROW_PATTERN
    _MAVERICK_ENTER_PATH_CONSTR_H_P_P_COL_PATTERN
}


// +----------------------------------------------------------------------------+
// |      _        _                             _             _       _        |
// |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                            |
// +----------------------------------------------------------------------------+


void _MAVERICK_ENTER_CLASS_NAME::pointConstraints ( integer const i_phase,
                     real    const __states_controls[],
                     real    const __parameters[],
                     real          __zeta,
                     real          __values[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::pointConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_p[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_J_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::pointConstraintsJacXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_J_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pointConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_J_XU_ROW_PATTERN
    _MAVERICK_ENTER_STATE_CONSTRAINTS_J_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pointConstraintsJacPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_J_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pointConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_J_P_ROW_PATTERN
    _MAVERICK_ENTER_STATE_CONSTRAINTS_J_P_COL_PATTERN
}

void _MAVERICK_ENTER_CLASS_NAME::pointConstraintsHess (integer const i_phase,
                                 real    const __states_controls[],
                                 real    const __parameters[],
                                 real          __zeta,
                                 real    const __lambda[],
                                 real          __hess_xu_xu[],
                                 real          __hess_xu_p[],
                                 real          __hess_p_p[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_BODY
 }
integer _MAVERICK_ENTER_CLASS_NAME::pointConstraintsHessXuXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_XU_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pointConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_XU_XU_ROW_PATTERN
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_XU_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pointConstraintsHessXuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_XU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pointConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_XU_P_ROW_PATTERN
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_XU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::pointConstraintsHessPPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_P_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::pointConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_P_P_ROW_PATTERN
    _MAVERICK_ENTER_STATE_CONSTRAINTS_H_P_P_COL_PATTERN
}

// +-----------------------------------------------------------------------------------------+
// |  _       _                       _                       _             _       _        |
// | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                  |___/                                                                  |
// +-----------------------------------------------------------------------------------------+

void _MAVERICK_ENTER_CLASS_NAME::intConstraints ( integer const i_phase,
                 real    const __states_controls[],
                 real    const __state_control_derivatives[],
                 real    const __algebraic_states_controls[],
                 real    const __parameters[],
                 real          __zeta,
                 real          __values[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::intConstraintsJac (integer const i_phase,
                          real const __states_controls[],
                          real const __state_control_derivatives[],
                          real const __algebraic_states_controls[],
                          real const __parameters[],
                          real       __zeta,
                          real       __jac_xu[],
                          real       __jac_dxu[],
                          real       __jac_axu[],
                          real       __jac_p[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_XU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_J_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_DXU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_J_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_AXU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_J_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_J_P_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_J_P_COL_PATTERN
}

void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHess(integer const i_phase,
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
    _MAVERICK_ENTER_INT_CONSTR_H_BODY
}
integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuXuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_XU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_XU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_XU_XU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_DXU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_XU_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_XU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_XU_P_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_XU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessDxuDxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_DXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_DXU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_DXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessDxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessDxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_P_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_DXU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessAxuAxuNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_AXU_AXU_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_AXU_AXU_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_AXU_AXU_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessAxuPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_AXU_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_AXU_P_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_AXU_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessPPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_P_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::intConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_INT_CONSTR_H_P_P_ROW_PATTERN
    _MAVERICK_ENTER_INT_CONSTR_H_P_P_COL_PATTERN
}

//   +--------------------------------------------------------------------------------------------------+
//   |  _                           _                                        _ _ _   _                  |
//   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
//   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
//   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
//   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
//   |                                           |___/                                                  |
//   +--------------------------------------------------------------------------------------------------+

void _MAVERICK_ENTER_CLASS_NAME::boundaryConditions ( integer const i_phase,
               real const __initial_state_control[],
               real const __final_state_control[],
               real const __parameters[],
               real       __zeta_i,
               real       __zeta_f,
               real       __values[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsJac ( integer const i_phase,
                             real const __initial_state_control[],
                             real const __final_state_control[],
                             real const __parameters[],
                             real       __zeta_i,
                             real       __zeta_f,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsJacXuInitNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_XUINIT_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsJacXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_XUINIT_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_XUINIT_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsJacXuFinNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_XUFIN_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsJacXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_XUFIN_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_XUFIN_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsJacPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_P_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_J_P_COL_PATTERN
}

void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHess ( integer const i_phase,
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
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_BODY
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuInitXuInitNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_XUINIT_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuInitXuInitPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_XUINIT_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_XUINIT_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuInitXuFinNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_XUFIN_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuInitXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_XUFIN_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_XUFIN_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuInitPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuInitPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_P_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUINIT_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuFinXuFinNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUFIN_XUFIN_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuFinXuFinPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUFIN_XUFIN_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUFIN_XUFIN_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuFinPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUFIN_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessXuFinPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUFIN_P_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_XUFIN_P_COL_PATTERN
}

integer _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessPPNnz ( integer const i_phase ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_P_P_NNZ
}
void _MAVERICK_ENTER_CLASS_NAME::boundaryConditionsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const {
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_P_P_ROW_PATTERN
    _MAVERICK_ENTER_BOUNDARY_CONDITIONS_H_P_P_COL_PATTERN
}

// +--------------------------------------------------------------------------------+
// |                       _                         _             _       _        |
// |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
// |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
// | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
// |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
// |                                                                                |
// +--------------------------------------------------------------------------------+

void _MAVERICK_ENTER_CLASS_NAME::eventConstraints ( integer const i_phase,
                      real const left_state_control[],
                      real const right_state_control[],
                      real const parameters[],
                      real const __zeta_l,
                      real const __zeta_r,
                      real       __values[] ) const {
}

void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsJac ( integer const i_phase,
                             real const left_state_control[],
                             real const right_state_control[],
                             real const parameters[],
                             real const __zeta_l,
                             real const __zeta_r,
                             real       __jac_xu_init[],
                             real       __jac_xu_fin[],
                             real       __jac_p[] ) const {
}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsJacXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsJacXuInitPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsJacXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsJacXuFinPattern ( integer const i_phase,
                                    integer rows[],
                                    integer cols[] ) const {

}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsJacPNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsJacPPattern ( integer const i_phase,
                               integer       rows[],
                               integer       cols[] ) const {

}

void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHess ( integer const i_phase,
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

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuInitXuInitNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuInitXuInitPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuInitXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuInitXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuInitPNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuInitPPattern ( integer const i_phase,
                                     integer       rows[],
                                     integer       cols[] ) const {

}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuFinXuFinNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuFinXuFinPattern ( integer const i_phase,
                                          integer rows[],
                                          integer cols[] ) const {

}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuFinPNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessXuFinPPattern ( integer const i_phase,
                                  integer       rows[],
                                  integer       cols[] ) const {

}

integer _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessPPNnz ( integer const i_phase ) const {
    return 0;
}
void _MAVERICK_ENTER_CLASS_NAME::eventConstraintsHessPPPattern ( integer const i_phase,
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

integer _MAVERICK_ENTER_CLASS_NAME::numberOfPostProcessing( integer const i_phase ) const { return (integer) _post_processing_names.at(i_phase).size(); }
integer _MAVERICK_ENTER_CLASS_NAME::numberOfDifferentialPostProcessing( integer const i_phase ) const { return (integer) _differential_post_processing_names.at(i_phase).size(); }
integer _MAVERICK_ENTER_CLASS_NAME::numberOfIntegralPostProcessing( integer const i_phase ) const { return (integer) _integral_post_processing_names.at(i_phase).size(); }
std::string _MAVERICK_ENTER_CLASS_NAME::postProcessingName( integer const i_phase, integer const i ) const { return _post_processing_names.at(i_phase).at(i); }
std::string _MAVERICK_ENTER_CLASS_NAME::differentialPostProcessingName( integer const i_phase, integer const i ) const { return _differential_post_processing_names.at(i_phase).at(i); }
std::string _MAVERICK_ENTER_CLASS_NAME::integralPostProcessingName( integer const i_phase, integer const i ) const { return _integral_post_processing_names.at(i_phase).at(i); }

void _MAVERICK_ENTER_CLASS_NAME::postProcessing(integer const i_phase,
                                                real    const __states_controls[],
                                                real    const __parameters[],
                                                real          __zeta,
                                                real          __values[] ) const {
    _MAVERICK_ENTER_POST_PROC_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::differentialPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
    _MAVERICK_ENTER_DIFFERENTIAL_POST_PROC_BODY
}

void _MAVERICK_ENTER_CLASS_NAME::integralPostProcessing(integer const i_phase,
                                                        real    const __states_controls[],
                                                        real    const __state_control_derivatives[],
                                                        real    const __algebraic_states_controls[],
                                                        real    const __parameters[],
                                                        real          __zeta,
                                                        real          __values[] ) const {
    _MAVERICK_ENTER_INTEGRAL_POST_PROC_BODY
}
