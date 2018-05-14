#include "MidpointOcp2NlpSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace Maverick;

void MidpointOcp2NlpSinglePhase::calculateNlpConstraintsBetweenMeshPoints(integer const first_mesh_point,
                                                                          integer const last_mesh_point,
                                                                          real const nlp_y[], real const ocp_params[],
                                                                          real constraints_out[],
                                                                          real int_constraints_out[],
                                                                          std::exception_ptr *exc_ptr
) const {
  try {

    // Y and P pointers
    real const *p_current_y_scaled = nlp_y;

    // OUTPUT STREAMS
    // constraints
    real *p_current_constraint = constraints_out;

    // now run over the entire _p_mesh intervals


    // BEGIN LOOP OVER MESH POINTS, MAIN PART
    for (integer current_mesh_interval = first_mesh_point;
         current_mesh_interval < last_mesh_point; current_mesh_interval++) {

      //zeta
      real const zeta = _p_mesh->getZetaCenter(current_mesh_interval);
      real const d_zeta = _p_mesh->getDz(current_mesh_interval);
      real const d_zeta_dual = _p_mesh->getDzDual(current_mesh_interval);

      //variables
      real const *p_left_state_control_scaled = p_current_y_scaled;
      real const *p_right_state_control_scaled = p_current_y_scaled + _dim_y + _dim_ay;
      real ocp_left_state_control[_dim_y];
      multiplyAndCopyVectorTo(p_left_state_control_scaled, ocp_left_state_control, _p_scaling_y, _dim_y);
      real ocp_state_control[_dim_xu]; //ocp states and controls at the middle point of the _p_mesh interval
      computeTpzCenter(p_left_state_control_scaled, p_right_state_control_scaled, _p_scaling_y, ocp_state_control,
                       _dim_xu);
      real ocp_state_control_derivative[_dim_xu]; //ocp states and controls derivatives at the middle point of the _p_mesh interval
      computeTpzDerivative(p_left_state_control_scaled, p_right_state_control_scaled, _p_scaling_y,
                           ocp_state_control_derivative, 1 / d_zeta, _dim_xu);

      real ocp_algebraic_state_control[_dim_axu];
      multiplyAndCopyVectorTo(p_left_state_control_scaled + _dim_y, ocp_algebraic_state_control, _p_scaling_ay,
                              _dim_ay);

#ifdef MAVERICK_DEBUG
      if ( _th_affinity.size() == 1 ) {
          MAVERICK_DEBUG_ASSERT( p_current_y_scaled == nlp_y + getNlpYPtrIndexForInterval(current_mesh_interval), "Pointer to Y variables exceeds limit.")


          real * p_exp = constraints_out + getNlpConstraintsPtrIndexForInterval(current_mesh_interval);
          MAVERICK_DEBUG_ASSERT( p_current_constraint == p_exp, "Wrong evaluation of NLP constraints pointer. Current " << p_current_constraint << ", expected " << p_exp << ".\n" )
      }
#endif

      evalConstraints(ocp_left_state_control,
                      ocp_state_control,
                      ocp_state_control_derivative,
                      ocp_algebraic_state_control,
                      ocp_params,
                      zeta,
                      d_zeta,
                      d_zeta_dual,
                      &p_current_constraint,
                      int_constraints_out);

      // update pointers
      p_current_y_scaled += _dim_y + _dim_ay;
    }
    // END LOOP OVER _p_mesh POINTS, MAIN PART

#ifdef MAVERICK_DEBUG
    if (_th_affinity.size()==1) {
        MAVERICK_DEBUG_ASSERT( p_current_y_scaled == nlp_y + getNlpParamPtrIndex() - _dim_y, "Not all states have been calculated. Current pointer " << p_current_y_scaled << ". Pointer to last p_mesh points parameter: " << nlp_y + getNlpParamPtrIndex() - _dim_y << "." )
    }
#endif
  } catch (...) {
    *exc_ptr = std::current_exception();
  }
}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void MidpointOcp2NlpSinglePhase::evalConstraints(real const ocp_left_state_control[],
                                                 real const ocp_state_control[],
                                                 real const ocp_state_control_derivative[],
                                                 real const ocp_algebraic_state_control[],
                                                 real const ocp_params[],
                                                 real const zeta,
                                                 real const d_zeta,
                                                 real const d_zeta_dual,
                                                 real **const p_p_current_constraint,
                                                 real int_constraints[]) const {

  real *p_current_constraint = *p_p_current_constraint;
  _ocp_problem.foEqns(_i_phase,
                      ocp_state_control,
                      ocp_state_control_derivative,
                      ocp_algebraic_state_control,
                      ocp_params,
                      zeta,
                      p_current_constraint);

  //scale
  real p_inv_scaling_fo_eqns[_dim_fo];
  if (_multiply_foeqns_by_dz)
    multiplyAndCopyVectorTo(_p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, d_zeta, _dim_fo);
  else
    copyVectorTo(_p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, _dim_fo);
  multiplyVectorBy(p_current_constraint, p_inv_scaling_fo_eqns, _dim_fo);
  p_current_constraint += _dim_fo;

  _ocp_problem.pathConstraints(_i_phase,
                               ocp_state_control,
                               ocp_state_control_derivative,
                               ocp_algebraic_state_control,
                               ocp_params,
                               zeta,
                               p_current_constraint);

  //scale
  real p_inv_scaling_path_constr[_dim_pc];
  if (_multiply_path_constr_by_dz)
    multiplyAndCopyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, d_zeta, _dim_pc);
  else
    copyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, _dim_pc);
  multiplyVectorBy(p_current_constraint, p_inv_scaling_path_constr, _dim_pc);
  p_current_constraint += _dim_pc;

  _ocp_problem.pointConstraints(_i_phase,
                                ocp_left_state_control,
                                ocp_params,
                                zeta,
                                p_current_constraint);

  //scale
  real p_inv_scaling_point_constr[_dim_poc];
  if (_multiply_point_constr_by_dz)
    multiplyAndCopyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, d_zeta_dual, _dim_poc);
  else
    copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);
  multiplyVectorBy(p_current_constraint, p_inv_scaling_point_constr, _dim_poc);
  p_current_constraint += _dim_poc;
  *p_p_current_constraint = p_current_constraint;

  // integral constraints
  real int_constr_values[_dim_ic];
  _ocp_problem.intConstraints(_i_phase,
                              ocp_state_control,
                              ocp_state_control_derivative,
                              ocp_algebraic_state_control,
                              ocp_params,
                              zeta,
                              int_constr_values);
  //scale
  multiplyVectorBy(int_constr_values, _p_inv_scaling_int_constr, _dim_ic);

  multiplyAndSumVectorTo(int_constr_values, int_constraints, d_zeta, _dim_ic);

}
