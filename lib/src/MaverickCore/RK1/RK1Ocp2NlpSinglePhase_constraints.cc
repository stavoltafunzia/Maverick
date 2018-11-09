#include "RK1Ocp2NlpSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace Maverick;

void RK1Ocp2NlpSinglePhase::calculateNlpConstraintsBetweenMeshPoints(integer const first_mesh_point,
                                                                          integer const last_mesh_point,
                                                                          real const nlp_y[], real const ocp_params[],
                                                                          real in_constraints_out[],
                                                                          real in_int_constraints_out[],
                                                                          std::exception_ptr *exc_ptr
                                                                          ) const
{
  
  try {
    
    MAVERICK_RESTRICT real * constraints_out = in_constraints_out;
    MAVERICK_RESTRICT real * int_constraints_out = in_int_constraints_out;
    
    real buffer[_dim_y+_dim_y+_dim_y+_dim_ay];
    ocpStateAtInterval ocp_state;
    ocp_state.left_state_control = buffer;
    ocp_state.state_control = ocp_state.left_state_control + _dim_y;
    ocp_state.state_control_derivative = ocp_state.state_control + _dim_y;
    ocp_state.algebraic_state_control = ocp_state.state_control_derivative + _dim_y;
    ocp_state.parameters = ocp_params;
    ocp_state.alpha = _p_mesh->getAlpha();

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
      ocp_state.zeta_left = _p_mesh->getZetaLeft(current_mesh_interval);
      ocp_state.zeta_alpha = _p_mesh->getZetaAlpha(current_mesh_interval);
      ocp_state.d_zeta = _p_mesh->getDz(current_mesh_interval);
      ocp_state.d_zeta_average = _p_mesh->getDzAverageAtIndex(current_mesh_interval);
      
      
      //variables
      real const *p_left_state_control_scaled = p_current_y_scaled;
      real const *p_right_state_control_scaled = p_current_y_scaled + _dim_y + _dim_ay;
      multiplyAndCopyVectorTo(p_left_state_control_scaled, ocp_state.left_state_control, _p_scaling_y, _dim_y);
      computeTpzAlpha(ocp_state.alpha, p_left_state_control_scaled, p_right_state_control_scaled, _p_scaling_y, ocp_state.state_control, _dim_xu);
      computeTpzDerivative(p_left_state_control_scaled, p_right_state_control_scaled, _p_scaling_y,
                           ocp_state.state_control_derivative, 1 / ocp_state.d_zeta, _dim_xu);

      multiplyAndCopyVectorTo(p_left_state_control_scaled + _dim_y, ocp_state.algebraic_state_control, _p_scaling_ay,
                              _dim_ay);

#ifdef MAVERICK_DEBUG
      if ( _th_affinity.size() == 1 ) {
          MAVERICK_DEBUG_ASSERT( p_current_y_scaled == nlp_y + getNlpYPtrIndexForInterval(current_mesh_interval), "Pointer to Y variables exceeds limit.")


          real * p_exp = constraints_out + getNlpConstraintsPtrIndexForInterval(current_mesh_interval);
          MAVERICK_DEBUG_ASSERT( p_current_constraint == p_exp, "Wrong evaluation of NLP constraints pointer. Current " << p_current_constraint << ", expected " << p_exp << ".\n" )
      }
#endif

      evalConstraints(ocp_state,
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

void RK1Ocp2NlpSinglePhase::evalConstraints(ocpStateAtInterval const & ocp_state,
                                                 real **const p_p_current_constraint,
                                                 real in_int_constraints[]
                                                 ) const
{

  MAVERICK_RESTRICT real * p_current_constraint = *p_p_current_constraint;
  MAVERICK_RESTRICT real * int_constraints = in_int_constraints;
  
  _ocp_problem.foEqns(_i_phase,
                      ocp_state.state_control,
                      ocp_state.state_control_derivative,
                      ocp_state.algebraic_state_control,
                      ocp_state.parameters,
                      ocp_state.zeta_alpha,
                      p_current_constraint);

  //scale
  real p_inv_scaling_fo_eqns[_dim_fo];
  if (_multiply_foeqns_by_dz)
    multiplyAndCopyVectorTo(_p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, ocp_state.d_zeta, _dim_fo);
  else
    copyVectorTo(_p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, _dim_fo);
  multiplyVectorBy(p_current_constraint, p_inv_scaling_fo_eqns, _dim_fo);
  p_current_constraint += _dim_fo;

  _ocp_problem.pathConstraints(_i_phase,
                               ocp_state.state_control,
                               ocp_state.state_control_derivative,
                               ocp_state.algebraic_state_control,
                               ocp_state.parameters,
                               ocp_state.zeta_alpha,
                               p_current_constraint);

  //scale
  real p_inv_scaling_path_constr[_dim_pc];
  if (_multiply_path_constr_by_dz)
    multiplyAndCopyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, ocp_state.d_zeta, _dim_pc);
  else
    copyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, _dim_pc);
  multiplyVectorBy(p_current_constraint, p_inv_scaling_path_constr, _dim_pc);
  p_current_constraint += _dim_pc;

  _ocp_problem.pointConstraints(_i_phase,
                                ocp_state.left_state_control,
                                ocp_state.parameters,
                                ocp_state.zeta_left,
                                p_current_constraint);

  //scale
  real p_inv_scaling_point_constr[_dim_poc];
  if (_multiply_point_constr_by_dz)
    multiplyAndCopyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, ocp_state.d_zeta_average, _dim_poc);
  else
    copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);
  multiplyVectorBy(p_current_constraint, p_inv_scaling_point_constr, _dim_poc);
  p_current_constraint += _dim_poc;
  *p_p_current_constraint = p_current_constraint;

  // integral constraints
  real int_constr_values[_dim_ic];
  _ocp_problem.intConstraints(_i_phase,
                              ocp_state.state_control,
                              ocp_state.state_control_derivative,
                              ocp_state.algebraic_state_control,
                              ocp_state.parameters,
                              ocp_state.zeta_alpha,
                              int_constr_values);
  //scale
  multiplyVectorBy(int_constr_values, _p_inv_scaling_int_constr, _dim_ic);

  multiplyAndSumVectorTo(int_constr_values, int_constraints, ocp_state.d_zeta, _dim_ic);

}
