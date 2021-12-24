#include "RK1Ocp2NlpSinglePhase.hh"
#include "../MaverickFunctions.hh"

using namespace Maverick;


//+------------------------------------------------------------------------+
//|  _                       _                         _ _            _    |
//| | |_ __ _ _ __ __ _  ___| |_    __ _ _ __ __ _  __| (_) ___ _ __ | |_  |
//| | __/ _` | '__/ _` |/ _ \ __|  / _` | '__/ _` |/ _` | |/ _ \ '_ \| __| |
//| | || (_| | | | (_| |  __/ |_  | (_| | | | (_| | (_| | |  __/ | | | |_  |
//|  \__\__,_|_|  \__, |\___|\__|  \__, |_|  \__,_|\__,_|_|\___|_| |_|\__| |
//|               |___/            |___/                                   |
//+------------------------------------------------------------------------+



void RK1Ocp2NlpSinglePhase::setupForNlpTargetGradient() {

  //collect quantities from ocp

  //mayer
  integer nnz = _ocp_problem.mayerJacXuInitNnz(_i_phase);
  integer mayer_j_xu_init_pattern[nnz];
  _ocp_problem.mayerJacXuInitPattern(_i_phase, mayer_j_xu_init_pattern);
  _p_scale_factor_mayer_j_yi = new real[nnz];
  SparseMatrix mayer_j_xu_init_mat(_dim_xu, 1);
  for (integer i = 0; i < nnz; i++) {
    integer index = mayer_j_xu_init_pattern[i];
    mayer_j_xu_init_mat.insert(index, 0) = 1;
    _p_scale_factor_mayer_j_yi[i] = _inv_scaling_target * _p_scaling_y[index];
  }

  nnz = _ocp_problem.mayerJacXuFinNnz(_i_phase);
  integer mayer_j_xu_fin_pattern[nnz];
  _ocp_problem.mayerJacXuFinPattern(_i_phase, mayer_j_xu_fin_pattern);
  _p_scale_factor_mayer_j_yf = new real[nnz];
  SparseMatrix mayer_j_xu_fin_mat(_dim_xu, 1);
  for (integer i = 0; i < nnz; i++) {
    integer index = mayer_j_xu_fin_pattern[i];
    mayer_j_xu_fin_mat.insert(index, 0) = 1;
    _p_scale_factor_mayer_j_yf[i] = _inv_scaling_target * _p_scaling_y[index];
  }

  nnz = _ocp_problem.mayerJacPNnz(_i_phase);
  integer mayer_j_p_pattern[nnz];
  _ocp_problem.mayerJacPPattern(_i_phase, mayer_j_p_pattern);
  _p_scale_factor_mayer_j_p = new real[nnz];
  SparseMatrix mayer_j_p_mat(_dim_p, 1);
  for (integer i = 0; i < nnz; i++) {
    integer index = mayer_j_p_pattern[i];
    mayer_j_p_mat.insert(index, 0) = 1;
    _p_scale_factor_mayer_j_p[i] = _inv_scaling_target * _p_scaling_r[index];
  }

  //lagrange

  nnz = _ocp_problem.lagrangeJacXuNnz(_i_phase);
  integer lagrange_j_xu_pattern[nnz];
  _ocp_problem.lagrangeJacXuPattern(_i_phase, lagrange_j_xu_pattern);
  SparseMatrix lagrange_j_xu_mat(_dim_xu, 1);
  for (integer i = 0; i < nnz; i++) {
    lagrange_j_xu_mat.insert(lagrange_j_xu_pattern[i], 0) = 1;
  }

  nnz = _ocp_problem.lagrangeJacDxuNnz(_i_phase);
  integer lagrange_j_dxu_pattern[nnz];
  _ocp_problem.lagrangeJacDxuPattern(_i_phase, lagrange_j_dxu_pattern);
  SparseMatrix lagrange_j_dxu_mat(_dim_xu, 1);
  for (integer i = 0; i < nnz; i++) {
    lagrange_j_dxu_mat.insert(lagrange_j_dxu_pattern[i], 0) = 1;
  }

  nnz = _ocp_problem.lagrangeJacAxuNnz(_i_phase);
  integer lagrange_j_ay_pattern[nnz];
  _ocp_problem.lagrangeJacAxuPattern(_i_phase, lagrange_j_ay_pattern);
  _p_scale_factor_lagrange_target_j_ay = new real[nnz];
  SparseMatrix lagrange_j_ay_mat(_dim_axu, 1);
  for (integer i = 0; i < nnz; i++) {
    integer index = lagrange_j_ay_pattern[i];
    lagrange_j_ay_mat.insert(index, 0) = 1;
    _p_scale_factor_lagrange_target_j_ay[i] = _inv_scaling_target * _p_scaling_ay[index];
  }

  nnz = _ocp_problem.lagrangeJacPNnz(_i_phase);
  integer lagrange_j_p_pattern[nnz];
  _ocp_problem.lagrangeJacPPattern(_i_phase, lagrange_j_p_pattern);
  _p_scale_factor_lagrange_target_j_p = new real[nnz];
  SparseMatrix lagrange_j_p_mat(_dim_p, 1);
  for (integer i = 0; i < nnz; i++) {
    integer index = lagrange_j_p_pattern[i];
    lagrange_j_p_mat.insert(index, 0) = 1;
    _p_scale_factor_lagrange_target_j_p[i] = _inv_scaling_target * _p_scaling_r[index];
  }

  //calculate nlp quantities

  //lagrange_j_y
  SparseMatrix lagrange_j_y_mat(_dim_xu, 1);
  lagrange_j_y_mat = lagrange_j_xu_mat + lagrange_j_dxu_mat;
  lagrange_j_y_mat.makeCompressed();
  _p_scale_factor_lagrange_target_j_y = new real[lagrange_j_y_mat.nonZeros()];
  integer counter = 0;
  for (SparseMatrix::InnerIterator it(lagrange_j_y_mat, 0); it; ++it) {
    _p_scale_factor_lagrange_target_j_y[counter] = _inv_scaling_target * _p_scaling_y[it.row()];
    counter++;
  }

  if (_is_gradient_dense) { //ipopt case
    _lagrange_target_j_y_nnz = _dim_y;
    _p_lagrange_target_j_y_pattern = new integer[_lagrange_target_j_y_nnz];
    for (integer k = 0; k < _lagrange_target_j_y_nnz; ++k) {
      _p_lagrange_target_j_y_pattern[k] = k;
    }
    _lagrange_target_j_ay_nnz = _dim_ay;

    _target_j_xi_nnz = _dim_y;
    _p_target_j_yi_pattern = new integer[_target_j_xi_nnz];
    copyVectorTo(_p_lagrange_target_j_y_pattern, _p_target_j_yi_pattern, _target_j_xi_nnz);

    _target_j_xf_nnz = _dim_y;
    _p_target_j_yf_pattern = new integer[_target_j_xf_nnz];
    copyVectorTo(_p_lagrange_target_j_y_pattern, _p_target_j_yf_pattern, _target_j_xf_nnz);

    //target j p
    _target_j_p_nnz = _dim_p;
    _p_target_j_p_pattern = new integer[_target_j_p_nnz];
    for (integer k = 0; k < _target_j_p_nnz; ++k)
      _p_target_j_p_pattern[k] = k;

    // now define the gradient pattern
    integer nnz = getNlpTargetGradientNnz();
    _p_nlp_target_j_cols = new integer[nnz];
    for (integer i = 0; i < nnz; i++)
      _p_nlp_target_j_cols[i] = i;

  } else { //whorp case

    _lagrange_target_j_y_nnz = (integer) lagrange_j_y_mat.nonZeros();
    _p_lagrange_target_j_y_pattern = new integer[_lagrange_target_j_y_nnz];
    counter = 0;
    for (SparseMatrix::InnerIterator it(lagrange_j_y_mat, 0); it; ++it) {
      _p_lagrange_target_j_y_pattern[counter] = (integer) it.row();
      counter++;
    }

    //lagrange_j_ay
    _lagrange_target_j_ay_nnz = _ocp_problem.lagrangeJacAxuNnz(_i_phase);

    //target j xi
    SparseMatrix target_j_xi_mat(_dim_xu, 1);
    target_j_xi_mat = lagrange_j_y_mat + mayer_j_xu_init_mat;
    target_j_xi_mat.makeCompressed();
    _target_j_xi_nnz = (integer) target_j_xi_mat.nonZeros();
    _p_target_j_yi_pattern = new integer[_target_j_xi_nnz];
    counter = 0;
    for (SparseMatrix::InnerIterator it(target_j_xi_mat, 0); it; ++it) {
      _p_target_j_yi_pattern[counter] = (integer) it.row();
      counter++;
    }

    //target j xf
    SparseMatrix target_j_xf_mat(_dim_xu, 1);
    target_j_xf_mat = lagrange_j_y_mat + mayer_j_xu_fin_mat;
    target_j_xf_mat.makeCompressed();
    _target_j_xf_nnz = (integer) target_j_xf_mat.nonZeros();
    _p_target_j_yf_pattern = new integer[_target_j_xf_nnz];
    counter = 0;
    for (SparseMatrix::InnerIterator it(target_j_xf_mat, 0); it; ++it) {
      _p_target_j_yf_pattern[counter] = (integer) it.row();
      counter++;
    }

    //target j p
    SparseMatrix target_j_p_mat(_dim_p, 1);
    target_j_p_mat = lagrange_j_p_mat + mayer_j_p_mat;
    target_j_p_mat.makeCompressed();
    _target_j_p_nnz = (integer) target_j_p_mat.nonZeros();
    _p_target_j_p_pattern = new integer[_target_j_p_nnz];
    counter = 0;
    for (SparseMatrix::InnerIterator it(target_j_p_mat, 0); it; ++it) {
      _p_target_j_p_pattern[counter] = (integer) it.row();
      counter++;
    }

    // now define the gradient pattern
    integer nnz = getNlpTargetGradientNnz();
    _p_nlp_target_j_cols = new integer[nnz];
    integer overall_offset = 0;
    integer *p_current_col_index = _p_nlp_target_j_cols;

    //yi
    sumAndWriteVectorTo(_p_target_j_yi_pattern, p_current_col_index, overall_offset, _target_j_xi_nnz);
    p_current_col_index += _target_j_xi_nnz;
    overall_offset += _dim_y;

    for (integer i = 0; i < (_p_mesh->getNumberOfIntervals() - 1); i++) {
      sumAndWriteVectorTo(_p_lagrange_target_j_y_pattern, p_current_col_index, overall_offset,
                          _lagrange_target_j_y_nnz);
      p_current_col_index += _lagrange_target_j_y_nnz;
      overall_offset += _dim_y;

      sumAndWriteVectorTo(lagrange_j_ay_pattern, p_current_col_index, overall_offset, _lagrange_target_j_ay_nnz);
      p_current_col_index += _lagrange_target_j_ay_nnz;
      overall_offset += _dim_ay;
    }

    //yf
    sumAndWriteVectorTo(_p_target_j_yf_pattern, p_current_col_index, overall_offset, _target_j_xf_nnz);
    p_current_col_index += _target_j_xf_nnz;
    overall_offset += _dim_y;

    //p
    sumAndWriteVectorTo(_p_target_j_p_pattern, p_current_col_index, overall_offset, _target_j_p_nnz);
    p_current_col_index += _target_j_p_nnz;
    overall_offset += _dim_p;

    MAVERICK_DEBUG_ASSERT(overall_offset == getNlpSize(), std::string(__FILE__)
        +": " + std::to_string(__LINE__) + ": wrong final overall offset value.\n")
  }

}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void RK1Ocp2NlpSinglePhase::calculateLagrangeTargetGradient(
    ocpStateAtInterval const & ocp_state,
    real *p_in_gradient_left,
    real *p_in_gradient_right,
    real *p_in_gradient_p) const
{
  MAVERICK_RESTRICT real * p_gradient_left = p_in_gradient_left;
  MAVERICK_RESTRICT real * p_gradient_right = p_in_gradient_right;
  MAVERICK_RESTRICT real * p_gradient_p = p_in_gradient_p;

  integer lagrange_j_xu_nnz = _ocp_problem.lagrangeJacXuNnz(_i_phase);
  integer lagrange_j_dxu_nnz = _ocp_problem.lagrangeJacDxuNnz(_i_phase);
  integer lagrange_j_axu_nnz = _ocp_problem.lagrangeJacAxuNnz(_i_phase);
  integer lagrange_j_p_nnz = _ocp_problem.lagrangeJacPNnz(_i_phase);
  real tmp_target_j_xu[lagrange_j_xu_nnz];
  real tmp_target_j_dxu[lagrange_j_dxu_nnz];
  real tmp_target_j_axu[lagrange_j_axu_nnz];
  real tmp_target_j_p[lagrange_j_p_nnz];

  _ocp_problem.lagrangeJac(_i_phase,
                           ocp_state.state_control,
                           ocp_state.state_control_derivative,
                           ocp_state.algebraic_state_control,
                           ocp_state.parameters,
                           ocp_state.zeta_alpha,
                           tmp_target_j_xu,
                           tmp_target_j_dxu,
                           tmp_target_j_axu,
                           tmp_target_j_p);

  //
  integer lagrange_j_xu_outer_start[2] = {0, lagrange_j_xu_nnz};
  integer lagrange_j_xu_pattern[lagrange_j_xu_nnz];
  _ocp_problem.lagrangeJacXuPattern(_i_phase, lagrange_j_xu_pattern);
  Eigen::Map<SparseMatrix> lagrange_j_xu_mat(_dim_xu, 1,
                                             lagrange_j_xu_nnz,
                                             lagrange_j_xu_outer_start,
                                             lagrange_j_xu_pattern,
                                             tmp_target_j_xu,
                                             0);

  integer lagrange_j_dxu_outer_start[2] = {0, lagrange_j_dxu_nnz};
  integer lagrange_j_dxu_pattern[lagrange_j_dxu_nnz];
  _ocp_problem.lagrangeJacDxuPattern(_i_phase, lagrange_j_dxu_pattern);
  Eigen::Map<SparseMatrix> lagrange_j_dxu_mat(_dim_xu, 1,
                                              lagrange_j_dxu_nnz,
                                              lagrange_j_dxu_outer_start,
                                              lagrange_j_dxu_pattern,
                                              tmp_target_j_dxu,
                                              0);

  SparseMatrix lagrange_j_y_right_mat(_dim_xu, 1);
  lagrange_j_y_right_mat = lagrange_j_xu_mat * ocp_state.alpha * ocp_state.d_zeta + lagrange_j_dxu_mat;
  SparseMatrix lagrange_j_y_left_mat(_dim_xu, 1);
  lagrange_j_y_left_mat = lagrange_j_xu_mat * (1-ocp_state.alpha) * ocp_state.d_zeta - lagrange_j_dxu_mat;

  writeRealToVector(p_gradient_right, 0, _lagrange_target_j_y_nnz);

  if (_is_gradient_dense) {

    integer counter = 0;
    for (SparseMatrix::InnerIterator it(lagrange_j_y_left_mat, 0); it; ++it) {
      p_gradient_left[it.row()] += it.value() * _p_scale_factor_lagrange_target_j_y[counter];
      counter++;
    }

    counter = 0;
    for (SparseMatrix::InnerIterator it(lagrange_j_y_right_mat, 0); it; ++it) {
      p_gradient_right[it.row()] = it.value() * _p_scale_factor_lagrange_target_j_y[counter];
      counter++;
    }

    // j_ay
    integer lagrange_target_j_ay_pattern[lagrange_j_axu_nnz];
    _ocp_problem.lagrangeJacAxuPattern(_i_phase, lagrange_target_j_ay_pattern);
    writeRealToVector(p_gradient_left + _lagrange_target_j_y_nnz, 0, _dim_ay);
    for (integer i = 0; i < lagrange_j_axu_nnz; i++) {
      *(p_gradient_left + _lagrange_target_j_y_nnz + lagrange_target_j_ay_pattern[i]) =
          tmp_target_j_axu[i] * _p_scale_factor_lagrange_target_j_ay[i] * ocp_state.d_zeta;
    }

    // j_p
    integer lagrange_target_j_p_pattern[lagrange_j_p_nnz];
    _ocp_problem.lagrangeJacPPattern(_i_phase, lagrange_target_j_p_pattern);
    for (integer i = 0; i < lagrange_j_p_nnz; i++) {
      *(p_gradient_p + lagrange_target_j_p_pattern[i]) +=
          tmp_target_j_p[i] * _p_scale_factor_lagrange_target_j_p[i] * ocp_state.d_zeta;
    }

  } else {
    multiplyAndSumVectorTo(lagrange_j_y_left_mat.valuePtr(), p_gradient_left, _p_scale_factor_lagrange_target_j_y,
                           _lagrange_target_j_y_nnz);
    multiplyVectorBy(tmp_target_j_axu, ocp_state.d_zeta, lagrange_j_axu_nnz);
    multiplyAndCopyVectorTo(tmp_target_j_axu, p_gradient_left + _lagrange_target_j_y_nnz,
                            _p_scale_factor_lagrange_target_j_ay, lagrange_j_axu_nnz);
    multiplyAndCopyVectorTo(lagrange_j_y_right_mat.valuePtr(), p_gradient_right, _p_scale_factor_lagrange_target_j_y,
                            _lagrange_target_j_y_nnz);
    multiplyVectorBy(tmp_target_j_p, _p_scale_factor_lagrange_target_j_p, lagrange_j_p_nnz);
    multiplyAndSumVectorTo(tmp_target_j_p, p_gradient_p, ocp_state.d_zeta, lagrange_j_p_nnz);
  }

}
