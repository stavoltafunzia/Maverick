#include "MidpointOcp2NlpSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"

#ifdef __linux__

#endif

using namespace Maverick;
using namespace std;

integer MidpointOcp2NlpSinglePhase::calculateNlpQuantities(real const nlp_y[], integer const n_y, real const lambda[],
                                                           real const lambda_0,
                                                           real *target_out,
                                                           real target_jac_out[], integer const n_t_j,
                                                           real constraints_out[], integer const n_c,
                                                           real constraints_jac_out[], integer const n_c_jac,
                                                           real hessian_out[], integer const n_hess
) const {
  MAVERICK_DEBUG_ASSERT(n_y == getNlpSize(),
                        "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong length of NLP data. Passed " << n_y
                                                                                                                << ", actual "
                                                                                                                << getNlpSize());

  bool const evaluate_target = target_out != nullptr;
  bool const evaluate_target_j = target_jac_out != nullptr;
  bool const evaluate_constraints = constraints_out != nullptr;
  bool const evaluate_constraints_j = constraints_jac_out != nullptr;
  bool const evaluate_hessian = hessian_out != nullptr;

#ifdef MAVERICK_DEBUG

  if (evaluate_target_j) {
      MAVERICK_ASSERT( n_t_j == getNlpTargetGradientNnz(), std::string(__FILE__) + ": " + std::to_string(__LINE__) + ": wrong target gradient size.\n")
      // MAVERICK_ASSERT( target_jac_out != nullptr, "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: asked to evaluate target jacobian with a nullptr target jacobian pointer.")
  }
  if (evaluate_constraints) {
      MAVERICK_ASSERT( n_c == getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: wrong number of constraints requested. Requested " << n_c << " actual " << getNlpConstraintsSize() << ".")
      // MAVERICK_ASSERT( constraints_out != nullptr, "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: asked to evaluate constraints with a nullptr constraints pointer.")
  }

  if (evaluate_constraints_j) {
      MAVERICK_ASSERT( n_c_jac == getNlpConstraintsJacobianNnz(), "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: wrong length of constraints jacobian requested. Requested " << n_c_jac << " actual " << getNlpConstraintsJacobianNnz() << ".")
      // MAVERICK_ASSERT( constraints_jac_out != nullptr, "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: asked to evaluate constraints jacobian with a nullptr constraints jacobian pointer.")
  }

  if (evaluate_hessian) {
      MAVERICK_ASSERT( n_hess == getNlpHessianNnz(), "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: wrong length of target hessian requested. Requested " << n_hess << " actual " << getNlpHessianNnz() << ".")
      MAVERICK_ASSERT( lambda != nullptr, "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: asked to evaluate hessian with a nullptr pointer to lambda.")
      // MAVERICK_ASSERT( hessian_out != nullptr, "MidpointOcp2NlpSinglePhase::calculateNlpQuantities: asked to evaluate hessian with a nullptr hessian pointer.")
  }

#endif

  //CREATE DATA ARRAYS FOR EACH THREAD
  real const *t_nlp_y[_actual_num_threads];

  for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
    integer current_mesh_interval = _thread_jobs[i_thread].start_mesh_interval;
    t_nlp_y[i_thread] = nlp_y + getNlpYPtrIndexForInterval(current_mesh_interval);
  }

  real ocp_params[_dim_p];
  multiplyAndCopyVectorTo(nlp_y + getNlpParamPtrIndex(), ocp_params, _p_scaling_r, _dim_p);
  real const *t_ocp_params = ocp_params;

  //target
  real *t_target_out[_actual_num_threads];
  if (evaluate_target) {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++)
      t_target_out[i_thread] = new real(0);
  } else {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++)
      t_target_out[i_thread] = nullptr;
  }

  //target jacobian
  real *t_lagrange_target_jac_out[_actual_num_threads];
  real *t_lagrange_target_j_r_out[_actual_num_threads];
  real *t_lagrange_target_j_y_last_out[_actual_num_threads];
  integer lagrange_jac_p_nnz = 0;
  if (evaluate_target_j) {
    lagrange_jac_p_nnz = _ocp_problem.lagrangeJacPNnz(_i_phase);

    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      integer current_mesh_interval = _thread_jobs[i_thread].start_mesh_interval;
      t_lagrange_target_jac_out[i_thread] =
          target_jac_out + getNlpTargetJacobianPtrIndexForInterval(current_mesh_interval);
      writeRealToVector(t_lagrange_target_jac_out[i_thread], 0,
                        _lagrange_target_j_y_nnz); //write zero only on first elements is necessary
      t_lagrange_target_j_r_out[i_thread] = new real[lagrange_jac_p_nnz];
      writeRealToVector(t_lagrange_target_j_r_out[i_thread], 0,
                        lagrange_jac_p_nnz); // write zero to the lagrange p gradient
      t_lagrange_target_j_y_last_out[i_thread] = new real[_lagrange_target_j_y_nnz];
    }
  } else {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      t_lagrange_target_jac_out[i_thread] = nullptr;
      t_lagrange_target_j_r_out[i_thread] = nullptr;
      t_lagrange_target_j_y_last_out[i_thread] = nullptr;
    }
  }

  //constraints
  real *t_constraints_out[_actual_num_threads];
  real *t_int_constraints_out[_actual_num_threads];
  if (evaluate_constraints) {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      integer current_mesh_interval = _thread_jobs[i_thread].start_mesh_interval;
      t_constraints_out[i_thread] = constraints_out + getNlpConstraintsPtrIndexForInterval(current_mesh_interval);
      t_int_constraints_out[i_thread] = new real[_dim_ic];
      writeRealToVector(t_int_constraints_out[i_thread], 0, _dim_ic);
    }
  } else {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      t_constraints_out[i_thread] = nullptr;
      t_int_constraints_out[i_thread] = nullptr;
    }
  }

  //constraints jacobian
  real *t_constraints_jac_out[_actual_num_threads];
  real *t_int_constraints_jac_y_out[_actual_num_threads];
  real *t_int_constraints_jac_p_out[_actual_num_threads];
  real *t_int_constraints_jac_y_last_out[_actual_num_threads];
  integer int_constr_j_p = 0;
  if (evaluate_constraints_j) {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      integer current_mesh_interval = _thread_jobs[i_thread].start_mesh_interval;
      t_constraints_jac_out[i_thread] =
          constraints_jac_out + getNlpConstraintsJacobainPtrIndexForInterval(current_mesh_interval);
      t_int_constraints_jac_y_out[i_thread] =
          constraints_jac_out + getNlpConstraintsJacobainPtrIndexForIntConstr(current_mesh_interval);
      writeRealToVector(t_int_constraints_jac_y_out[i_thread], 0, _int_constr_j_y_nnz);
      int_constr_j_p = _ocp_problem.intConstraintsJacPNnz(_i_phase);
      t_int_constraints_jac_p_out[i_thread] = new real[int_constr_j_p];
      writeRealToVector(t_int_constraints_jac_p_out[i_thread], 0, int_constr_j_p);
      t_int_constraints_jac_y_last_out[i_thread] = new real[_int_constr_j_y_nnz];
    }
  } else {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      t_constraints_jac_out[i_thread] = nullptr;
      t_int_constraints_jac_y_out[i_thread] = nullptr;
      t_int_constraints_jac_p_out[i_thread] = nullptr;
      t_int_constraints_jac_y_last_out[i_thread] = nullptr;
    }
  }

  //hessian
  real *t_hessian_out[_actual_num_threads];
  real *t_hessian_last_column_out[_actual_num_threads];
  SparseMatrix *t_hess_p_p_lower_mat[_actual_num_threads];
  real const *t_lambda[_actual_num_threads];
  real *integral_constraint_lambda_scaled;
  if (evaluate_hessian) {
    integral_constraint_lambda_scaled = new real[_dim_ic];
    multiplyAndCopyVectorTo(lambda + getNlpConstraintsPtrIndexForIntConstr(), integral_constraint_lambda_scaled,
                            _p_inv_scaling_int_constr, _dim_ic);

    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      integer current_mesh_interval = _thread_jobs[i_thread].start_mesh_interval;
      t_hessian_out[i_thread] = hessian_out + getNlpHessianPtrIndexForLeftBlock(current_mesh_interval);
      writeRealToVector(t_hessian_out[i_thread], 0, getNlpHessianLeftColumnBlockNnz());
      t_hessian_last_column_out[i_thread] = new real[getNlpHessianRightColumnBlockNnz()];
      //            writeRealToVector(t_hessian_last_column_out[i_thread], 4, getNlpHessianMiddleColumnBlockNnz() );
      t_hess_p_p_lower_mat[i_thread] = new SparseMatrix(_dim_p, _dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
      (t_hess_p_p_lower_mat[i_thread])->reserve( _hess_p_p_lower_mat_nnz );
#endif
      t_lambda[i_thread] = lambda + _dim_q * current_mesh_interval;
    }
  } else {
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      t_hessian_out[i_thread] = nullptr;
      t_hessian_last_column_out[i_thread] = nullptr;
      t_hess_p_p_lower_mat[i_thread] = nullptr;
      t_lambda[i_thread] = nullptr;
    }
  }

  //    now run over the entire _p_mesh intervals
  integer const eigen_num_threads = Eigen::nbThreads();  // save number of threads for eigen
  Eigen::setNbThreads(1); // the function evaluation is already parallel

  for (u_integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
    ThreadJob const &th_job = _thread_jobs[i_thread];
    th_job.job = std::bind(&MidpointOcp2NlpSinglePhase::calculateNlpQuantitiesBetweenMeshPoints, this,
                           th_job.start_mesh_interval, th_job.end_mesh_interval,
                           t_nlp_y[i_thread], t_ocp_params,
                           t_target_out[i_thread],
                           t_lagrange_target_jac_out[i_thread], t_lagrange_target_j_r_out[i_thread],
                           t_lagrange_target_j_y_last_out[i_thread],
                           t_constraints_out[i_thread], t_int_constraints_out[i_thread],
                           t_constraints_jac_out[i_thread], t_int_constraints_jac_y_out[i_thread],
                           t_int_constraints_jac_p_out[i_thread], t_int_constraints_jac_y_last_out[i_thread],
                           t_hessian_out[i_thread], t_hessian_last_column_out[i_thread], t_hess_p_p_lower_mat[i_thread],
                           t_lambda[i_thread], integral_constraint_lambda_scaled, lambda_0,
                           th_job.exc_ptr
    );
    //nullptr);
    // threads[i_thread] = new std::thread(&MidpointOcp2NlpSinglePhase::calculateNlpQuantitiesBetweenMeshPoints, this,
    //                                     _thread_mesh_intervals[i_thread], _thread_mesh_intervals[i_thread+1],
    //                                     t_nlp_y[i_thread], t_ocp_params,
    //                                     t_target_out[i_thread],
    //                                     t_lagrange_target_jac_out[i_thread], t_lagrange_target_j_r_out[i_thread],  t_lagrange_target_j_y_last_out[i_thread],
    //                                     t_constraints_out[i_thread], t_int_constraints_out[i_thread],
    //                                     t_constraints_jac_out[i_thread], t_int_constraints_jac_y_out[i_thread], t_int_constraints_jac_p_out[i_thread], t_int_constraints_jac_y_last_out[i_thread],
    //                                     t_hessian_out[i_thread], t_hessian_last_column_out[i_thread], t_hess_p_p_lower_mat[i_thread],
    //                                     t_lambda[i_thread], integral_constraint_lambda_scaled, lambda_0,
    //                                     &exc_ptrs[i_thread]
    //                                     );
    {
      unique_lock<mutex> lock(*(th_job.job_mutex));
      th_job.job_todo = true;
    }
    th_job.cond_var->notify_one();
  }

  real const final_zeta = _p_mesh->getZeta(_p_mesh->getNumberOfIntervals());
  real const initial_zeta = _p_mesh->getZeta(0);
  real const *const p_initial_state_control_scaled = nlp_y;
  real const *const p_final_state_control_scaled = nlp_y + getNlpYPtrIndexForInterval(_p_mesh->getNumberOfIntervals());
  real ocp_initial_state_control[_dim_y];
  multiplyAndCopyVectorTo(p_initial_state_control_scaled, ocp_initial_state_control, _p_scaling_y, _dim_y);
  real ocp_final_state_control[_dim_y];
  multiplyAndCopyVectorTo(p_final_state_control_scaled, ocp_final_state_control, _p_scaling_y, _dim_y);

  // while all thread are running evaluate mayer target
  real *p_mayer_target = nullptr;
  if (evaluate_target) {
    p_mayer_target = new real;
    _ocp_problem.mayer(_i_phase,
                       ocp_initial_state_control,
                       ocp_final_state_control,
                       ocp_params,
                       *p_mayer_target);
    *p_mayer_target *= _inv_scaling_target;

  }

  // while all thread are running evaluate mayer target jacobian
  real *p_mayer_j_xup = nullptr;

  if (evaluate_target_j) {
    integer mayer_j_xu_init_nnz = _ocp_problem.mayerJacXuInitNnz(_i_phase);
    integer mayer_j_xu_fin_nnz = _ocp_problem.mayerJacXuFinNnz(_i_phase);
    integer mayer_j_p_nnz = _ocp_problem.mayerJacPNnz(_i_phase);

    p_mayer_j_xup = new real[mayer_j_xu_init_nnz + mayer_j_xu_fin_nnz + mayer_j_p_nnz];

    _ocp_problem.mayerJac(_i_phase,
                          ocp_initial_state_control,
                          ocp_final_state_control,
                          ocp_params,
                          p_mayer_j_xup,
                          p_mayer_j_xup + mayer_j_xu_init_nnz,
                          p_mayer_j_xup + mayer_j_xu_init_nnz + mayer_j_xu_fin_nnz);

    // now scale the mayer target
    multiplyVectorBy(p_mayer_j_xup, _p_scale_factor_mayer_j_yi, mayer_j_xu_init_nnz);
    multiplyVectorBy(p_mayer_j_xup + mayer_j_xu_init_nnz, _p_scale_factor_mayer_j_yf, mayer_j_xu_fin_nnz);
    multiplyVectorBy(p_mayer_j_xup + mayer_j_xu_init_nnz + mayer_j_xu_fin_nnz, _p_scale_factor_mayer_j_p,
                     mayer_j_p_nnz);
  }

  // while all threads are running evaluate last point constraint and boundary conditions
  if (evaluate_constraints) {

    real *p_current_constraint = constraints_out + getNlpConstraintsPtrIndexForLastPointConstr();

    _ocp_problem.pointConstraints(_i_phase,
                                  ocp_final_state_control,
                                  ocp_params,
                                  final_zeta,
                                  p_current_constraint);
    //scale
    real p_inv_scaling_point_constr[_dim_poc];
    if (_multiply_point_constr_by_dz)
      multiplyAndCopyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr,
                              _p_mesh->getDzDual(_p_mesh->getNumberOfDiscretisationPoints() - 1), _dim_poc);
    else
      copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);

    multiplyVectorBy(p_current_constraint, p_inv_scaling_point_constr, _dim_poc);
    p_current_constraint += _dim_poc;

    _ocp_problem.boundaryConditions(_i_phase,
                                    ocp_initial_state_control,
                                    ocp_final_state_control,
                                    ocp_params,
                                    initial_zeta,
                                    final_zeta,
                                    p_current_constraint);
    //scale
    multiplyVectorBy(p_current_constraint, _p_inv_scaling_bcs, _dim_bc);
    p_current_constraint += _dim_bc;

#ifdef MAVERICK_DEBUG
    if ( _th_affinity.size() == 1 ) {
        p_current_constraint += _dim_ic;
        MAVERICK_ASSERT( p_current_constraint == constraints_out + getNlpConstraintsSize(), "Wrong final pointer for constraints. Current pointer " << p_current_constraint << ". Pointer to last p_mesh points parameter: " << constraints_out + getNlpConstraintsSize() << "." )
    }
#endif
  }

  // while all threads are running evaluate last point constraints and boundary conditions jacobian
  if (evaluate_constraints_j) {
    real *p_current_constraints_j = constraints_jac_out + getNlpConstraintsJacobainPtrIndexForLastPointConstraints();

    // last point constraints j xu and p
    integer nnz = _ocp_problem.pointConstraintsJacXuNnz(_i_phase);
    _ocp_problem.pointConstraintsJac(_i_phase, ocp_final_state_control, ocp_params, final_zeta,
                                     p_current_constraints_j,
                                     p_current_constraints_j + nnz);
    //scale
    multiplyVectorBy(p_current_constraints_j, _p_scale_factor_point_constr_j_y, nnz);
    p_current_constraints_j += nnz;
    nnz = _ocp_problem.pointConstraintsJacPNnz(_i_phase);
    multiplyVectorBy(p_current_constraints_j, _p_scale_factor_point_constr_j_p, nnz);
    p_current_constraints_j += nnz;

#ifdef MAVERICK_DEBUG
    if ( _th_affinity.size() == 1 ) {
        MAVERICK_ASSERT(p_current_constraints_j == constraints_jac_out + getNlpConstraintsJacobainPtrIndexForF() ,
                        "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong constraints jacobian pointer for matrix F. Current pointer: " << p_current_constraints_j << ", expected: " << constraints_jac_out + getNlpConstraintsJacobainPtrIndexForF() << ".")
    }
#endif
    calculateNlpConstraintsJacobianMatrixF(ocp_initial_state_control,
                                           ocp_final_state_control,
                                           ocp_params,
                                           initial_zeta,
                                           final_zeta,
                                           p_current_constraints_j);

    p_current_constraints_j += getNlpConstraintsJacobainMatrixFNnz();
#ifdef MAVERICK_DEBUG
    if ( _th_affinity.size() == 1 ) {
        p_current_constraints_j += (_int_constr_j_y_nnz + _int_constr_j_ay_nnz) * _p_mesh->getNumberOfIntervals() + _int_constr_j_y_nnz + _ocp_problem.intConstraintsJacPNnz(_i_phase);
        MAVERICK_ASSERT(p_current_constraints_j == constraints_jac_out + getNlpConstraintsJacobianNnz() ,
                        "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong constraints jacobian final pointer.")
    }
#endif
  }

  //while all threads are running evaluate the hessian for the first and last block
  SparseMatrix last_point_constr_hess_y_y_lower_mat(_dim_xu, _dim_xu);
  SparseMatrix last_point_constr_hess_y_p_mat(_dim_p, _dim_xu);
  SparseMatrix last_point_constr_hess_p_p_lower_mat(_dim_p, _dim_p);
  SparseMatrix bcs_hess_xu_init_xu_init_lower_mat(_dim_xu, _dim_xu);
  SparseMatrix bcs_hess_xu_init_xu_fin_mat(_dim_xu, _dim_xu);
  SparseMatrix bcs_hess_xu_init_p_mat(_dim_p, _dim_xu);
  SparseMatrix bcs_hess_xu_fin_xu_fin_lower_mat(_dim_xu, _dim_xu);
  SparseMatrix bcs_hess_xu_fin_p_mat(_dim_p, _dim_xu);
  SparseMatrix bcs_hess_p_p_lower_mat(_dim_p, _dim_p);
  SparseMatrix mayer_hess_xu_init_xu_init_lower_mat(_dim_xu, _dim_xu);
  SparseMatrix mayer_hess_xu_init_xu_fin_mat(_dim_xu, _dim_xu);
  SparseMatrix mayer_hess_xu_init_p_mat(_dim_p, _dim_xu);
  SparseMatrix mayer_hess_xu_fin_xu_fin_lower_mat(_dim_xu, _dim_xu);
  SparseMatrix mayer_hess_xu_fin_p_mat(_dim_p, _dim_xu);
  SparseMatrix mayer_hess_p_p_lower_mat(_dim_p, _dim_p);

  if (evaluate_hessian) {
    real const *p_current_constraint_lambda = lambda + getNlpConstraintsPtrIndexForLastPointConstr();

    calculateHessianLastConstrBcsMayer(ocp_initial_state_control, ocp_final_state_control, ocp_params,
                                       initial_zeta,
                                       final_zeta,
                                       _p_mesh->getDzDual(_p_mesh->getNumberOfDiscretisationPoints() - 1),
                                       lambda, p_current_constraint_lambda, lambda_0,
                                       last_point_constr_hess_y_y_lower_mat,
                                       last_point_constr_hess_y_p_mat,
                                       last_point_constr_hess_p_p_lower_mat,
                                       bcs_hess_xu_init_xu_init_lower_mat,
                                       bcs_hess_xu_init_xu_fin_mat,
                                       bcs_hess_xu_init_p_mat,
                                       bcs_hess_xu_fin_xu_fin_lower_mat,
                                       bcs_hess_xu_fin_p_mat,
                                       bcs_hess_p_p_lower_mat,
                                       mayer_hess_xu_init_xu_init_lower_mat,
                                       mayer_hess_xu_init_xu_fin_mat,
                                       mayer_hess_xu_init_p_mat,
                                       mayer_hess_xu_fin_xu_fin_lower_mat,
                                       mayer_hess_xu_fin_p_mat,
                                       mayer_hess_p_p_lower_mat);

    // scale last point constr
    real *values = (real *) last_point_constr_hess_y_y_lower_mat.valuePtr();
    integer counter = 0;
    for (integer k = 0; k < last_point_constr_hess_y_y_lower_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(last_point_constr_hess_y_y_lower_mat, k); it; ++it) {
        values[counter] *= _p_scaling_y[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) last_point_constr_hess_y_p_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < last_point_constr_hess_y_p_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(last_point_constr_hess_y_p_mat, k); it; ++it) {
        values[counter] *= _p_scaling_r[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    // scale boundary conditions
    values = (real *) bcs_hess_xu_init_xu_init_lower_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < bcs_hess_xu_init_xu_init_lower_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(bcs_hess_xu_init_xu_init_lower_mat, k); it; ++it) {
        values[counter] *= _p_scaling_y[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) bcs_hess_xu_init_xu_fin_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < bcs_hess_xu_init_xu_fin_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(bcs_hess_xu_init_xu_fin_mat, k); it; ++it) {
        values[counter] *= _p_scaling_y[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) bcs_hess_xu_init_p_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < bcs_hess_xu_init_p_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(bcs_hess_xu_init_p_mat, k); it; ++it) {
        values[counter] *= _p_scaling_r[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) bcs_hess_xu_fin_xu_fin_lower_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < bcs_hess_xu_fin_xu_fin_lower_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(bcs_hess_xu_fin_xu_fin_lower_mat, k); it; ++it) {
        values[counter] *= _p_scaling_y[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) bcs_hess_xu_fin_p_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < bcs_hess_xu_fin_p_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(bcs_hess_xu_fin_p_mat, k); it; ++it) {
        values[counter] *= _p_scaling_r[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    // scale mayer
    values = (real *) mayer_hess_xu_init_xu_init_lower_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < mayer_hess_xu_init_xu_init_lower_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(mayer_hess_xu_init_xu_init_lower_mat, k); it; ++it) {
        values[counter] *= _p_scaling_y[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) mayer_hess_xu_init_xu_fin_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < mayer_hess_xu_init_xu_fin_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(mayer_hess_xu_init_xu_fin_mat, k); it; ++it) {
        values[counter] *= _p_scaling_y[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) mayer_hess_xu_init_p_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < mayer_hess_xu_init_p_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(mayer_hess_xu_init_p_mat, k); it; ++it) {
        values[counter] *= _p_scaling_r[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) mayer_hess_xu_fin_xu_fin_lower_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < mayer_hess_xu_fin_xu_fin_lower_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(mayer_hess_xu_fin_xu_fin_lower_mat, k); it; ++it) {
        values[counter] *= _p_scaling_y[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

    values = (real *) mayer_hess_xu_fin_p_mat.valuePtr();
    counter = 0;
    for (integer k = 0; k < mayer_hess_xu_fin_p_mat.outerSize(); ++k)
      for (SparseMatrix::InnerIterator it(mayer_hess_xu_fin_p_mat, k); it; ++it) {
        values[counter] *= _p_scaling_r[it.row()] * _p_scaling_y[it.col()];
        counter++;
      }

  }

  // wait for threads to finish calculus
  for (u_integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
    ThreadJob &th_job = _thread_jobs[i_thread];
    // wait for the thread to finish
    {
      auto check_pred = [&th_job]() -> bool { return !(th_job.job_todo); };
      unique_lock<mutex> lock(*(th_job.job_mutex));
      th_job.cond_var->wait(lock, check_pred);
    }

    // rethrow exception if needed
    if (th_job.exc_ptr)
      std::rethrow_exception(th_job.exc_ptr);
  }
  Eigen::setNbThreads(eigen_num_threads); // restore eigen parallel


  // now glue all together

  //target
  if (evaluate_target) {
    *target_out = *p_mayer_target;
    delete p_mayer_target;

    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      *target_out += *t_target_out[i_thread];
      delete t_target_out[i_thread];
    }
  }

  if (evaluate_target_j) {
    //write zero to the necessary part of the target

    real *const p_target_j_r_out = target_jac_out + getNlpTargetGradientNnz() - _target_j_p_nnz;
    writeRealToVector(p_target_j_r_out, 0, _target_j_p_nnz);

    {
      real *const p_target_j_y_last_out =
          target_jac_out + getNlpTargetJacobianPtrIndexForInterval(_p_mesh->getNumberOfIntervals());
      writeRealToVector(p_target_j_y_last_out, 0, _lagrange_target_j_y_nnz);
    }

    real lagrange_jac_p[lagrange_jac_p_nnz];
    writeRealToVector(lagrange_jac_p, 0, lagrange_jac_p_nnz);

    // first sum the lagrange target from all the threads
    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {

      integer current_mesh_interval_end = _thread_jobs[i_thread].end_mesh_interval;
      real *lagrange_target_j_y_end =
          target_jac_out + getNlpTargetJacobianPtrIndexForInterval(current_mesh_interval_end);
      sumVectorTo(t_lagrange_target_j_y_last_out[i_thread], lagrange_target_j_y_end, _lagrange_target_j_y_nnz);
      delete[] t_lagrange_target_j_y_last_out[i_thread];

      sumVectorTo(t_lagrange_target_j_r_out[i_thread], lagrange_jac_p, lagrange_jac_p_nnz);
      delete[] t_lagrange_target_j_r_out[i_thread];
    }

    // if the gradient is dense, then the lagrange target j_p must be de-sparsified
    if (_is_gradient_dense) {
      integer p_lagrange_target_j_p_pattern[lagrange_jac_p_nnz];
      _ocp_problem.lagrangeJacPPattern(_i_phase, p_lagrange_target_j_p_pattern);
      for (integer ip = 0; ip < lagrange_jac_p_nnz; ip++) {
        p_target_j_r_out[p_lagrange_target_j_p_pattern[ip]] = lagrange_jac_p[ip];
      }
    } else {
      copyVectorTo(lagrange_jac_p, p_target_j_r_out, lagrange_jac_p_nnz);
    }

    integer mayer_j_xu_init_nnz = _ocp_problem.mayerJacXuInitNnz(_i_phase);
    integer mayer_j_xu_fin_nnz = _ocp_problem.mayerJacXuFinNnz(_i_phase);
    integer mayer_j_p_nnz = _ocp_problem.mayerJacPNnz(_i_phase);

    // collect mayer target
    integer mayer_j_xu_init_outer_start[2] = {0, mayer_j_xu_init_nnz};
    integer mayer_j_xu_init_pattern[mayer_j_xu_init_nnz];
    _ocp_problem.mayerJacXuInitPattern(_i_phase, mayer_j_xu_init_pattern);
    Eigen::Map<SparseMatrix> mayer_j_xu_init_mat(_dim_xu, 1,
                                                 mayer_j_xu_init_nnz,
                                                 mayer_j_xu_init_outer_start,
                                                 mayer_j_xu_init_pattern,
                                                 p_mayer_j_xup,
                                                 0);

    integer mayer_j_xu_fin_outer_start[2] = {0, mayer_j_xu_fin_nnz};
    integer mayer_j_xu_fin_pattern[mayer_j_xu_fin_nnz];
    _ocp_problem.mayerJacXuFinPattern(_i_phase, mayer_j_xu_fin_pattern);
    Eigen::Map<SparseMatrix> mayer_j_xu_fin_mat(_dim_xu, 1,
                                                mayer_j_xu_fin_nnz,
                                                mayer_j_xu_fin_outer_start,
                                                mayer_j_xu_fin_pattern,
                                                p_mayer_j_xup + mayer_j_xu_init_nnz,
                                                0);

    integer mayer_j_p_outer_start[2] = {0, mayer_j_p_nnz};
    integer mayer_j_p_pattern[mayer_j_p_nnz];
    _ocp_problem.mayerJacPPattern(_i_phase, mayer_j_p_pattern);
    Eigen::Map<SparseMatrix> mayer_j_p_mat(_dim_p, 1,
                                           mayer_j_p_nnz,
                                           mayer_j_p_outer_start,
                                           mayer_j_p_pattern,
                                           p_mayer_j_xup + mayer_j_xu_init_nnz + mayer_j_xu_fin_nnz,
                                           0);

    //collect lagange target
    real *lagrange_j_xi = target_jac_out + _target_j_xi_nnz - _lagrange_target_j_y_nnz;
    real *lagrange_j_p = p_target_j_r_out;
    real *lagrange_j_xf = lagrange_j_p - _target_j_xf_nnz;


    integer lagrange_j_y_outer_start[2] = {0, _lagrange_target_j_y_nnz};
    Eigen::Map<SparseMatrix> lagrange_j_y_init_mat(_dim_xu, 1,
                                                   _lagrange_target_j_y_nnz,
                                                   lagrange_j_y_outer_start,
                                                   _p_lagrange_target_j_y_pattern,
                                                   lagrange_j_xi,
                                                   0);

    Eigen::Map<SparseMatrix> lagrange_j_y_fin_mat(_dim_xu, 1,
                                                  _lagrange_target_j_y_nnz,
                                                  lagrange_j_y_outer_start,
                                                  _p_lagrange_target_j_y_pattern,
                                                  lagrange_j_xf,
                                                  0);

    integer lagrange_j_p_nnz = _ocp_problem.lagrangeJacPNnz(_i_phase);
    integer lagrange_j_p_outer_start[2] = {0, lagrange_j_p_nnz};
    integer lagrange_j_p_pattern[lagrange_j_p_nnz];
    _ocp_problem.lagrangeJacPPattern(_i_phase, lagrange_j_p_pattern);
    Eigen::Map<SparseMatrix> lagrange_j_p_mat(_dim_p, 1,
                                              lagrange_j_p_nnz,
                                              lagrange_j_p_outer_start,
                                              lagrange_j_p_pattern,
                                              lagrange_j_p,
                                              0);

    SparseMatrix target_j_xi_mat = lagrange_j_y_init_mat + mayer_j_xu_init_mat;
    SparseMatrix target_j_xf_mat = lagrange_j_y_fin_mat + mayer_j_xu_fin_mat;
    SparseMatrix target_j_p_mat = lagrange_j_p_mat + mayer_j_p_mat;

    if (_is_gradient_dense) {

      for (SparseMatrix::InnerIterator it(target_j_xi_mat, 0); it; ++it)
        target_jac_out[it.row()] = it.value();

      for (SparseMatrix::InnerIterator it(target_j_xf_mat, 0); it; ++it)
        lagrange_j_xf[it.row()] = it.value();

      writeRealToVector(lagrange_j_p, 0, _target_j_p_nnz);
      for (SparseMatrix::InnerIterator it(target_j_p_mat, 0); it; ++it)
        lagrange_j_p[it.row()] = it.value();

    } else {

      copyVectorTo(target_j_xi_mat.valuePtr(), target_jac_out, _target_j_xi_nnz);

      copyVectorTo(target_j_xf_mat.valuePtr(), lagrange_j_xf, _target_j_xf_nnz);

      copyVectorTo(target_j_p_mat.valuePtr(), lagrange_j_p, _target_j_p_nnz);
    }

    delete[] p_mayer_j_xup;

  }

  //constraints
  if (evaluate_constraints) {
    real *const p_int_constraints_out = constraints_out + getNlpConstraintsPtrIndexForIntConstr();
    writeRealToVector(p_int_constraints_out, 0, _dim_ic);

    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      sumVectorTo(t_int_constraints_out[i_thread], p_int_constraints_out, _dim_ic);
      delete[] t_int_constraints_out[i_thread];
    }

  }

  // constraints jacobian
  if (evaluate_constraints_j) {
    //write zero to the _p integral constraints jacobian
    real *const p_int_constr_j_p = constraints_jac_out + getNlpConstraintsJacobainPtrIndexForIntConstrParams();
    writeRealToVector(p_int_constr_j_p, 0, int_constr_j_p);

    {
      //write zero to the last _y integral constraints jacobian
      real *int_constraints_jac_y_last_out =
          constraints_jac_out + getNlpConstraintsJacobainPtrIndexForIntConstr(_p_mesh->getNumberOfIntervals());
      writeRealToVector(int_constraints_jac_y_last_out, 0, _int_constr_j_y_nnz);
    }

    for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
      integer current_mesh_interval_end = _thread_jobs[i_thread].end_mesh_interval;
      real *current_int_constraints_jac_y_end =
          constraints_jac_out + getNlpConstraintsJacobainPtrIndexForIntConstr(current_mesh_interval_end);

      sumVectorTo(t_int_constraints_jac_y_last_out[i_thread], current_int_constraints_jac_y_end, _int_constr_j_y_nnz);
      delete[] t_int_constraints_jac_y_last_out[i_thread];

      sumVectorTo(t_int_constraints_jac_p_out[i_thread], p_int_constr_j_p, int_constr_j_p);
      delete[] t_int_constraints_jac_p_out[i_thread];
    }
  }

  //hessian
  if (evaluate_hessian) {
    delete[] integral_constraint_lambda_scaled;

    SparseMatrix hess_p_p_lower_mat(_dim_p, _dim_p);
    hess_p_p_lower_mat.reserve(_hess_p_p_lower_mat_nnz);

    //write zero to the necessary parts
    {
      real *hessian_last_column = hessian_out + getNlpHessianPtrIndexForLeftBlock(_p_mesh->getNumberOfIntervals());
      writeRealToVector(hessian_last_column, 0, getNlpHessianRightColumnBlockNnz());
    }
    //        cout << "\nhess1:\n";
    //        for (integer i=0; i<getNlpHessianNnz(); i++)
    //            cout << hessian_out[i] << " ";
    //        cout << "\n\n";
    {
      //first glue the hessian written by all threads
      for (integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
        integer current_mesh_interval_end = _thread_jobs[i_thread].end_mesh_interval;;

        // first add the y_y block
        real *current_hessian_middle_column_end =
            hessian_out + getNlpHessianPtrIndexForLeftBlock(current_mesh_interval_end);
        sumVectorTo(t_hessian_last_column_out[i_thread], current_hessian_middle_column_end, _hess_y_y_lower_mat_nnz);

        // now add the y_p block
        current_hessian_middle_column_end += _hess_y_y_lower_mat_nnz;
        if (i_thread != (_actual_num_threads - 1)) {
          current_hessian_middle_column_end += _hess_y_ay_mat_nnz + _hess_yleft_yright_mat_nnz;
        }
        sumVectorTo(t_hessian_last_column_out[i_thread] + _hess_y_y_lower_mat_nnz, current_hessian_middle_column_end,
                    _hess_y_p_mat_nnz);
        //                cout << "column: ";
        //                for (integer i=0; i<nnz; i++) {
        //                    cout << t_hessian_last_column_out[i_thread][i] << " ";
        //                }
        //                cout << "\n";
        delete[] t_hessian_last_column_out[i_thread];

        hess_p_p_lower_mat += *(t_hess_p_p_lower_mat[i_thread]);
        delete t_hess_p_p_lower_mat[i_thread];
      }
    }
    //        cout << "\nhess2:\n";
    //        for (integer i=0; i<getNlpHessianNnz(); i++)
    //            cout << hessian_out[i] << " ";
    //        cout << "\n\n";

    // update the hessian with the values of the boundary conditions, mayer target and last point constraints
    {
      //first, the first hessian column
      real *p_hessia_written = hessian_out + getNlpHessianPtrIndexForLeftBlock(0);
      real *p_current_hessian = hessian_out;

      // matrix of the first y_y hessian block
      Eigen::Map<SparseMatrix> hess_y_y_lower_mat(_dim_xu, _dim_xu, _hess_y_y_lower_mat_nnz,
                                                  _p_hess_y_y_lower_mat_outer_starts,
                                                  _p_hess_y_y_lower_mat_rows,
                                                  p_hessia_written,
                                                  0);

      // copy the first y_ay and yleft_yright hessian blocks
      integer const yay_left_right_nnz = _hess_y_ay_mat_nnz + _hess_yleft_yright_mat_nnz;
      real hess_yay_left_right_mat_copy[yay_left_right_nnz];
      copyVectorTo(p_hessia_written + _hess_y_y_lower_mat_nnz, hess_yay_left_right_mat_copy, yay_left_right_nnz);

      // matrix of the first y_p block
      Eigen::Map<SparseMatrix> hess_y_p_mat(_dim_p, _dim_xu, _hess_y_p_mat_nnz,
                                            _p_hess_y_p_mat_outer_starts,
                                            _p_hess_y_p_mat_rows,
                                            p_hessia_written + _hess_y_y_lower_mat_nnz + yay_left_right_nnz,
                                            0);
      SparseMatrix hess_first_y_y_lower_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
      hess_first_y_y_lower_mat.reserve( _hess_first_y_y_lower_mat_nnz );
#endif
      hess_first_y_y_lower_mat =
          hess_y_y_lower_mat + bcs_hess_xu_init_xu_init_lower_mat + mayer_hess_xu_init_xu_init_lower_mat;
      copyVectorTo((real *) hess_first_y_y_lower_mat.valuePtr(), p_current_hessian, _hess_first_y_y_lower_mat_nnz);
      p_current_hessian += _hess_first_y_y_lower_mat_nnz;

      copyVectorTo(hess_yay_left_right_mat_copy, p_current_hessian, yay_left_right_nnz);
      p_current_hessian += yay_left_right_nnz;

      SparseMatrix hess_xi_xf_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
      hess_xi_xf_mat.reserve( _hess_xi_xf_mat_nnz );
#endif
      hess_xi_xf_mat = bcs_hess_xu_init_xu_fin_mat + mayer_hess_xu_init_xu_fin_mat;
      copyVectorTo((real *) hess_xi_xf_mat.valuePtr(), p_current_hessian, _hess_xi_xf_mat_nnz);
      p_current_hessian += _hess_xi_xf_mat_nnz;

      SparseMatrix hess_first_y_p_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
      hess_first_y_p_mat.reserve( _hess_first_y_p_mat_nnz );
#endif
      hess_first_y_p_mat = hess_y_p_mat + bcs_hess_xu_init_p_mat + mayer_hess_xu_init_p_mat;
      copyVectorTo((real *) hess_first_y_p_mat.valuePtr(), p_current_hessian, _hess_first_y_p_mat_nnz);
      p_current_hessian += _hess_first_y_p_mat_nnz;

      //            cout << _hess_first_14_lower_mat_nnz << "\n";
      //            cout << _hess_2_mat_nnz << "\n";
      //            cout << _hess_xi_xf_mat_nnz << "\n";
      //            cout << _hess_first_35_mat_nnz << "\n";

#ifdef  MAVERICK_DEBUG
      real * p_expected = hessian_out + getNlpHessianFirstColumnBlockNnz();
      MAVERICK_ASSERT( p_current_hessian == p_expected, "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong pointer for hessian first column end.\n")
#endif

    }
    {
      //then, the last hessian column
      real *p_hessia_written =
          hessian_out + getNlpHessianNnz() - _hess_p_p_lower_mat_nnz - getNlpHessianLastColumnBlockNnz();
#ifdef  MAVERICK_DEBUG
      real * p_expected = hessian_out + getNlpHessianFirstColumnBlockNnz() + getNlpHessianLeftColumnBlockNnz() * (_p_mesh->getNumberOfIntervals() - 1 ) + + getNlpHessianCentreColumnBlockNnz() * _p_mesh->getNumberOfIntervals();
      MAVERICK_ASSERT( p_hessia_written == p_expected, "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong pointer for hessian last column.\n")
#endif
      Eigen::Map<SparseMatrix> hess_y_y_lower_mat(_dim_xu, _dim_xu, _hess_y_y_lower_mat_nnz,
                                                  _p_hess_y_y_lower_mat_outer_starts,
                                                  _p_hess_y_y_lower_mat_rows,
                                                  p_hessia_written,
                                                  0);

      Eigen::Map<SparseMatrix> hess_y_p_mat(_dim_p, _dim_xu, _hess_y_p_mat_nnz,
                                            _p_hess_y_p_mat_outer_starts,
                                            _p_hess_y_p_mat_rows,
                                            p_hessia_written + _hess_y_y_lower_mat_nnz,
                                            0);
      SparseMatrix hess_last_y_y_lower_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
      hess_last_y_y_lower_mat.reserve( _hess_last_y_y_lower_mat_nnz );
#endif
      hess_last_y_y_lower_mat =
          hess_y_y_lower_mat + bcs_hess_xu_fin_xu_fin_lower_mat + mayer_hess_xu_fin_xu_fin_lower_mat +
          last_point_constr_hess_y_y_lower_mat;
      real *p_current_hessian = p_hessia_written;
      copyVectorTo((real *) hess_last_y_y_lower_mat.valuePtr(), p_current_hessian, _hess_last_y_y_lower_mat_nnz);
      p_current_hessian += _hess_last_y_y_lower_mat_nnz;

      SparseMatrix hess_last_y_p_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
      hess_last_y_p_mat.reserve( _hess_last_y_p_mat_nnz );
#endif
      hess_last_y_p_mat =
          hess_y_p_mat + bcs_hess_xu_fin_p_mat + mayer_hess_xu_fin_p_mat + last_point_constr_hess_y_p_mat;
      copyVectorTo((real *) hess_last_y_p_mat.valuePtr(), p_current_hessian, _hess_last_y_p_mat_nnz);
      p_current_hessian += _hess_last_y_p_mat_nnz;
    }
    {
      //then, the p-p block, and we have to scale it
      real *p_current_hessian = hessian_out + getNlpHessianPtrIndexForPP();
      SparseMatrix hess_p_p_final_lower_mat(_dim_p, _dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
      hess_p_p_final_lower_mat.reserve(_hess_p_p_lower_mat_nnz);
#endif
      hess_p_p_final_lower_mat = hess_p_p_lower_mat + last_point_constr_hess_p_p_lower_mat + bcs_hess_p_p_lower_mat +
                                 mayer_hess_p_p_lower_mat;

      real *values = (real *) hess_p_p_final_lower_mat.valuePtr();
      integer counter = 0;
      for (integer k = 0; k < hess_p_p_final_lower_mat.outerSize(); ++k)
        for (SparseMatrix::InnerIterator it(hess_p_p_final_lower_mat, k); it; ++it) {
          p_current_hessian[counter] = values[counter] * _p_scaling_r[it.row()] * _p_scaling_r[it.col()];
          counter++;
        }

      // copyVectorTo( (real *) hess_p_p_final_lower_mat.valuePtr(), p_current_hessian, _hess_p_p_lower_mat_nnz);
      p_current_hessian += _hess_p_p_lower_mat_nnz;

#ifdef MAVERICK_DEBUG
      MAVERICK_ASSERT( p_current_hessian == hessian_out + getNlpHessianNnz(), "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong final hessian pointer.\n")
#endif
    }
  }
  return 0;
}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________



void MidpointOcp2NlpSinglePhase::calculateNlpQuantitiesBetweenMeshPoints(integer const first_mesh_point,
                                                                         integer const last_mesh_point,
                                                                         real const nlp_y[], real const ocp_params[],
                                                                         real *lagrange_target_out,
                                                                         real lagrange_target_jac_y_out[],
                                                                         real lagrange_target_j_r_out[],
                                                                         real lagrange_target_j_y_last_out[],
                                                                         real constraints_out[],
                                                                         real int_constraints_out[],
                                                                         real constraints_jac_out[],
                                                                         real int_constraints_jac_y_out[],
                                                                         real int_constraints_jac_p_out[],
                                                                         real int_constraints_jac_y_last_out[],
                                                                         real hessian_out[],
                                                                         real hessian_last_column_out[],
                                                                         SparseMatrix *hess_p_p_lower_mat,
                                                                         real const lambda[],
                                                                         real const integral_constraint_lambda_scaled[],
                                                                         real const lambda_0,
                                                                         std::exception_ptr &exc_ptr
) const {
  try {
    //    lagrange_target_jac_y_out=nullptr;
    //    lagrange_target_j_r_out=nullptr;
    //    lagrange_target_j_y_last_out=nullptr;

    // Y and P pointers
    real const *p_current_y_scaled = nlp_y;
    //real const * p_current_y = nlp_y + min_mesh_point;

    // OUTPUT STREAMS
    //target

    // target jacobian
    real *p_current_target_j_y = lagrange_target_jac_y_out; //pointer to first element of the jacobian of the target w.r.t the NLP variables of the current _p_mesh interval

    // constraints
    real *p_current_constraint = constraints_out;

    // constraints jacobian
    real *p_current_constraints_j = constraints_jac_out;
    real *p_current_int_constr_j_y = int_constraints_jac_y_out;

    // hessian
    real *p_current_hessian = hessian_out;
    real const *p_current_constraint_lambda = lambda;

    // now run over the entire _p_mesh intervals
    // BEGIN LOOP OVER MESH POINTS, MAIN PART
    for (integer current_mesh_interval = first_mesh_point;
         current_mesh_interval < last_mesh_point; current_mesh_interval++) {

      //zeta
      real const zeta_left = _p_mesh->getZetaLeft(current_mesh_interval);
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
      computeTpzDerivative(p_left_state_control_scaled, p_left_state_control_scaled + _dim_ay + _dim_y, _p_scaling_y,
                           ocp_state_control_derivative, 1 / d_zeta, _dim_xu);
      // algebraic states and controls
      real ocp_algebraic_state_control[_dim_axu];
      multiplyAndCopyVectorTo(p_left_state_control_scaled + _dim_y, ocp_algebraic_state_control, _p_scaling_ay,
                              _dim_axu);

#ifdef MAVERICK_DEBUG
      if ( _th_affinity.size() == 1 ) {
          MAVERICK_ASSERT( p_current_y_scaled == nlp_y + getNlpYPtrIndexForInterval(current_mesh_interval), "Pointer to Y variables is not correct.")

          if (constraints_out) {
              real * p_exp = constraints_out + getNlpConstraintsPtrIndexForInterval(current_mesh_interval);
              MAVERICK_ASSERT( p_current_constraint == p_exp, "Wrong evaluation of NLP constraints pointer. Current " << p_current_constraint << ", expected " << p_exp << ".\n" )
          }
          if (constraints_jac_out)
              MAVERICK_ASSERT( p_current_constraints_j == constraints_jac_out + getNlpConstraintsJacobainPtrIndexForInterval(current_mesh_interval) , "Wrong evaluation of NLP constraints jacobian pointer." )
      }
#endif

      if (lagrange_target_out) {
        real tmp_target;
        _ocp_problem.lagrange(_i_phase,
                              ocp_state_control,
                              ocp_state_control_derivative,
                              ocp_algebraic_state_control,
                              ocp_params,
                              zeta,
                              tmp_target);
        *lagrange_target_out += tmp_target * _inv_scaling_target * d_zeta;
      }

      if (lagrange_target_jac_y_out) {

        real *p_lagrange_target_j_right = p_current_target_j_y + _lagrange_target_j_y_nnz + _lagrange_target_j_ay_nnz;
        if (current_mesh_interval == (last_mesh_point - 1)) {
          p_lagrange_target_j_right = lagrange_target_j_y_last_out;
        }

        calculateLagrangeTargetGradient(ocp_state_control,
                                        ocp_state_control_derivative,
                                        ocp_algebraic_state_control,
                                        ocp_params,
                                        zeta,
                                        d_zeta,
                                        p_current_target_j_y,
                                        p_lagrange_target_j_right,
                                        lagrange_target_j_r_out);

        p_current_target_j_y += _dim_y + _dim_ay;
      }

      if (constraints_out) {
        //            cout << "mesh point " << current_mesh_interval << "\n";
        //            cout << "Y left: ";
        //            for (integer i=0; i < _dim_y; i++) {
        //                cout << *(ocp_left_state_control+i) << "\t";
        //            }
        //            cout << "\n";
        //            cout << "Y right: ";
        //            for (integer i=0; i < _dim_y; i++) {
        //                cout << p_left_state_control_scaled[i+_dim_y]*_p_scaling_y[i] << "\t";
        //            }
        //            cout << "\n";
        evalConstraints(ocp_left_state_control,
                        ocp_state_control,
                        ocp_state_control_derivative,
                        ocp_algebraic_state_control,
                        ocp_params,
                        zeta_left,
                        zeta,
                        d_zeta,
                        d_zeta_dual,
                        &p_current_constraint,
                        int_constraints_out);
      }

      if (constraints_jac_out) {
        calculateNlpConstraintsJacobianMatrixY(ocp_left_state_control,
                                               ocp_state_control,
                                               ocp_state_control_derivative,
                                               ocp_algebraic_state_control,
                                               ocp_params,
                                               zeta_left,
                                               zeta,
                                               d_zeta,
                                               d_zeta_dual,
                                               p_current_constraints_j);

        p_current_constraints_j += getNlpConstraintsJacobainMatrixYNnz();

        real *p_current_int_constr_j_y_right = p_current_int_constr_j_y + _int_constr_j_y_nnz + _int_constr_j_ay_nnz;
        if (current_mesh_interval == last_mesh_point - 1)
          p_current_int_constr_j_y_right = int_constraints_jac_y_last_out;

        calculateNlpConstraintsJacobianMatrixI(ocp_state_control,
                                               ocp_state_control_derivative,
                                               ocp_algebraic_state_control,
                                               ocp_params,
                                               zeta,
                                               d_zeta,
                                               p_current_int_constr_j_y,
                                               p_current_int_constr_j_y_right,
                                               int_constraints_jac_p_out);

        p_current_int_constr_j_y += _int_constr_j_y_nnz + _int_constr_j_ay_nnz;
      }

      if (hessian_out) {
        real *p_hessian_y_y_right_block =
            p_current_hessian + getNlpHessianLeftColumnBlockNnz() + getNlpHessianCentreColumnBlockNnz();
        real *p_hessian_y_p_right_block =
            p_hessian_y_y_right_block + _hess_y_y_lower_mat_nnz + _hess_y_ay_mat_nnz + _hess_yleft_yright_mat_nnz;

        //if we are at the last mesh point, then store hessian to a different memeory location to avoirace conditions
        if (current_mesh_interval == (last_mesh_point - 1)) {
          p_hessian_y_y_right_block = hessian_last_column_out;
          p_hessian_y_p_right_block = p_hessian_y_y_right_block + _hess_y_y_lower_mat_nnz;
        }

        calculateHessianBlockAtMeshMiddle(ocp_left_state_control,
                                          ocp_state_control,
                                          ocp_state_control_derivative,
                                          ocp_algebraic_state_control,
                                          ocp_params,
                                          zeta_left,
                                          zeta,
                                          d_zeta,
                                          d_zeta_dual,
                                          lambda_0,
                                          p_current_constraint_lambda,
                                          integral_constraint_lambda_scaled,
                                          p_current_hessian,
                                          p_hessian_y_y_right_block,
                                          p_hessian_y_p_right_block,
                                          *hess_p_p_lower_mat);

        //            cout << "middle column: ";
        //            for (integer i=0; i<getNlpHessianMiddleColumnBlockNnz(); i++) {
        //                cout << p_current_hessian[i] << "\t";
        //            }
        //            cout << "\nright column: ";
        //            for (integer i=0; i< _hess_14_lower_mat_nnz; i++) {
        //                cout << p_hessian_fourth_block[i] << "\t";
        //            }
        //            cout << "\nright column end: ";
        //            for (integer i=0; i< _hess_35_mat_nnz; i++) {
        //                cout << p_hessian_fifth_block[i] << "\t";
        //            }
        //            cout << "\n\n\n";
        //            cout << "\nhessian: ";
        //            for (integer i=0; i< getNlpHessianNnz(); i++) {
        //                cout << hessian_out[i] << "\t";
        //            }
        //            cout << "\n\n\n";

        p_current_constraint_lambda += _dim_q;
        p_current_hessian += getNlpHessianLeftColumnBlockNnz() + getNlpHessianCentreColumnBlockNnz();
      }

      // update pointers
      p_current_y_scaled += _dim_y + _dim_ay;
    }
    // END LOOP OVER _p_mesh POINTS, MAIN PART

#ifdef MAVERICK_DEBUG
    if (_th_affinity.size()==1) {
        MAVERICK_DEBUG_ASSERT( p_current_y_scaled == nlp_y + getNlpParamPtrIndex() - _dim_y, "Not all states have been calculated. Position difference is " << p_current_y_scaled - ( nlp_y + getNlpParamPtrIndex() - _dim_y ) )

        if (lagrange_target_jac_y_out) {
            real * p_exp = lagrange_target_jac_y_out + ((_lagrange_target_j_y_nnz + _lagrange_target_j_ay_nnz)) * (_p_mesh->getNumberOfIntervals());
            MAVERICK_DEBUG_ASSERT( p_current_target_j_y == p_exp ,
                                  std::string(__FILE__) + ": " + std::to_string(__LINE__) +  "Wrong final target gradient pointer. Difference is: " << p_current_target_j_y - p_exp)
        }
    }
#endif
  } catch (...) {
    exc_ptr = std::current_exception();
  }
}
