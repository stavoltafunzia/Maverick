/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_RK1_OCP2NLP_SINGLE_PHASE_HH
#define MAVERICK_RK1_OCP2NLP_SINGLE_PHASE_HH

#include "MaverickCore/Ocp2Nlp.hh"
#include "MaverickCore/RK1/RK1MeshSinglePhase.hh"
#include "MaverickCore/RK1/RK1OcpSolutionSinglePhase.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include <thread>
#include <condition_variable>

#define _dim_y _dim_xu
#define _dim_ay _dim_axu
#define _point_constr_j_y_nnz  _ocp_problem.pointConstraintsJacXuNnz(_i_phase)
#define _point_constr_j_p_nnz  _ocp_problem.pointConstraintsJacPNnz(_i_phase)
#define _fo_eqns_j_xu_nnz      _ocp_problem.foEqnsJacXuNnz(_i_phase)
#define _fo_eqns_j_dxu_nnz      _ocp_problem.foEqnsJacDxuNnz(_i_phase)
#define _fo_eqns_j_ay_nnz      _ocp_problem.foEqnsJacAxuNnz(_i_phase)
#define _fo_eqns_j_p_nnz       _ocp_problem.foEqnsJacPNnz(_i_phase)
#define _path_constr_j_xu_nnz   _ocp_problem.pathConstraintsJacXuNnz(_i_phase)
#define _path_constr_j_dxu_nnz   _ocp_problem.pathConstraintsJacDxuNnz(_i_phase)
#define _path_constr_j_ay_nnz _ocp_problem.pathConstraintsJacAxuNnz(_i_phase)
#define _path_constr_j_p_nnz   _ocp_problem.pathConstraintsJacPNnz(_i_phase)
#define _int_constr_j_xu_nnz   _ocp_problem.intConstraintsJacXuNnz(_i_phase)
#define _int_constr_j_dxu_nnz   _ocp_problem.intConstraintsJacDxuNnz(_i_phase)
#define _int_constr_j_ay_nnz   _ocp_problem.intConstraintsJacAxuNnz(_i_phase)
#define _int_constr_j_p_nnz   _ocp_problem.intConstraintsJacPNnz(_i_phase)

namespace Maverick {

  class RK1Ocp2NlpSinglePhase : public Ocp2Nlp {

    /*
     0) a p_mesh is formed by N+1 p_mesh points and N p_mesh intervals

     1) variables are stored into a single array in the following order:

     NLP_vars = (y0,y1,y2,y3,.....yN+1,yN+1,r) where p_mesh intervals go from 1 to N.
     y are the NLP variables at every p_mesh interval. yi = xi,ui (states, controls)
     r are the NLP constant parameters. r = p

     2) constraints are discretised in the following order

     NLP_constraints = (fo_eqns0,path_constr0,constr0,fo_eqns1,path_constr1,constr1,...,fo_eqnsN,path_constrN,constrN, constrN+1, boundary_conditions, integral_constraints )
     where fo_eqns are the first order equations, constr are the point constraints, path_constr are the path constraints
     q is the NLP constraints number at every mesh point ( q = dim_fo+dim_dc+dim_sc)

     note: from 1 and 2 the jacobian and hessian form is determined

     */

  public:

    RK1Ocp2NlpSinglePhase(MaverickOcp const &ocp_problem, Mesh const &mesh, integer const i_phase);

    virtual ~RK1Ocp2NlpSinglePhase();

    // public output

    virtual integer getNlpSize() const;

    virtual void getNlpBounds(real lower_bounds[], real upper_bounds[], integer const n) const;

    //target related methods

    virtual integer getNlpTargetGradientNnz() const;

    //constraint related methods

    virtual integer getNlpConstraintsSize() const;

    virtual void getNlpConstraintsBounds(real lower_bounds[], real upper_bounds[], integer const n_b) const;

    virtual integer getNlpConstraintsJacobianNnz() const;

    // hessian related

    virtual integer getNlpHessianNnz() const;

    // compute quantitities
    virtual integer
    calculateNlpQuantities(real const nlp_y[], integer const n_y, real const lambda[], real const lambda_0,
                           real *target_out,
                           real target_jac_out[], integer const n_t_j,
                           real constraints_out[], integer const n_c,
                           real constraints_jac_out[], integer const n_c_jac,
                           real hessian_out[], integer const n_hess
    ) const;

    // convert nlp to ocp solution without doing any scaling
    virtual std::unique_ptr<OcpSolution> translateNlp2OcpSolution(Nlp const &nlp) const;

    virtual std::unique_ptr<RK1OcpSolutionSinglePhase> translateNlp2RK1OcpSolution(Nlp const &nlp) const;

    // convert ocp guess ot solution to nlp without doing any scaling
    virtual Nlp translateOcpGuess2Nlp(OcpGuess const &ocp_guess) const;


    // scale and unscale nlp
    virtual void scaleNlp(Nlp &nlp, bool const unscale = false) const;

    virtual void convertNlp2OcpMultipliers(Nlp &nlp, bool const inverse) const;

    // set methods

    void setIsTargetLagrangeFromGuess(OcpGuess const &ocp_guess);

    void setIsTargetLagrangeFromGuess(Nlp const &nlp_guess);

    void setThreadsAffinity(threads_affinity const &th_affinity);

    threads_affinity getActualThreadsAffinityUsed(integer const i_phase) const;

  protected:
    
    struct ocpStateAtInterval {
      real * left_state_control = nullptr;
      real * state_control = nullptr;
      real * state_control_derivative = nullptr;
      real * algebraic_state_control = nullptr;
      real const * parameters = nullptr;
      real zeta_left = 0;
      real zeta_alpha = 0;
      real d_zeta = 0;
      real d_zeta_average = 0;
      real alpha = 0.5;
    };

    RK1MeshSinglePhase const *_p_mesh = nullptr;

    // ocp and nlp dimensions
    integer _i_phase = 0;
    integer _dim_xu = 0; //number of OCP states and controls
    integer _dim_axu = 0; //number of OCP algebraic states and controls
    integer _dim_x = 0; //number of OCP states
    integer _dim_ax = 0; //number of OCP algebraic states
    integer _dim_p = 0; //number of OCP parameters
    integer _dim_fo = 0; //number of OCP firs order equations
    integer _dim_poc = 0; //number of OCP point constraints
    integer _dim_pc = 0; //number of OCP path constraints
    integer _dim_ic = 0; //number of OCP integral constraints
    integer _dim_bc = 0; //number of OCP boundary conditions
    integer _dim_q = 0; //number of NLP constraints for every p_mesh points. constraints may depend on state derivatives too.

    // NLP variables pointers index
    inline integer getNlpYPtrIndexForInterval(integer const mesh_interval) const {
      return (_dim_y + _dim_ay) * mesh_interval;
    } // is not ok for last mesh point!!!
    inline integer getNlpParamPtrIndex() const { return _dim_y + (_dim_y + _dim_ay) * _p_mesh->getNumberOfIntervals(); }

    //Target jacobian related quantities
    integer _lagrange_target_j_y_nnz = 0;
    integer _lagrange_target_j_ay_nnz = 0;
    integer *_p_lagrange_target_j_y_pattern = nullptr;
    integer _target_j_xi_nnz = 0;
    integer _target_j_xf_nnz = 0;
    integer _target_j_p_nnz = 0;
    integer *_p_target_j_yi_pattern = nullptr;
    integer *_p_target_j_yf_pattern = nullptr;
    integer *_p_target_j_p_pattern = nullptr;

    inline integer getNlpTargetJacobianPtrIndexForInterval(integer const mesh_interval) const {
      return (_target_j_xi_nnz - _lagrange_target_j_y_nnz) +
             (_lagrange_target_j_y_nnz + _lagrange_target_j_ay_nnz) * mesh_interval;
    }

    void calculateLagrangeTargetGradient(ocpStateAtInterval const & ocp_state,
                                         real *p_gradient_left,
                                         real *p_gradient_right,
                                         real *p_gradient_p) const;

    void setupForNlpTargetGradient();

    //Constraints methods

    void calculateNlpConstraintsBetweenMeshPoints(integer const first_mesh_point, integer const last_mesh_point,
                                                  real const nlp_y[], real const ocp_params[],
                                                  real constraints_out[], real int_constraints_out[],
                                                  std::exception_ptr *exc_ptr
    ) const;

    void evalConstraints(ocpStateAtInterval const & ocp_state,
                         real **const p_p_current_constraint,
                         real int_constraints[]) const;

    inline integer getNlpConstraintsPtrIndexForInterval(integer const mesh_interval) const {
      return _dim_q * mesh_interval;
    }

    inline integer getNlpConstraintsPtrIndexForLastPointConstr() const {
      return getNlpConstraintsPtrIndexForInterval(_p_mesh->getNumberOfIntervals());
    }

    inline integer getNlpConstraintsPtrIndexForIntConstr() const {
      return getNlpConstraintsPtrIndexForLastPointConstr() + _dim_poc + _dim_bc;
    }

    inline integer getNlpConstraintsPtrIndexForBcs() const {
      return _dim_q * _p_mesh->getNumberOfIntervals() + _dim_poc;
    }

    //Constraint Jacobian related quantities
    integer *_p_fo_eqns_j_xu_outer_start = nullptr;
    integer *_p_fo_eqns_j_dxu_outer_start = nullptr;
    integer *_p_path_constr_j_xu_outer_start = nullptr;
    integer *_p_path_constr_j_dxu_outer_start = nullptr;
    integer *_p_int_constr_j_xu_outer_start = nullptr;
    integer *_p_int_constr_j_dxu_outer_start = nullptr;

    integer _fo_eqns_j_y_nnz = 0;
    integer _path_constr_j_y_nnz = 0;
    integer _int_constr_j_y_nnz = 0;

    inline integer getNlpConstraintsJacobainMatrixYNnz() const {
      return (_fo_eqns_j_y_nnz + _path_constr_j_y_nnz) * 2 + _point_constr_j_y_nnz + _fo_eqns_j_ay_nnz +
             _path_constr_j_ay_nnz + _fo_eqns_j_p_nnz + _path_constr_j_p_nnz + _point_constr_j_p_nnz;
    }

    inline integer getNlpConstraintsJacobainMatrixFNnz() const {
      return _ocp_problem.boundaryConditionsJacXuInitNnz(_i_phase) +
             _ocp_problem.boundaryConditionsJacXuFinNnz(_i_phase) + _ocp_problem.boundaryConditionsJacPNnz(_i_phase);
    }

    inline integer getNlpConstraintsJacobainPtrIndexForInterval(integer const mesh_interval) const {
      return getNlpConstraintsJacobainMatrixYNnz() * mesh_interval;
    }

    inline integer getNlpConstraintsJacobainPtrIndexForLastPointConstraints() const {
      return getNlpConstraintsJacobainPtrIndexForInterval(_p_mesh->getNumberOfIntervals());
    }

    inline integer getNlpConstraintsJacobainPtrIndexForF() const {
      return getNlpConstraintsJacobainPtrIndexForLastPointConstraints() + _point_constr_j_y_nnz + _point_constr_j_p_nnz;
    }

    virtual integer getNlpConstraintsJacobainPtrIndexForIntConstr(integer const mesh_interval) const {
      return getNlpConstraintsJacobainPtrIndexForF() + getNlpConstraintsJacobainMatrixFNnz() +
             mesh_interval * (_int_constr_j_y_nnz + _int_constr_j_ay_nnz);
    }

    virtual integer getNlpConstraintsJacobainPtrIndexForIntConstrParams() const {
      return getNlpConstraintsJacobainPtrIndexForIntConstr(_p_mesh->getNumberOfIntervals()) + _int_constr_j_y_nnz;
    }

    void setupForNlpConstraintsJacobianMatrixes();

    void calculateNlpConstraintsJacobianMatrixY(ocpStateAtInterval const & ocp_state,
                                                real values[]) const;

    void calculateNlpConstraintsJacobianMatrixI(ocpStateAtInterval const & ocp_state,
                                                real values_left[],
                                                real values_right[],
                                                real jac_p_values[]) const;

    void calculateNlpConstraintsJacobianMatrixF(real const ocp_initial_state_control[],
                                                real const ocp_final_state_control[],
                                                real const ocp_parameters[],
                                                real const initial_zeta,
                                                real const final_zeta,
                                                real values[]) const;

    void calculateNlpConstraintsJacobainPattern(integer const fo_eqns_j_y_rows[],
                                                integer const fo_eqns_j_y_cols[],
                                                integer const path_constr_j_y_rows[],
                                                integer const path_constr_j_y_cols[],
                                                integer const int_constr_j_y_rows[],
                                                integer const int_constr_j_y_cols[]);

    // Hessian related quantities
    integer *_p_lag_hess_xu_xu_outer_start = nullptr;
    integer *_p_lag_hess_xu_dxu_outer_start = nullptr;
    integer *_p_lag_hess_xu_axu_outer_start = nullptr;
    integer *_p_lag_hess_xu_p_outer_start = nullptr;
    integer *_p_lag_hess_dxu_dxu_outer_start = nullptr;
    integer *_p_lag_hess_dxu_axu_outer_start = nullptr;
    integer *_p_lag_hess_dxu_p_outer_start = nullptr;
    integer *_p_lag_hess_axu_axu_outer_start = nullptr;
    integer *_p_lag_hess_axu_p_outer_start = nullptr;
    integer *_p_lag_hess_p_p_outer_start = nullptr;

    integer *_p_fo_eqns_hess_xu_xu_outer_start = nullptr;
    integer *_p_fo_eqns_hess_xu_dxu_outer_start = nullptr;
    integer *_p_fo_eqns_hess_xu_axu_outer_start = nullptr;
    integer *_p_fo_eqns_hess_xu_p_outer_start = nullptr;
    integer *_p_fo_eqns_hess_dxu_dxu_outer_start = nullptr;
    integer *_p_fo_eqns_hess_dxu_axu_outer_start = nullptr;
    integer *_p_fo_eqns_hess_dxu_p_outer_start = nullptr;
    integer *_p_fo_eqns_hess_axu_axu_outer_start = nullptr;
    integer *_p_fo_eqns_hess_axu_p_outer_start = nullptr;
    integer *_p_fo_eqns_hess_p_p_outer_start = nullptr;

    integer *_p_path_constr_hess_xu_xu_outer_start = nullptr;
    integer *_p_path_constr_hess_xu_dxu_outer_start = nullptr;
    integer *_p_path_constr_hess_xu_axu_outer_start = nullptr;
    integer *_p_path_constr_hess_xu_p_outer_start = nullptr;
    integer *_p_path_constr_hess_dxu_dxu_outer_start = nullptr;
    integer *_p_path_constr_hess_dxu_axu_outer_start = nullptr;
    integer *_p_path_constr_hess_dxu_p_outer_start = nullptr;
    integer *_p_path_constr_hess_axu_axu_outer_start = nullptr;
    integer *_p_path_constr_hess_axu_p_outer_start = nullptr;
    integer *_p_path_constr_hess_p_p_outer_start = nullptr;

    integer *_p_int_constr_hess_xu_xu_outer_start = nullptr;
    integer *_p_int_constr_hess_xu_dxu_outer_start = nullptr;
    integer *_p_int_constr_hess_xu_axu_outer_start = nullptr;
    integer *_p_int_constr_hess_xu_p_outer_start = nullptr;
    integer *_p_int_constr_hess_dxu_dxu_outer_start = nullptr;
    integer *_p_int_constr_hess_dxu_axu_outer_start = nullptr;
    integer *_p_int_constr_hess_dxu_p_outer_start = nullptr;
    integer *_p_int_constr_hess_axu_axu_outer_start = nullptr;
    integer *_p_int_constr_hess_axu_p_outer_start = nullptr;
    integer *_p_int_constr_hess_p_p_outer_start = nullptr;

    integer *_p_point_constr_hess_xu_xu_outer_start = nullptr;
    integer *_p_point_constr_hess_xu_p_outer_start = nullptr;
    integer *_p_point_constr_hess_p_p_outer_start = nullptr;

    integer *_p_bcs_hess_xu_init_xu_init_outer_start = nullptr;
    integer *_p_bcs_hess_xu_init_xu_fin_outer_start = nullptr;
    integer *_p_bcs_hess_xu_init_p_outer_start = nullptr;
    integer *_p_bcs_hess_xu_fin_xu_fin_outer_start = nullptr;
    integer *_p_bcs_hess_xu_fin_p_outer_start = nullptr;
    integer *_p_bcs_hess_p_p_outer_start = nullptr;

    integer *_p_mayer_hess_xu_init_xu_init_outer_start = nullptr;
    integer *_p_mayer_hess_xu_init_xu_fin_outer_start = nullptr;
    integer *_p_mayer_hess_xu_init_p_outer_start = nullptr;
    integer *_p_mayer_hess_xu_fin_xu_fin_outer_start = nullptr;
    integer *_p_mayer_hess_xu_fin_p_outer_start = nullptr;
    integer *_p_mayer_hess_p_p_outer_start = nullptr;

    integer _hess_y_y_lower_mat_nnz = 0;
    integer *_p_hess_y_y_lower_mat_outer_starts = nullptr;
    integer *_p_hess_y_y_lower_mat_rows = nullptr;
    integer _hess_first_y_y_lower_mat_nnz = 0;
    integer _hess_last_y_y_lower_mat_nnz = 0;

    integer _hess_ay_ay_lower_mat_nnz = 0;
    integer _hess_y_ay_mat_nnz = 0;
    integer _hess_yleft_yright_mat_nnz = 0;

    integer _hess_y_p_mat_nnz = 0;
    integer *_p_hess_y_p_mat_outer_starts = nullptr;
    integer *_p_hess_y_p_mat_rows = nullptr;
    integer _hess_first_y_p_mat_nnz = 0;
    integer _hess_last_y_p_mat_nnz = 0;

    integer _hess_ay_p_mat_nnz = 0;

    integer _hess_xi_xf_mat_nnz = 0;
    integer _hess_p_p_lower_mat_nnz = 0;

    inline integer getNlpHessianPtrIndexForPP() const { return getNlpHessianNnz() - _hess_p_p_lower_mat_nnz; }

    inline integer getNlpHessianLeftColumnBlockNnz() const {
      return _hess_y_y_lower_mat_nnz + _hess_y_ay_mat_nnz + _hess_yleft_yright_mat_nnz + _hess_y_p_mat_nnz;
    }

    inline integer getNlpHessianCentreColumnBlockNnz() const {
      return _hess_ay_ay_lower_mat_nnz + _hess_y_ay_mat_nnz + _hess_ay_p_mat_nnz;
    }

    inline integer getNlpHessianRightColumnBlockNnz() const { return _hess_y_y_lower_mat_nnz + _hess_y_p_mat_nnz; }

    inline integer getNlpHessianFirstColumnBlockNnz() const {
      return _hess_first_y_y_lower_mat_nnz + _hess_y_ay_mat_nnz + _hess_yleft_yright_mat_nnz + _hess_xi_xf_mat_nnz +
             _hess_first_y_p_mat_nnz;
    };

    inline integer getNlpHessianLastColumnBlockNnz() const {
      return _hess_last_y_y_lower_mat_nnz + _hess_last_y_p_mat_nnz;
    };

    inline integer getNlpHessianPtrIndexForLeftBlock(integer const mesh_interval) const {
      return getNlpHessianFirstColumnBlockNnz() - getNlpHessianLeftColumnBlockNnz() +
             (getNlpHessianLeftColumnBlockNnz() + getNlpHessianCentreColumnBlockNnz()) * mesh_interval;
    }

    void setupForNlpHessianMatrixes();

    void calculateHessianBlockAtMeshMiddle(ocpStateAtInterval const & ocp_state,
                                           real const lambda_0_not_scaled,
                                           real const lambda_not_scaled[],
                                           real const int_constr_lambda_scaled[],
                                           real hessian_y_y[],
                                           real hessian_y_y_next[],
                                           real hessian_y_p_next[],
                                           SparseMatrix &hess_p_p_mat) const;

    void calculateHessianLastConstrBcsMayer(real const initial_state_control[],
                                            real const final_point_constrol[], real const ocp_params[],
                                            real const initial_zeta,
                                            real const final_zeta,
                                            real const last_dz,
                                            real const lambda_not_scaled[], real const last_constr_lambda_not_scaled[],
                                            real const lambda_0_not_scaled,
                                            SparseMatrix &out_last_constr_hess_xu_xu_lower_mat,
                                            SparseMatrix &out_point_constr_hess_xu_p_mat,
                                            SparseMatrix &out_point_constr_hess_p_p_lower_mat,
                                            SparseMatrix &out_bcs_hess_xu_init_xu_init_lower_mat,
                                            SparseMatrix &out_bcs_hess_xu_init_xu_fin_mat,
                                            SparseMatrix &out_bcs_hess_xu_init_p_mat,
                                            SparseMatrix &out_bcs_hess_xu_fin_xu_fin_lower_mat,
                                            SparseMatrix &out_bcs_hess_xu_fin_p_mat,
                                            SparseMatrix &out_bcs_hess_p_p_lower_mat,
                                            SparseMatrix &out_mayer_hess_xu_init_xu_init_lower_mat,
                                            SparseMatrix &out_mayer_hess_xu_init_xu_fin_mat,
                                            SparseMatrix &out_mayer_hess_xu_init_p_mat,
                                            SparseMatrix &out_mayer_hess_xu_fin_xu_fin_lower_mat,
                                            SparseMatrix &out_mayer_hess_xu_fin_p_mat,
                                            SparseMatrix &out_mayer_hess_p_p_lower_mat) const;

    void calculateNlpHessianPattern(SparseMatrix const &hess_first_y_y_lower_mat,
                                    SparseMatrix const &hess_y_ay_mat,
                                    SparseMatrix const &hess_yleft_yright_mat,
                                    SparseMatrix const &hess_xi_xf_mat,
                                    SparseMatrix const &hess_first_y_p_mat,
                                    SparseMatrix const &hess_ay_ay_lower_mat,
                                    SparseMatrix const &hess_ay_p_mat,
                                    SparseMatrix const &hess_y_y_lower_mat,
                                    SparseMatrix const &hess_y_p_mat,
                                    SparseMatrix const &hess_last_y_y_lower_mat,
                                    SparseMatrix const &hess_last_y_p_mat,
                                    SparseMatrix const &hess_p_p_lower_mat);

    void setup();

    void setupScaling();

    bool _multiply_foeqns_by_dz;
    bool _multiply_path_constr_by_dz;
    bool _multiply_point_constr_by_dz;

    // scaling quantities

    real *_p_scaling_y = nullptr;
    real *_p_inv_scaling_y = nullptr;
    real *_p_scaling_ay = nullptr;
    real *_p_inv_scaling_ay = nullptr;
    real *_p_scaling_r = nullptr;
    real *_p_inv_scaling_r = nullptr;

    real _inv_scaling_target = 1;
    bool _is_target_lagrange = false;

    real *_p_inv_scaling_fo_eqns_global = nullptr;
    bool _scale_fo_eqns_by_dz = false;
    real *_p_inv_scaling_point_constr_global = nullptr;
    bool _scale_point_constr_by_dz = false;
    real *_p_inv_scaling_path_constr_global = nullptr;
    bool _scale_path_constr_by_dz = false;
    real *_p_inv_scaling_int_constr = nullptr;
    real *_p_inv_scaling_bcs = nullptr;

    //target gradient scaling
    real *_p_scale_factor_lagrange_target_j_y = nullptr;
    real *_p_scale_factor_lagrange_target_j_ay = nullptr;
    real *_p_scale_factor_lagrange_target_j_p = nullptr;
    real *_p_scale_factor_mayer_j_yi = nullptr;
    real *_p_scale_factor_mayer_j_yf = nullptr;
    real *_p_scale_factor_mayer_j_p = nullptr;

    //constraints jacobian scaling
    real *_p_scale_factor_fo_eqns_j_y = nullptr;
    real *_p_scale_factor_fo_eqns_j_ay = nullptr;
    real *_p_scale_factor_fo_eqns_j_p = nullptr;
    real *_p_scale_factor_path_constr_j_y = nullptr;
    real *_p_scale_factor_path_constr_j_ay = nullptr;
    real *_p_scale_factor_path_constr_j_p = nullptr;
    real *_p_scale_factor_int_constr_j_y = nullptr;
    real *_p_scale_factor_int_constr_j_ay = nullptr;
    real *_p_scale_factor_int_constr_j_p = nullptr;
    real *_p_scale_factor_point_constr_j_y = nullptr;
    real *_p_scale_factor_point_constr_j_p = nullptr;
    real *_p_scale_factor_bcs_j_xifp = nullptr;

    // lagrangian hessian scaling
    real *_p_scale_factor_hess_y_y_lower_mat = nullptr;
    real *_p_scale_factor_hess_ay_ay_lower_mat = nullptr;
    real *_p_scale_factor_hess_y_ay_mat = nullptr;
    real *_p_scale_factor_hess_ay_y_mat = nullptr;
    real *_p_scale_factor_hess_y_p_mat = nullptr;
    real *_p_scale_factor_hess_ay_p_mat = nullptr;
    real *_p_scale_factor_hess_yleft_yright_mat = nullptr;

    // method to calculate the Nlp quantities between 2 mesh pionts (will be runned by some treads)

    void calculateNlpQuantitiesBetweenMeshPoints(integer const first_mesh_point, integer const last_mesh_point,
                                                 real const nlp_y[], real const ocp_params[],
                                                 real *lagrange_target_out,
                                                 real lagrange_target_jac_y_out[], real lagrange_target_j_r_out[],
                                                 real lagrange_target_j_y_last_out[],
                                                 real constraints_out[], real int_constraints_out[],
                                                 real constraints_jac_out[], real int_constraints_jac_y_out[],
                                                 real int_constraints_jac_p_out[],
                                                 real int_constraints_jac_y_last_out[],
                                                 real hessian_out[], real hessian_last_column_out[],
                                                 SparseMatrix *hess_p_p_lower_mat,
                                                 real const lambda[], real const integral_constraint_lambda_scaled[],
                                                 real const lambda_0,
                                                 std::exception_ptr &exc_ptr
    ) const;

    // thread related variables

    struct ThreadJob {
      std::thread *th = nullptr;
      mutable std::function<void()> job = nullptr;
      std::condition_variable *cond_var = nullptr;
      std::mutex *job_mutex = nullptr;
      std::exception_ptr exc_ptr = nullptr;
      mutable bool job_todo = false;
      bool kill = false;
      integer start_mesh_interval = 0;
      integer end_mesh_interval = 0;
      single_thread_affinity affinity = {};
    };

    // array of jobs
    ThreadJob *_thread_jobs = nullptr;
    integer _actual_num_threads = 0;

    void clearThreadJobs();

    // method to calculate how to span the nlp over multiple threads
    void calculateWorkForThreads();

  private:

    RK1Ocp2NlpSinglePhase();

    RK1Ocp2NlpSinglePhase(const RK1Ocp2NlpSinglePhase &);

    RK1Ocp2NlpSinglePhase &operator=(const RK1Ocp2NlpSinglePhase &);

  };
}

#endif
