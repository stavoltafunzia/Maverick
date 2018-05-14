#include "MaverickCore/EquationSolverInterface.hh"
#include "MaverickCore/Midpoint/MidpointIntegrator.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"

#ifdef DO_NOT_USE_MAV_SHARED_LIB
#include "Maverick2Ipopt/getIpoptSolvers.hh"
#include "MaverickTensolve/getTensolveSolver.hh"
#else

#endif

#define _dim_fo _dim_eq

using namespace std;

namespace Maverick {

  MidpointIntegrator::MidpointIntegrator(MaverickOcp const &ocp_problem, OcpScaling const &ocp_scaling,
                                         integer const i_phase,
                                         MeshSolutionRefiner::EquationIntegratorType integrator_type) : _ocp(
      ocp_problem), _scaling(ocp_scaling), _i_phase(i_phase) {

    _dim_x = _ocp.numberOfStates(_i_phase);
    _dim_u = _ocp.numberOfControls(_i_phase);
    _dim_ax = _ocp.numberOfAlgebraicStates(_i_phase);
    _dim_au = _ocp.numberOfAlgebraicControls(_i_phase);
    _dim_y = _dim_u + _dim_x;
    _dim_ay = _dim_au + _dim_ax;
    _dim_eq = _ocp.numberOfFirstOrderEquations(_i_phase);
    _dim_unk = _dim_x + _dim_ax;

    _p_c_xu_right_ns = new real[_dim_y];
    _p_c_xu_left_ns = new real[_dim_y];
    _p_c_xu_center_ns = new real[_dim_y];
    _p_c_xu_diff_ns = new real[_dim_y];
    _p_c_axu_ns = new real[_dim_ay];

    _dim_p = _ocp.numberOfParameters(_i_phase);
    _p_p_ns = new real[_dim_p];

    _p_scaling_y = new real[_dim_y];
    copyVectorTo(_scaling.getStatesControlScaling(_i_phase).data(), _p_scaling_y, _dim_y);
    _p_inv_scaling_y = new real[_dim_y];
    writeInverseVectorTo(_p_scaling_y, _p_inv_scaling_y, _dim_y);

    _p_scaling_ay = new real[_dim_ay];
    copyVectorTo(_scaling.getAlgebraicStatesControlScaling(_i_phase).data(), _p_scaling_ay, _dim_ay);
    _p_inv_scaling_ay = new real[_dim_ay];
    writeInverseVectorTo(_p_scaling_ay, _p_inv_scaling_ay, _dim_ay);

    _p_inv_scaling_fo_eqns = new real[_dim_fo];
    writeInverseVectorTo(_scaling.getFoEqnsScaling(_i_phase).data(), _p_inv_scaling_fo_eqns, _dim_fo);

    initializeMatrixes();

    if (integrator_type == MeshSolutionRefiner::EquationIntegratorType::integrator_tensolve)
      loadTensolveEquationSolver();
    else
      loadIpoptEquationSolver();

  }


  MidpointIntegrator::~MidpointIntegrator() {
    _p_solver = nullptr;

    std::vector<real *> to_delete_real = {_p_scaling_y, _p_inv_scaling_y, _p_scaling_ay, _p_inv_scaling_ay,
                                          _p_c_xu_left_ns, _p_c_xu_right_ns, _p_c_axu_ns, _p_p_ns, _p_c_xu_center_ns,
                                          _p_c_xu_diff_ns,
                                          _p_inv_scaling_fo_eqns, _p_scale_factor_jac, _p_scale_factor_jac_dense,
                                          _p_scale_factor_hess
    };

    for (std::vector<real *>::iterator it = to_delete_real.begin(); it != to_delete_real.end(); it++) {
      if (*it != nullptr)
        delete[] *it;
    }

    std::vector<integer *> to_delete_int = {_p_j_xu_outer_starts, _p_j_dxu_outer_starts, _p_j_axu_outer_starts,
                                            _p_j_xu_rows, _p_j_dxu_rows, _p_j_axu_rows,
                                            _p_h_xu_xu_outer_starts, _p_h_xu_dxu_outer_starts, _p_h_xu_axu_outer_starts,
                                            _p_h_dxu_dxu_outer_starts, _p_h_dxu_axu_outer_starts,
                                            _p_h_axu_axu_outer_starts,
                                            _p_h_xu_xu_rows, _p_h_xu_dxu_rows, _p_h_xu_axu_rows, _p_h_dxu_dxu_rows,
                                            _p_h_dxu_axu_rows, _p_h_axu_axu_rows,
                                            _p_jac_cols, _p_jac_rows};//, _p_hess_cols, _p_hess_rows};

    for (std::vector<integer *>::iterator it = to_delete_int.begin(); it != to_delete_int.end(); it++) {
      if (*it != nullptr)
        delete[] *it;
    }


  }

  void MidpointIntegrator::initializeMatrixes() {
    // JACOBIAN
    {
      integer const _nnz_j_xu = _ocp.foEqnsJacXuNnz(_i_phase);
      integer const _nnz_j_dxu = _ocp.foEqnsJacDxuNnz(_i_phase);
      integer const _nnz_j_axu = _ocp.foEqnsJacAxuNnz(_i_phase);

      integer j_xu_cols[_nnz_j_xu];
      _p_j_xu_rows = new integer[_nnz_j_xu];
      integer j_dxu_cols[_nnz_j_dxu];
      _p_j_dxu_rows = new integer[_nnz_j_dxu];
      integer j_axu_cols[_nnz_j_axu];
      _p_j_axu_rows = new integer[_nnz_j_axu];
      _ocp.foEqnsJacXuPattern(_i_phase, _p_j_xu_rows, j_xu_cols);
      _ocp.foEqnsJacDxuPattern(_i_phase, _p_j_dxu_rows, j_dxu_cols);
      _ocp.foEqnsJacAxuPattern(_i_phase, _p_j_axu_rows, j_axu_cols);

      SparseMatrix jac_x(_dim_eq, _dim_x);
      SparseMatrix jac_ax(_dim_eq, _dim_ax);

      {
        SparseMatrix jac_xu(_dim_eq, _dim_y);
        jac_xu.reserve(_nnz_j_xu);
        for (integer i = 0; i < _nnz_j_xu; i++) {
          integer row = _p_j_xu_rows[i];
          integer col = j_xu_cols[i];
          jac_xu.insert(row, col) = 1;
        }
        jac_xu.makeCompressed();
        _p_j_xu_outer_starts = new integer[jac_xu.outerSize() + 1];
        copyVectorTo(jac_xu.outerIndexPtr(), _p_j_xu_outer_starts, (integer) jac_xu.outerSize() + 1);

        SparseMatrix jac_dxu(_dim_eq, _dim_y);
        jac_dxu.reserve(_nnz_j_dxu);
        for (integer i = 0; i < _nnz_j_dxu; i++) {
          integer row = _p_j_dxu_rows[i];
          integer col = j_dxu_cols[i];
          jac_dxu.insert(row, col) = 1;
        }
        jac_dxu.makeCompressed();
        _p_j_dxu_outer_starts = new integer[jac_dxu.outerSize() + 1];
        copyVectorTo(jac_dxu.outerIndexPtr(), _p_j_dxu_outer_starts, (integer) jac_dxu.outerSize() + 1);

        SparseMatrix jac_axu(_dim_eq, _dim_ay);
        jac_axu.reserve(_nnz_j_axu);
        for (integer i = 0; i < _nnz_j_axu; i++) {
          integer row = _p_j_axu_rows[i];
          integer col = j_axu_cols[i];
          jac_axu.insert(row, col) = 1;
        }
        jac_axu.makeCompressed();
        _p_j_axu_outer_starts = new integer[jac_axu.outerSize() + 1];
        copyVectorTo(jac_axu.outerIndexPtr(), _p_j_axu_outer_starts, (integer) jac_axu.outerSize() + 1);

        jac_x = jac_xu.block(0, 0, _dim_eq, _dim_x) + jac_dxu.block(0, 0, _dim_eq, _dim_x);
        jac_ax = jac_axu.block(0, 0, _dim_eq, _dim_ax);
      }
      jac_x.makeCompressed();
      jac_ax.makeCompressed();

      integer _nnz_jac_x = (integer) jac_x.nonZeros();
      integer _nnz_jac_ax = (integer) jac_ax.nonZeros();
      _nnz_jac = _nnz_jac_x + _nnz_jac_ax;
      _p_scale_factor_jac = new real[_nnz_jac];
      _p_jac_cols = new integer[_nnz_jac];
      _p_jac_rows = new integer[_nnz_jac];

      integer counter = 0;
      for (integer k = 0; k < jac_x.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(jac_x, k); it; ++it) {
          integer i_row = (integer) it.row();
          integer i_col = (integer) it.col();
          _p_jac_cols[counter] = i_col;
          _p_jac_rows[counter] = i_row;
          _p_scale_factor_jac[counter] = _p_inv_scaling_fo_eqns[i_row] * _p_scaling_y[i_col];
          counter++;
        }
      }
      for (integer k = 0; k < jac_ax.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(jac_ax, k); it; ++it) {
          integer i_row = (integer) it.row();
          integer i_col = (integer) it.col() + _dim_x;
          _p_jac_cols[counter] = i_col;
          _p_jac_rows[counter] = i_row;
          // for the scaling, check if the column is that of a normal state or algebraic state
          _p_scale_factor_jac[counter] = _p_inv_scaling_fo_eqns[i_row] * _p_scaling_ay[i_col - _dim_x];
          counter++;
        }
      }

      counter = 0;
      _p_scale_factor_jac_dense = new real[_dim_eq * _dim_unk];
      for (integer j_col = 0; j_col < _dim_unk; j_col++)
        for (integer i_row = 0; i_row < _dim_eq; i_row++) {
          if (j_col < _dim_x)
            _p_scale_factor_jac_dense[counter] = _p_inv_scaling_fo_eqns[i_row] * _p_scaling_y[j_col];
          else
            _p_scale_factor_jac_dense[counter] = _p_inv_scaling_fo_eqns[i_row] * _p_scaling_ay[j_col - _dim_x];
          counter++;
        }
    }

    //HESSIAN
    {
      integer const _nnz_h_xu_xu = _ocp.foEqnsHessXuXuNnz(_i_phase);
      integer const _nnz_h_xu_dxu = _ocp.foEqnsHessXuDxuNnz(_i_phase);
      integer const _nnz_h_xu_axu = _ocp.foEqnsHessXuAxuNnz(_i_phase);
      integer const _nnz_h_dxu_dxu = _ocp.foEqnsHessDxuDxuNnz(_i_phase);
      integer const _nnz_h_dxu_axu = _ocp.foEqnsHessDxuAxuNnz(_i_phase);
      integer const _nnz_h_axu_axu = _ocp.foEqnsHessAxuAxuNnz(_i_phase);

      integer h_xu_xu_cols[_nnz_h_xu_xu];
      _p_h_xu_xu_rows = new integer[_nnz_h_xu_xu];
      integer h_xu_dxu_cols[_nnz_h_xu_dxu];
      _p_h_xu_dxu_rows = new integer[_nnz_h_xu_dxu];
      integer h_xu_axu_cols[_nnz_h_xu_axu];
      _p_h_xu_axu_rows = new integer[_nnz_h_xu_axu];
      integer h_dxu_dxu_cols[_nnz_h_dxu_dxu];
      _p_h_dxu_dxu_rows = new integer[_nnz_h_dxu_dxu];
      integer h_dxu_axu_cols[_nnz_h_dxu_axu];
      _p_h_dxu_axu_rows = new integer[_nnz_h_dxu_axu];
      integer h_axu_axu_cols[_nnz_h_axu_axu];
      _p_h_axu_axu_rows = new integer[_nnz_h_axu_axu];

      _ocp.foEqnsHessXuXuPattern(_i_phase, _p_h_xu_xu_rows, h_xu_xu_cols);
      _ocp.foEqnsHessXuDxuPattern(_i_phase, _p_h_xu_dxu_rows, h_xu_dxu_cols);
      _ocp.foEqnsHessXuAxuPattern(_i_phase, _p_h_xu_axu_rows, h_xu_axu_cols);
      _ocp.foEqnsHessDxuDxuPattern(_i_phase, _p_h_dxu_dxu_rows, h_dxu_dxu_cols);
      _ocp.foEqnsHessDxuAxuPattern(_i_phase, _p_h_dxu_axu_rows, h_dxu_axu_cols);
      _ocp.foEqnsHessAxuAxuPattern(_i_phase, _p_h_axu_axu_rows, h_axu_axu_cols);

      DenseMatrix hess(_dim_unk, _dim_unk);

      {
        SparseMatrix hess_xu_xu(_dim_y, _dim_y);
        {
          hess_xu_xu.reserve(_nnz_h_xu_xu + _nnz_h_xu_dxu + _nnz_h_dxu_dxu);
          for (integer i = 0; i < _nnz_h_xu_xu; i++) {
            integer col = h_xu_xu_cols[i];
            integer row = _p_h_xu_xu_rows[i];
            hess_xu_xu.insert(row, col) = 1;
          }
          hess_xu_xu.makeCompressed();
          _p_h_xu_xu_outer_starts = new integer[hess_xu_xu.outerSize() + 1];
          copyVectorTo(hess_xu_xu.outerIndexPtr(), _p_h_xu_xu_outer_starts, (integer) hess_xu_xu.outerSize() + 1);
        }
        SparseMatrix hess_xu_dxu(_dim_y, _dim_y);
        {
          hess_xu_dxu.reserve(_nnz_h_xu_xu + _nnz_h_xu_dxu + _nnz_h_dxu_dxu);
          for (integer i = 0; i < _nnz_h_xu_dxu; i++) {
            integer col = h_xu_dxu_cols[i];
            integer row = _p_h_xu_dxu_rows[i];
            hess_xu_dxu.insert(row, col) = 1;
          }
          hess_xu_dxu.makeCompressed();
          _p_h_xu_dxu_outer_starts = new integer[hess_xu_dxu.outerSize() + 1];
          copyVectorTo(hess_xu_dxu.outerIndexPtr(), _p_h_xu_dxu_outer_starts, (integer) hess_xu_dxu.outerSize() + 1);
        }

        SparseMatrix hess_xu_axu(_dim_ay, _dim_y);
        {
          hess_xu_axu.reserve(_nnz_h_xu_axu);
          for (integer i = 0; i < _nnz_h_xu_axu; i++) {
            integer col = h_xu_axu_cols[i];
            integer row = _p_h_xu_axu_rows[i];
            hess_xu_axu.insert(row, col) = 1;
          }
          hess_xu_axu.makeCompressed();
          _p_h_xu_axu_outer_starts = new integer[hess_xu_axu.outerSize() + 1];
          copyVectorTo(hess_xu_axu.outerIndexPtr(), _p_h_xu_axu_outer_starts, (integer) hess_xu_axu.outerSize() + 1);
        }

        SparseMatrix hess_dxu_dxu(_dim_y, _dim_y);
        {
          hess_dxu_dxu.reserve(_nnz_h_xu_xu + _nnz_h_xu_dxu + _nnz_h_dxu_dxu);
          for (integer i = 0; i < _nnz_h_dxu_dxu; i++) {
            integer row = _p_h_dxu_dxu_rows[i];
            integer col = h_dxu_dxu_cols[i];
            hess_dxu_dxu.insert(row, col) = 1;
          }
          hess_dxu_dxu.makeCompressed();
          _p_h_dxu_dxu_outer_starts = new integer[hess_dxu_dxu.outerSize() + 1];
          copyVectorTo(hess_dxu_dxu.outerIndexPtr(), _p_h_dxu_dxu_outer_starts, (integer) hess_dxu_dxu.outerSize() + 1);
        }

        SparseMatrix hess_dxu_axu(_dim_ay, _dim_y);
        {
          hess_dxu_axu.reserve(_nnz_h_dxu_axu);
          for (integer i = 0; i < _nnz_h_dxu_axu; i++) {
            integer row = _p_h_dxu_axu_rows[i];
            integer col = h_dxu_axu_cols[i];
            hess_dxu_axu.insert(row, col) = 1;
          }
          hess_dxu_axu.makeCompressed();
          _p_h_dxu_axu_outer_starts = new integer[hess_dxu_axu.outerSize() + 1];
          copyVectorTo(hess_dxu_axu.outerIndexPtr(), _p_h_dxu_axu_outer_starts, (integer) hess_dxu_axu.outerSize() + 1);
        }

        SparseMatrix hess_axu_axu(_dim_ay, _dim_ay);
        {
          hess_axu_axu.reserve(_nnz_h_axu_axu);
          for (integer i = 0; i < _nnz_h_axu_axu; i++) {
            integer row = _p_h_axu_axu_rows[i];
            integer col = h_axu_axu_cols[i];
            hess_axu_axu.insert(row, col) = 1;
          }
          hess_axu_axu.makeCompressed();
          _p_h_axu_axu_outer_starts = new integer[hess_axu_axu.outerSize() + 1];
          copyVectorTo(hess_axu_axu.outerIndexPtr(), _p_h_axu_axu_outer_starts, (integer) hess_axu_axu.outerSize() + 1);
        }

        hess.block(0, 0, _dim_x, _dim_x) =
            hess_xu_xu.block(0, 0, _dim_x, _dim_x) + hess_xu_dxu.block(0, 0, _dim_x, _dim_x) +
            hess_dxu_dxu.block(0, 0, _dim_x, _dim_x);
        hess.block(_dim_x, _dim_x, _dim_ax, _dim_ax) = hess_axu_axu.block(0, 0, _dim_ax, _dim_ax);
        hess.block(_dim_x, 0, _dim_ax, _dim_x) =
            hess_xu_axu.block(0, 0, _dim_ax, _dim_x) + hess_dxu_axu.block(0, 0, _dim_ax, _dim_x);
        hess.block(0, _dim_x, _dim_x, _dim_ax) = hess.block(_dim_x, 0, _dim_ax, _dim_x).transpose();
      }
      _p_scale_factor_hess = new real[_dim_unk * _dim_unk];

      integer counter = 0;
      {
        real scale_row = 1;
        real scale_col = 1;
        for (integer j_col = 0; j_col < _dim_unk; j_col++)
          for (integer i_row = 0; i_row < _dim_unk; i_row++) {

            if (j_col < _dim_x)
              scale_col = _p_scaling_y[j_col];
            else
              scale_col = _p_scaling_ay[j_col - _dim_x];

            if (i_row < _dim_x)
              scale_row = _p_scaling_y[i_row];
            else
              scale_row = _p_scaling_ay[i_row - _dim_x];

            _p_scale_factor_hess[counter] = scale_row * scale_col;
            counter++;
          }
      }
    }
  }

  void MidpointIntegrator::loadIpoptEquationSolver() {
    _p_solver = nullptr;

#ifndef DO_NOT_USE_MAV_SHARED_LIB
    MaverickSingleton &maverick = MaverickSingleton::getInstance();
    void *p_mavericktoip = maverick.getMaverickToIpSharedLibHandle();
    typedef std::unique_ptr<Maverick::EquationSolverInterface> (*GetIpoptEquationSolver)(
        EquationSolverSupplierInterface const &);

    GetIpoptEquationSolver p_getIpoptEquationSolver = nullptr;
    p_getIpoptEquationSolver = (GetIpoptEquationSolver) dlsym(p_mavericktoip, "getIpoptEquationSolver");
    // if sl_handle exist
    if (p_getIpoptEquationSolver != nullptr) {
      _p_solver = p_getIpoptEquationSolver(*this);
    } else {
      string message = "error while loading shared library: libmavericktoip: " + string(dlerror()) + "\n";
      throw runtime_error(message);
    }
#else
    _p_solver = getIpoptEquationSolver( *this );
#endif
  }

  void MidpointIntegrator::loadTensolveEquationSolver() {
    _p_solver = nullptr;

#ifndef DO_NOT_USE_MAV_SHARED_LIB
    MaverickSingleton &maverick = MaverickSingleton::getInstance();
    void *p_maverickts = maverick.getMaverickTsSharedLibHandle();

    typedef std::unique_ptr<Maverick::EquationSolverInterface> (*GetTensolveEquationSolver)(
        EquationSolverSupplierInterface const &);

    GetTensolveEquationSolver p_getTensolveEquationSolver = nullptr;
    p_getTensolveEquationSolver = (GetTensolveEquationSolver) dlsym(p_maverickts, "getTensolveEquationSolver");
    // if sl_handle exist
    if (p_getTensolveEquationSolver) {
      _p_solver = p_getTensolveEquationSolver(*this);
    } else {
      std::string message = "Unknown error in the implementation of the shared library libmaverickts\n";
      throw std::runtime_error(message);
    }
#else
    _p_solver = getTensolveEquationSolver( *this );
#endif
  }

  integer MidpointIntegrator::integrateForward(integer const n_x, real const x_left[], real x_right_solution[],
                                               integer const n_u, real const u_left[], real const u_right[],
                                               integer const n_alg_x, real alg_x_solution[],
                                               integer const n_alg_u, real const alg_u[],
                                               integer const n_p, real const p[],
                                               real const zeta_left, real const d_zeta,
                                               real const starting_x[],
                                               real const starting_alg_x[],
                                               real &solution_error) {
    _zeta_center = zeta_left + d_zeta / 2.0;
    _d_zeta_inv = 1 / d_zeta;
    //        _is_forward = true;

    //write guess
    copyVectorTo(starting_x, _p_c_xu_right_ns, _dim_x);
    copyVectorTo(starting_alg_x, _p_c_axu_ns, _dim_ax);
    copyVectorTo(p, _p_p_ns, _dim_p);

    //write left state
    copyVectorTo(x_left, _p_c_xu_left_ns, _dim_x);

    //write left control
    copyVectorTo(u_left, _p_c_xu_left_ns + _dim_x, _dim_u);

    //write right control
    copyVectorTo(u_right, _p_c_xu_right_ns + _dim_x, _dim_u);

    //write algebraic control
    copyVectorTo(alg_u, _p_c_axu_ns + _dim_ax, _dim_au);

    //perform integration
    EquationSolverInterface::EquationSolverReturnStatus status = _p_solver->solve();

    solution_error = _sol_error;

    // write solution to output
    copyVectorTo(_p_c_xu_right_ns, x_right_solution, _dim_x);
    copyVectorTo(_p_c_axu_ns, alg_x_solution, _dim_ax);

    if (status == EquationSolverInterface::problem_detected)
      return 1;

    return 0;

  }

  void MidpointIntegrator::writeStateScaled(real const x[], real const ax[]) const {
    // copy right x to right state
    multiplyAndCopyVectorTo(x, _p_c_xu_right_ns, _p_scaling_y, _dim_x);
    // copy algebraic x
    multiplyAndCopyVectorTo(ax, _p_c_axu_ns, _p_scaling_ay, _dim_ax);

    // calculate center state and derivative
    computeTpzCenterWithoutScaling(_p_c_xu_left_ns, _p_c_xu_right_ns, _p_c_xu_center_ns, _dim_y);
    computeTpzDerivativeWithoutScaling(_p_c_xu_left_ns, _p_c_xu_right_ns, _p_c_xu_diff_ns, _d_zeta_inv, _dim_y);
  }

  // EquationSolverSupplierInterface methods

  void MidpointIntegrator::getProblemInfo(integer &n_x, integer &n_eq) const {
    n_x = _dim_unk;
    n_eq = _dim_eq;
  }

  void MidpointIntegrator::getProblemInfo(integer &n_x, integer &n_eq, integer &nnz_jac_sparse) const {
    getProblemInfo(n_x, n_eq);
    nnz_jac_sparse = _nnz_jac;
    //        nnz_hess = _nnz_hess;
  }

  void MidpointIntegrator::getVarsBounds(integer const n_x, real lower[], real upper[]) const {
    MAVERICK_DEBUG_ASSERT(n_x == _dim_unk, "MidpointIntegrator::getVarsBounds: wrong x size\n")

    // differential state bounds
    real tmp_low[_dim_y];
    real tmp_up[_dim_y];
    _ocp.getStatesControlsBounds(_i_phase, _zeta_center, tmp_low, tmp_up);
    multiplyVectorBy(tmp_low, _p_inv_scaling_y, _dim_y);
    multiplyVectorBy(tmp_up, _p_inv_scaling_y, _dim_y);

    // algebraic state bounds
    real tmp_a_low[_dim_ay];
    real tmp_a_up[_dim_ay];
    _ocp.getAlgebraicStatesControlsBounds(_i_phase, _zeta_center, tmp_a_low, tmp_a_up);
    multiplyVectorBy(tmp_a_low, _p_inv_scaling_ay, _dim_ay);
    multiplyVectorBy(tmp_a_up, _p_inv_scaling_ay, _dim_ay);

    // now write the bounds, but relax the bounds slightly
    real tmp_middle;
    real tmp_width;
    real const relax_bound_factor = 10.0; // bounds relaxing factor
    for (integer i = 0; i < _dim_x; i++) {
      tmp_middle = (tmp_up[i] + tmp_low[i]) / 2;
      tmp_width = relax_bound_factor * (tmp_up[i] - tmp_low[i]);
      lower[i] = tmp_middle - tmp_width;
      upper[i] = tmp_middle + tmp_width;
    }
    for (integer i = 0; i < _dim_ax; i++) {
      tmp_middle = (tmp_a_up[i] + tmp_a_low[i]) / 2;
      tmp_width = relax_bound_factor * (tmp_a_up[i] - tmp_a_low[i]);
      lower[_dim_x + i] = tmp_middle - tmp_width;
      upper[_dim_x + i] = tmp_middle + tmp_width;
    }
  }

  void MidpointIntegrator::getSparseJacStructure(integer const nnz_jac, integer cols[], integer rows[]) const {
    MAVERICK_DEBUG_ASSERT(nnz_jac == _nnz_jac, "MidpointIntegrator::getJacStructure: wrong jacobian size\n")
    copyVectorTo(_p_jac_rows, rows, nnz_jac);
    copyVectorTo(_p_jac_cols, cols, nnz_jac);
  }

  void MidpointIntegrator::evalEquations(bool const new_unk,
                                         integer const n_unk, real const unk[],
                                         integer const n_eq, real eq[]) const {
    MAVERICK_DEBUG_ASSERT(n_unk == _dim_unk, "MidpointIntegrator::evalEquations: wrong unknown size\n");
    MAVERICK_DEBUG_ASSERT(n_eq == _dim_eq, "MidpointIntegrator::evalEquations: wrong equation size\n");

    if (new_unk)
      writeStateScaled(unk, unk + _dim_x);

    _ocp.foEqns(_i_phase, _p_c_xu_center_ns, _p_c_xu_diff_ns, _p_c_axu_ns, _p_p_ns, _zeta_center, eq);

    multiplyVectorBy(eq, _p_inv_scaling_fo_eqns, n_eq);
  }

  void MidpointIntegrator::evalEquationsSparseJac(bool const new_unk,
                                                  integer const n_unk, real const unk[],
                                                  integer const n_grad, real grad[]) const {
    MAVERICK_DEBUG_ASSERT(n_unk == _dim_unk, "MidpointIntegrator::evalEquationsSparseJac: wrong unknown size\n");
    MAVERICK_DEBUG_ASSERT(n_grad == _nnz_jac, "MidpointIntegrator::evalEquationsSparseJac: wrong gradient size\n");
    if (new_unk)
      writeStateScaled(unk, unk + _dim_x);

    integer const _nnz_j_xu = _ocp.foEqnsJacXuNnz(_i_phase);
    integer const _nnz_j_dxu = _ocp.foEqnsJacDxuNnz(_i_phase);
    integer const _nnz_j_axu = _ocp.foEqnsJacAxuNnz(_i_phase);
    integer const _nnz_j_p = _ocp.foEqnsJacPNnz(_i_phase);

    real grad_xu[_nnz_j_xu];
    real grad_dxu[_nnz_j_dxu];
    real grad_axu[_nnz_j_axu];
    real grad_p[_nnz_j_p];
    _ocp.foEqnsJac(_i_phase, _p_c_xu_center_ns, _p_c_xu_diff_ns, _p_c_axu_ns, _p_p_ns, _zeta_center, grad_xu, grad_dxu,
                   grad_axu, grad_p);

    Eigen::Map<SparseMatrix> grad_xu_mat(_dim_eq, _dim_y, _nnz_j_xu,
                                         _p_j_xu_outer_starts,
                                         _p_j_xu_rows,
                                         grad_xu,
                                         0);

    Eigen::Map<SparseMatrix> grad_dxu_mat(_dim_eq, _dim_y, _nnz_j_dxu,
                                          _p_j_dxu_outer_starts,
                                          _p_j_dxu_rows,
                                          grad_dxu,
                                          0);

    Eigen::Map<SparseMatrix> grad_axu_mat(_dim_eq, _dim_ay, _nnz_j_axu,
                                          _p_j_axu_outer_starts,
                                          _p_j_axu_rows,
                                          grad_axu,
                                          0);

    SparseMatrix grad_x(_dim_eq, _dim_x);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    grad_x.reserve(n_grad);
#endif
    //        if (_is_forward)
    grad_x = grad_xu_mat.block(0, 0, _dim_eq, _dim_x) * 0.5 + grad_dxu_mat.block(0, 0, _dim_eq, _dim_x) * _d_zeta_inv;
    grad_x.makeCompressed();
    integer const grad_x_nnz = (integer) grad_x.nonZeros();
    multiplyAndCopyVectorTo(grad_x.valuePtr(), grad, _p_scale_factor_jac, grad_x_nnz);

    SparseMatrix grad_ax(_dim_eq, _dim_ax);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    grad_ax.reserve(n_grad);
#endif
    grad_ax = grad_axu_mat.block(0, 0, _dim_eq, _dim_ax);
    grad_ax.makeCompressed();
    multiplyAndCopyVectorTo(grad_ax.valuePtr(), grad + grad_x_nnz, _p_scale_factor_jac + grad_x_nnz,
                            (integer) grad_ax.nonZeros());
  }

  void MidpointIntegrator::evalEquationsDenseJac(bool const new_unk,
                                                 integer const n_unk, real const unk[],
                                                 real grad[]) const {
    MAVERICK_DEBUG_ASSERT(_dim_x + _dim_ax == _dim_unk,
                          "MidpointIntegrator::evalEquationsDenseJac: wrong unknown size\n");
    if (new_unk)
      writeStateScaled(unk, unk + _dim_x);

    integer const _nnz_j_xu = _ocp.foEqnsJacXuNnz(_i_phase);
    integer const _nnz_j_dxu = _ocp.foEqnsJacDxuNnz(_i_phase);
    integer const _nnz_j_axu = _ocp.foEqnsJacAxuNnz(_i_phase);
    integer const _nnz_j_p = _ocp.foEqnsJacPNnz(_i_phase);

    real grad_xu[_nnz_j_xu];
    real grad_dxu[_nnz_j_dxu];
    real grad_axu[_nnz_j_axu];
    real grad_p[_nnz_j_p];
    _ocp.foEqnsJac(_i_phase, _p_c_xu_center_ns, _p_c_xu_diff_ns, _p_c_axu_ns, _p_p_ns, _zeta_center, grad_xu, grad_dxu,
                   grad_axu, grad_p);

    Eigen::Map<SparseMatrix> grad_xu_mat(_dim_eq, _dim_y, _nnz_j_xu,
                                         _p_j_xu_outer_starts,
                                         _p_j_xu_rows,
                                         grad_xu,
                                         0);

    Eigen::Map<SparseMatrix> grad_dxu_mat(_dim_eq, _dim_y, _nnz_j_dxu,
                                          _p_j_dxu_outer_starts,
                                          _p_j_dxu_rows,
                                          grad_dxu,
                                          0);

    Eigen::Map<SparseMatrix> grad_axu_mat(_dim_eq, _dim_ay, _nnz_j_axu,
                                          _p_j_axu_outer_starts,
                                          _p_j_axu_rows,
                                          grad_axu,
                                          0);

    DenseMatrix result(_dim_eq, _dim_unk);

    result.block(0, 0, _dim_eq, _dim_x) =
        grad_xu_mat.block(0, 0, _dim_eq, _dim_x) * 0.5 + grad_dxu_mat.block(0, 0, _dim_eq, _dim_x) * _d_zeta_inv;
    result.block(0, _dim_x, _dim_eq, _dim_ax) = grad_axu_mat.block(0, 0, _dim_eq, _dim_ax);

    multiplyAndCopyVectorTo(result.data(), grad, _p_scale_factor_jac_dense, _dim_eq * _dim_unk);
  }

  void MidpointIntegrator::evalEquationsDenseHess(bool const new_unk,
                                                  integer const n_unk, real const unk[],
                                                  integer const n_eq, real const lambda[],
                                                  real hess[]) const {

    MAVERICK_DEBUG_ASSERT(_dim_x + _dim_ax == _dim_unk, "MidpointIntegrator::evalEquationsDenseHess: wrong x size\n");
    MAVERICK_DEBUG_ASSERT(n_eq == _dim_eq, "MidpointIntegrator::evalEquationsDenseHess: wrong equation size\n");

    // scale the lambda
    real lambda_scaled[_dim_x];
    multiplyAndCopyVectorTo(lambda, lambda_scaled, _p_inv_scaling_fo_eqns, _dim_fo);

    if (new_unk)
      writeStateScaled(unk, unk + _dim_x);

    integer const nnz_xu_xu = _ocp.foEqnsHessXuXuNnz(_i_phase);
    integer const nnz_xu_dxu = _ocp.foEqnsHessXuDxuNnz(_i_phase);
    integer const nnz_xu_axu = _ocp.foEqnsHessXuAxuNnz(_i_phase);
    integer const nnz_dxu_dxu = _ocp.foEqnsHessDxuDxuNnz(_i_phase);
    integer const nnz_dxu_axu = _ocp.foEqnsHessDxuAxuNnz(_i_phase);
    integer const nnz_axu_axu = _ocp.foEqnsHessAxuAxuNnz(_i_phase);

    real hess_xu_xu[nnz_xu_xu];
    real hess_xu_dxu[nnz_xu_dxu];
    real hess_xu_axu[nnz_xu_axu];
    real hess_dxu_dxu[nnz_dxu_dxu];
    real hess_dxu_axu[nnz_dxu_axu];
    real hess_axu_axu[nnz_axu_axu];

    {
      real hess_xu_p[_ocp.foEqnsHessXuPNnz(_i_phase)];
      real hess_dxu_p[_ocp.foEqnsHessDxuPNnz(_i_phase)];
      real hess_axu_p[_ocp.foEqnsHessAxuPNnz(_i_phase)];
      real hess_p_p[_ocp.foEqnsHessPPNnz(_i_phase)];

      _ocp.foEqnsHess(_i_phase, _p_c_xu_center_ns, _p_c_xu_diff_ns, _p_c_axu_ns, _p_p_ns, _zeta_center, lambda_scaled,
                      hess_xu_xu, hess_dxu_dxu, hess_xu_axu, hess_xu_p, hess_dxu_dxu, hess_dxu_axu, hess_dxu_p,
                      hess_axu_axu, hess_axu_p, hess_p_p);
    }

    Eigen::Map<SparseMatrix> hess_xu_xu_mat(_dim_y, _dim_y, nnz_xu_xu,
                                            _p_h_xu_xu_outer_starts,
                                            _p_h_xu_xu_rows,
                                            hess_xu_xu,
                                            0);

    Eigen::Map<SparseMatrix> hess_xu_dxu_mat(_dim_y, _dim_y, nnz_xu_dxu,
                                             _p_h_xu_dxu_outer_starts,
                                             _p_h_xu_dxu_rows,
                                             hess_xu_dxu,
                                             0);

    Eigen::Map<SparseMatrix> hess_xu_axu_mat(_dim_ay, _dim_y, nnz_xu_axu,
                                             _p_h_xu_axu_outer_starts,
                                             _p_h_xu_axu_rows,
                                             hess_xu_axu,
                                             0);

    Eigen::Map<SparseMatrix> hess_dxu_dxu_mat(_dim_y, _dim_y, nnz_dxu_dxu,
                                              _p_h_dxu_dxu_outer_starts,
                                              _p_h_dxu_dxu_rows,
                                              hess_dxu_dxu,
                                              0);

    Eigen::Map<SparseMatrix> hess_dxu_axu_mat(_dim_ay, _dim_y, nnz_dxu_axu,
                                              _p_h_dxu_axu_outer_starts,
                                              _p_h_dxu_axu_rows,
                                              hess_dxu_axu,
                                              0);

    Eigen::Map<SparseMatrix> hess_axu_axu_mat(_dim_ay, _dim_ay, nnz_axu_axu,
                                              _p_h_axu_axu_outer_starts,
                                              _p_h_axu_axu_rows,
                                              hess_axu_axu,
                                              0);

    DenseMatrix result(_dim_unk, _dim_unk);
    //        if (_is_forward)
    result.block(0, 0, _dim_x, _dim_x) = hess_xu_xu_mat.block(0, 0, _dim_x, _dim_x) * 0.25
                                         + hess_dxu_dxu_mat.block(0, 0, _dim_x, _dim_x) * (_d_zeta_inv * _d_zeta_inv)
                                         + hess_xu_dxu_mat.block(0, 0, _dim_x, _dim_x) * 0.5 * _d_zeta_inv;
    result.block(_dim_x, 0, _dim_ax, _dim_x) = hess_xu_axu_mat.block(0, 0, _dim_ax, _dim_x) * 0.5 +
                                               hess_dxu_axu_mat.block(0, 0, _dim_ax, _dim_x) * _d_zeta_inv;
    result.block(0, _dim_x, _dim_x, _dim_ax) = result.block(_dim_x, 0, _dim_ax, _dim_x).transpose();
    result.block(_dim_x, _dim_x, _dim_ax, _dim_ax) = hess_axu_axu_mat.block(0, 0, _dim_ax, _dim_ax);

    multiplyAndCopyVectorTo(result.data(), hess, _p_scale_factor_hess, _dim_unk * _dim_unk);
  }

  void MidpointIntegrator::getStartingPoint(integer const n_x, real x[]) const {
    MAVERICK_DEBUG_ASSERT(n_x == _dim_unk, "MidpointIntegrator::getStartingPoint: wrong x size\n")
    multiplyAndCopyVectorTo(_p_c_xu_right_ns, x, _p_inv_scaling_y, _dim_x);
    multiplyAndCopyVectorTo(_p_c_axu_ns, x + _dim_x, _p_inv_scaling_ay, _dim_ax);
  }

  void MidpointIntegrator::finalizeSolution(integer const n_x, real const x_solution[], real const error) const {
    MAVERICK_DEBUG_ASSERT(n_x == _dim_unk, "MidpointIntegrator::finalizeSolution: wrong x size\n")
    multiplyAndCopyVectorTo(x_solution, _p_c_xu_right_ns, _p_scaling_y, _dim_x);
    multiplyAndCopyVectorTo(x_solution + _dim_x, _p_c_axu_ns, _p_scaling_ay, _dim_ax);
    _sol_error = error;
  }
}
