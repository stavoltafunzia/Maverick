#include "IpoptEquationSolver.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"

using namespace Maverick;
using namespace Ipopt;
using namespace std;

#define DEFUALT_IPOPT_ITERATIONS 100

IpoptEquationSolver::IpoptEquationSolver(EquationSolverSupplierInterface const &problem) : EquationSolverInterface(
    problem) {
  _ipopt_app.Options()->SetStringValue("mu_strategy", "adaptive");
  // _ipopt_app.Options()->SetStringValue("ma57_automatic_scaling", "yes");
  // _ipopt_app.Options()->SetStringValue("linear_solver", "ma57");
  _ipopt_app.Options()->SetIntegerValue("max_iter", DEFUALT_IPOPT_ITERATIONS);
  _ipopt_app.Options()->SetNumericValue("tol", 1e-6);
  _ipopt_app.Options()->SetIntegerValue("print_level", 0);
  //supress the standard Ipopt message
  _ipopt_app.Options()->SetStringValue("sb", "yes");
#ifdef MAVERICK_DEBUG
  _ipopt_app.Options()->SetStringValue("check_derivatives_for_naninf", "yes");
#endif
}

IpoptEquationSolver::~IpoptEquationSolver() {

}

void IpoptEquationSolver::setMaxIterations(integer const max_iterations) {
  if (max_iterations < 0) {
    _ipopt_app.Options()->SetIntegerValue("max_iter", DEFUALT_IPOPT_ITERATIONS);
    MaverickSingleton &maverick = MaverickSingleton::getInstance();
    maverick.Log(InfoLevel::info_level_warning,
                 "IpoptEquationSolver::setMaxIterations: negative number of maximum iterations not allowed. Will use the deualt one: " +
                 std::to_string(DEFUALT_IPOPT_ITERATIONS) + "\n");
  } else {
    _ipopt_app.Options()->SetIntegerValue("max_iter", max_iterations);
  }
}

IpoptEquationSolver::EquationSolverReturnStatus IpoptEquationSolver::solve() {

  ApplicationReturnStatus status;
  status = _ipopt_app.Initialize();
  if (status != Solve_Succeeded) {
    return IpoptEquationSolver::EquationSolverReturnStatus::problem_detected;
  }

  Ipopt::SmartPtr <IpoptEquationSolver::EquationNlp> _p_tnlp = new EquationNlp(_problem);

  status = _ipopt_app.OptimizeTNLP(_p_tnlp);

  if ((status == Solve_Succeeded) ||
      (status == Solved_To_Acceptable_Level) ||
      (status == Feasible_Point_Found) ||
      (status == Search_Direction_Becomes_Too_Small))
    return EquationSolverInterface::EquationSolverReturnStatus::solution_found;

  if (status == Maximum_Iterations_Exceeded)
    return EquationSolverInterface::EquationSolverReturnStatus::max_iterations_exceeded;

  if ((status == Invalid_Option) ||
      (status == Restoration_Failed) ||
      (status == Error_In_Step_Computation) ||
      (status == Invalid_Problem_Definition) ||
      (status == Not_Enough_Degrees_Of_Freedom) ||
      (status == Invalid_Number_Detected) ||
      (status == Unrecoverable_Exception) ||
      (status == NonIpopt_Exception_Thrown) ||
      (status == Insufficient_Memory) ||
      (status == Internal_Error))
    return EquationSolverInterface::EquationSolverReturnStatus::problem_detected;

  return EquationSolverInterface::EquationSolverReturnStatus::solution_not_found;

}

IpoptEquationSolver::EquationNlp::EquationNlp(EquationSolverSupplierInterface const &problem) : _problem(problem) {}

// IMPLEMENTATION OF TNLP INTERFACE

/** Method to return some info about the nlp */
bool IpoptEquationSolver::EquationNlp::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                                                    Index &nnz_h_lag, IndexStyleEnum &index_style) {
#ifdef MAVERICK_DEBUG
  try {
#endif
  integer tmp;
  _problem.getProblemInfo(n, tmp, tmp);
  m = 0;
  nnz_jac_g = 0;
  nnz_h_lag = n * (n + 1) / 2; // we treat he hessian as dense
  index_style = C_STYLE;
#ifdef MAVERICK_DEBUG
  } catch ( runtime_error err) {
      cout << err.what();
      throw err;
  }
#endif
  return true;
}

/** Method to return the bounds for my problem */
bool IpoptEquationSolver::EquationNlp::get_bounds_info(Index n, Number *x_l, Number *x_u,
                                                       Index m, Number *g_l, Number *g_u) {
  //set vars bounds
  _problem.getVarsBounds(n, x_l, x_u);
  return true;
}

/** Method to return the starting point for the algorithm */
bool IpoptEquationSolver::EquationNlp::get_starting_point(Index n, bool init_x, Number *x,
                                                          bool init_z, Number *z_L, Number *z_U,
                                                          Index m, bool init_lambda,
                                                          Number *lambda) {
  if (init_x)
    _problem.getStartingPoint(n, x);
  if (init_z) {
    MAVERICK_ASSERT(false, "IpoptEquationSolver::get_starting_point: multiplier initialisation not implemented\n")
  }
  if (init_lambda) {
    MAVERICK_ASSERT(false, "IpoptEquationSolver::get_starting_point: multiplier initialisation not implemented\n")
  }
  return true;
}

/** Method to return the objective value */
bool IpoptEquationSolver::EquationNlp::eval_f(Index n, const Number *x, bool new_x, Number &obj_value) {
  real equations[n];
  _problem.evalEquations(new_x, n, x, n, equations);
  obj_value = 0;
  for (integer i = 0; i < n; i++)
    obj_value += equations[i] * equations[i];
  return true;
}

/** Method to return the gradient of the objective */
bool IpoptEquationSolver::EquationNlp::eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) {
  DenseColumnVector equations(n, 1);
  _problem.evalEquations(new_x, n, x, n, equations.data());

  DenseMatrix denseJacoban(n, n);
  _problem.evalEquationsDenseJac(false, n, x, denseJacoban.data());

  Eigen::Map<DenseMatrix> gradient(grad_f, n, 1);

  gradient.noalias() = 2 * denseJacoban.transpose() * equations;
  return true;
}

/** Method to return the constraint residuals */
bool IpoptEquationSolver::EquationNlp::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
  return true;
}

/** Method to return:
 *   1) The structure of the jacobian (if "values" is nullptr)
 *   2) The values of the jacobian (if "values" is not nullptr)
 */
bool IpoptEquationSolver::EquationNlp::eval_jac_g(Index n, const Number *x, bool new_x,
                                                  Index m, Index nele_jac, Index *iRow, Index *jCol,
                                                  Number *values) {
  return true;
}

/** Method to return:
 *   1) The structure of the hessian of the lagrangian (if "values" is nullptr)
 *   2) The values of the hessian of the lagrangian (if "values" is not nullptr)
 */
bool IpoptEquationSolver::EquationNlp::eval_h(Index n, const Number *x, bool new_x,
                                              Number obj_factor, Index m, const Number *lambda,
                                              bool new_lambda, Index nele_hess, Index *iRow,
                                              Index *jCol, Number *values) {

  if (values != nullptr) {
    //eval equations
    real equations[n];
    _problem.evalEquations(new_x, n, x, n, equations);

    // write the first part of the hessian: f * hess(f)
    DenseMatrix part1(n, n);
    _problem.evalEquationsDenseHess(false, n, x, n, equations, part1.data());

    // write the second part of the hessian: grad(f)^T * grad(f)
    DenseMatrix denseJacoban(n, n);
    _problem.evalEquationsDenseJac(false, n, x, denseJacoban.data());

    //save in part1 the full hessian
    part1 = (part1 + denseJacoban.transpose() * denseJacoban).triangularView<Eigen::Lower>();

    //extract lower triangular part
    SparseMatrix result(n, n);
    result = 2 * obj_factor * part1.sparseView();

    copyVectorTo(result.valuePtr(), values, nele_hess);
  } else {
    //write the structure of the dense hessian
    integer counter = 0;
    for (integer i_col = 0; i_col < n; i_col++)
      for (integer i_row = i_col; i_row < n; i_row++) {
        iRow[counter] = i_row;
        jCol[counter] = i_col;
        counter++;
      }
  }
  return true;
}

/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
void IpoptEquationSolver::EquationNlp::finalize_solution(SolverReturn status,
                                                         Index n, const Number *x, const Number *z_L, const Number *z_U,
                                                         Index m, const Number *g, const Number *lambda,
                                                         Number obj_value,
                                                         const IpoptData *ip_data,
                                                         IpoptCalculatedQuantities *ip_cq) {

#ifdef MAVERICK_DEBUG
  try {
#endif
  real equations[n];
  _problem.evalEquations(true, n, x, n, equations);
  real error = 0;
  for (integer i = 0; i < n; i++) {
    real tmp = abs(equations[i] * equations[i]);
    if (tmp > error)
      error = tmp;
  }
  _problem.finalizeSolution(n, x, error);
#ifdef MAVERICK_DEBUG
  } catch ( runtime_error err) {
      cout << err.what();
      throw err;
  }
#endif
}
