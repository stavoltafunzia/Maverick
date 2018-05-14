#include "Nlp2Ipopt.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include "iomanip"
#include <fstream>

using namespace Maverick;
using namespace Ipopt;
using namespace std;

Nlp2Ipopt::Nlp2Ipopt(Nlp const &guess, Ocp2Nlp const &ocp_2_nlp) : _guess(guess), _ocp_2_nlp(ocp_2_nlp) {}

Nlp2Ipopt::~Nlp2Ipopt() {}

/** Method to return some info about the nlp */
bool Nlp2Ipopt::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g,
                             Index &nnz_h_lag, IndexStyleEnum &index_style) {

  _num_constr = _ocp_2_nlp.getNlpConstraintsSize();
  _num_constr_jac = _ocp_2_nlp.getNlpConstraintsJacobianNnz();

  n = _ocp_2_nlp.getNlpSize();
  m = _num_constr;
  nnz_jac_g = _num_constr_jac;
  nnz_h_lag = _ocp_2_nlp.getNlpHessianNnz();

  index_style = C_STYLE;

  return true;
}

/** Method to return the bounds for my problem */
bool Nlp2Ipopt::get_bounds_info(Index n, Number *x_l, Number *x_u,
                                Index m, Number *g_l, Number *g_u) {

  _ocp_2_nlp.getNlpBounds(x_l, x_u, n);
  _ocp_2_nlp.getNlpConstraintsBounds(g_l, g_u, m);
  return true;
}

/** Method to return the starting point for the algorithm */
bool Nlp2Ipopt::get_starting_point(Index n, bool init_x, Number *x,
                                   bool init_z, Number *z_L, Number *z_U,
                                   Index m, bool init_lambda,
                                   Number *lambda) {

  if (init_x) {
    MAVERICK_ASSERT(_guess.getNlpSize() == n,
                    "Nlp2Ipopt::get_starting_point: nlp guess size is different from real nlp size.\n")
    Nlp nlp_scaled(_guess);
    _ocp_2_nlp.scaleNlp(nlp_scaled);
    copyVectorTo(nlp_scaled.getY().data(), x, n);
  }

  if (init_z) {
    MAVERICK_ASSERT(_guess.getNlpSize() == n,
                    "Nlp2Ipopt::get_starting_point: nlp guess size is different from real nlp size.\n")
    Nlp nlp_scaled(_guess);
    _ocp_2_nlp.scaleNlp(nlp_scaled);
    copyVectorTo(nlp_scaled.getLowerBoundsMultiplier().data(), z_L, n);
    copyVectorTo(nlp_scaled.getUpperBoundsMultiplier().data(), z_U, n);
  }

  if (init_lambda) {
    MAVERICK_ASSERT(_guess.getNlpConstraintsSize() == m,
                    "Nlp2Ipopt::get_starting_point: nlp constraints guess size is different from real nlp constraints size.\n")
    Nlp nlp_scaled(_guess);
    _ocp_2_nlp.scaleNlp(nlp_scaled);
    copyVectorTo(nlp_scaled.getConstraintsMultipliers().data(), lambda, m);
  }
  return true;
}

/** Method to return the objective value */
bool Nlp2Ipopt::eval_f(Index n, const Number *x, bool new_x, Number &obj_value) {
  _ocp_2_nlp.calculateNlpQuantities(x, n, nullptr, 0, &obj_value, nullptr, 0, nullptr, 0, nullptr, 0, nullptr, 0);
  return true;
}

/** Method to return the gradient of the objective */
bool Nlp2Ipopt::eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) {
  _ocp_2_nlp.calculateNlpQuantities(x, n, nullptr, 0, nullptr, grad_f, n, nullptr, 0, nullptr, 0, nullptr, 0);
  return true;
}

/** Method to return the constraint residuals */
bool Nlp2Ipopt::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
  _ocp_2_nlp.calculateNlpQuantities(x, n, nullptr, 0,
                                    nullptr,
                                    nullptr, 0,
                                    g, m,
                                    nullptr, 0,
                                    nullptr, 0
  );
  return true;
}

/** Method to return:
 *   1) The structure of the jacobian (if "values" is nullptr)
 *   2) The values of the jacobian (if "values" is not nullptr)
 */
bool Nlp2Ipopt::eval_jac_g(Index n, const Number *x, bool new_x,
                           Index m, Index nele_jac, Index *iRow, Index *jCol,
                           Number *values) {
  if (values != nullptr)
    _ocp_2_nlp.calculateNlpQuantities(x, n, nullptr, 0, nullptr, nullptr, 0, nullptr, 0, values, nele_jac, nullptr, 0);
  else
    _ocp_2_nlp.getNlpConstraintsJacobianPattern(iRow, jCol, 0, 0, nele_jac);
  return true;
}

/** Method to return:
 *   1) The structure of the hessian of the lagrangian (if "values" is nullptr)
 *   2) The values of the hessian of the lagrangian (if "values" is not nullptr)
 */
bool Nlp2Ipopt::eval_h(Index n, const Number *x, bool new_x,
                       Number obj_factor, Index m, const Number *lambda,
                       bool new_lambda, Index nele_hess, Index *iRow,
                       Index *jCol, Number *values) {
  if (values != nullptr)
    _ocp_2_nlp.calculateNlpQuantities(x, n, lambda, obj_factor, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, values,
                                      nele_hess);
  else
    _ocp_2_nlp.getNlpHessianPattern(iRow, jCol, 0, 0, nele_hess);
  return true;
}

/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
void Nlp2Ipopt::finalize_solution(SolverReturn status,
                                  Index n, const Number *x, const Number *z_L, const Number *z_U,
                                  Index m, const Number *g, const Number *lambda,
                                  Number obj_value,
                                  const IpoptData *ip_data,
                                  IpoptCalculatedQuantities *ip_cq) {

  _nlp_unscaled_solution.setNlp(n, x, z_U, z_L, m, g, lambda); // this is a scaled solution
//	_nlp_solver_output.return_status = convertIpoptExitStatusToMaverick(status);
  _ocp_2_nlp.scaleNlp(_nlp_unscaled_solution, true); // perform unscale
}


Nlp const &Nlp2Ipopt::getSolution() const {
  return _nlp_unscaled_solution;
}

SolverExitCode Nlp2Ipopt::convertIpoptExitStatusToMaverick(Ipopt::ApplicationReturnStatus const ipopt_status) {

  SolverExitCode mav_status;

  switch (ipopt_status) {
    case Solve_Succeeded:
      mav_status = converged_optimal_solution;
      break;
    case Solved_To_Acceptable_Level:
      mav_status = converged_accetable_level;
      break;
    case Infeasible_Problem_Detected:
      mav_status = infeasable_problem_detected;
      break;
    case Search_Direction_Becomes_Too_Small:
      mav_status = not_converged;
      break;
    case User_Requested_Stop:
      mav_status = not_converged;
      break;
    case Diverging_Iterates:
      mav_status = not_converged;
      break;
    case Feasible_Point_Found:
      mav_status = not_converged;
      break;
    case Maximum_Iterations_Exceeded:
      mav_status = number_of_iterations_exceeded;
      break;
    case Restoration_Failed:
      mav_status = crashed_during_iterations;
      break;
    case Error_In_Step_Computation:
      mav_status = crashed_during_iterations;
      break;
    case Maximum_CpuTime_Exceeded:
      mav_status = not_converged;
      break;
    case Not_Enough_Degrees_Of_Freedom:
      mav_status = problem_detected;
      break;
    case Invalid_Problem_Definition:
      mav_status = problem_detected;
      break;
    case Invalid_Option:
      mav_status = problem_detected;
      break;
    case Invalid_Number_Detected:
      mav_status = problem_detected;
      break;
    case Unrecoverable_Exception:
      mav_status = crashed_during_iterations;
      break;
    case NonIpopt_Exception_Thrown:
      mav_status = crashed_during_iterations;
      break;
    case Insufficient_Memory:
      mav_status = crashed_during_iterations;
      break;
    case Internal_Error:
      mav_status = crashed_during_iterations;
      break;
    default:
      mav_status = crashed_during_iterations;
      break;
  }

  return mav_status;
}

string Nlp2Ipopt::convertIpoptExitStatusToString(Ipopt::ApplicationReturnStatus const ipopt_status) {
  string return_string;

  switch (ipopt_status) {
    case Solve_Succeeded:
      return_string = "Solve succeeded";
      break;
    case Solved_To_Acceptable_Level:
      return_string = "Solved to acceptable level";
      break;
    case Infeasible_Problem_Detected:
      return_string = "Infeasible problem detected";
      break;
    case Search_Direction_Becomes_Too_Small:
      return_string = "Search direction becomes too small";
      break;
    case User_Requested_Stop:
      return_string = "User requested stop";
      break;
    case Diverging_Iterates:
      return_string = "Diverging iterates";
      break;
    case Feasible_Point_Found:
      return_string = "Feasible point found";
      break;
    case Maximum_Iterations_Exceeded:
      return_string = "Maximum iterations exceeded";
      break;
    case Restoration_Failed:
      return_string = "Restoration failed";
      break;
    case Error_In_Step_Computation:
      return_string = "Error in step computation";
      break;
    case Maximum_CpuTime_Exceeded:
      return_string = "Maximum cpu-time exceeded";
      break;
    case Not_Enough_Degrees_Of_Freedom:
      return_string = "Not enough degrees of freedom";
      break;
    case Invalid_Problem_Definition:
      return_string = "Invalid problem definition";
      break;
    case Invalid_Option:
      return_string = "Invalid option";
      break;
    case Invalid_Number_Detected:
      return_string = "Invalid number detected";
      break;
    case Unrecoverable_Exception:
      return_string = "Unrecoverable exception";
      break;
    case NonIpopt_Exception_Thrown:
      return_string = "NonIpopt exception thrown";
      break;
    case Insufficient_Memory:
      return_string = "Insufficient memory";
      break;
    case Internal_Error:
      return_string = "Internal error";
      break;
    default:
      return_string = "unkown status";
      break;
  }

  return return_string;
}
