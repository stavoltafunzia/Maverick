#include "tensolveCppInterface.hh"
#include <math.h>

using namespace Tensolve;
using namespace std;
using namespace Maverick;

thread_local Tensolve::EquationSolver *_tensolve_solver = nullptr;

extern "C" {

void tensolveEvalFunctions(Maverick::real const x[], Maverick::real f[], Maverick::integer const *m,
                           Maverick::integer const *n) {
  _tensolve_solver->_eval_func(x, f, *m, *n);
}

void tensolveEvalJacobian(Maverick::real const x[], Maverick::real jac[], Maverick::integer const *m,
                          Maverick::integer const *n) {
  _tensolve_solver->_eval_jac(x, jac, *m, *n, _tensolve_solver->getMaxM());
}
}

// implementation of EquationSolver class

// constructor which initialize the object to the given problem dimensions
EquationSolver::EquationSolver(Maverick::integer const num_equations, Maverick::integer const num_unknowns,
                               TensolveCppEvalFunc const eval_func, TensolveCppEvalJac const eval_jac) : _num_equations(
    num_equations), _num_unknowns(num_unknowns), _eval_func(eval_func), _eval_jac(eval_jac) {
  setupForProblem();
}

// destructor
EquationSolver::~EquationSolver() {
  deleteAllPointers();
}

void EquationSolver::deleteAllPointers() {
  vector<Maverick::real *> real_vecs_to_delete = {_typ_x, _typ_f};
  // vector<Maverick::integer *> int_vecs_to_delete = {_i_work_n};

  for (vector<Maverick::real *>::iterator it = real_vecs_to_delete.begin(); it != real_vecs_to_delete.end(); it++) {
    if (*it != nullptr) {
      delete[] *it;
      *it = nullptr;
    }
  }
}

// initialize the object to the given problem dimensions
void EquationSolver::setupForProblem() {
  //create the scaling arrays
  _typ_x = new Maverick::real[_num_unknowns];
  _typ_f = new Maverick::real[_num_equations];

  //calculate memory workspaces dimensions
  _max_m = _num_equations + _num_unknowns + 2;
  _max_n = _num_unknowns + 2;
  _max_p = ceil(sqrt(_num_unknowns));

  // initialize the parameters with the default tensolve values
  initTensolveDefault();
}

void EquationSolver::initTensolveDefault() {
  integer jac_flag;
  // set the default value for the above parameters
  tensolve_tsdflt(&_num_equations, &_num_unknowns, &_it_lim, &jac_flag, &_grad_tol, &_step_tol,
                  &_f_tol, &_method, &_global, &_step_max, &_dlt,
                  _typ_x, _typ_f, &_ipr);
  _msg = 0;
}

// get current problem dimensions
Maverick::integer EquationSolver::getNumEquations() const { return _num_equations; }

Maverick::integer EquationSolver::getNumUnknowns() const { return _num_unknowns; }

// setup tensolve parameters
void EquationSolver::setup(// Maverick::integer parameters
    Maverick::integer const msg,           // message level: 0 default, 16 suppress output.
    // see tensolve help for more information
    // If msg < 0, it will use the default value
    Maverick::integer const it_lim,        // maximum nember of iterations.
    // If it_lim < 0, it will use the default value
    Maverick::integer const method,        // 0 for Newton or Gauss-Newton algorithm
    // 1 for Tensor algorithm (default)
    // If method < 0, it will use the default value
    Maverick::integer const global,        // 0 for line search algorithm (default)
    // 1 for two-dimensional trust region algorithm
    // If global < 0, it will use the default value
    Maverick::integer const ipr,           // The unit on which the tenslve package
    // outputs information. Defualt is 6
    // If ipr < 0, it will use the default value

    // scaling factors
    Maverick::real const typ_x[],          // array of length n with the typical magnitude
    // of the components of the unknowns
    // if typ_x == nullptr, the defualt scaling will be used
    Maverick::real const typ_f[],          // array of length m with the typical magnitude
    // of the functions
    // if typ_f == nullptr, the defualt scaling will be used

    //stopping criteria
    Maverick::real const grad_tol,         // gradient tolerance. See tensolve guide for
    // more informations.
    // If it is < 0 will use the default value
    Maverick::real const step_tol,         // minimum allowable step length.
    // See tensolve guide for more informations.
    // If it is < 0 will use the default value
    Maverick::real const f_tol,            // functions tolerance.
    // See tensolve guide for more informations.
    // If it is < 0 will use the default value
    Maverick::real const step_max,         // maximum allowable scaled step length
    // See tensolve guide for more informations.
    // If it is < 0 will use the default value
    Maverick::real const dlt               // initial trust region radius
    // See tensolve guide for more informations.
    // If it is < 0 will use the default value
) {
  initTensolveDefault();
  if (msg >= 0) _msg = msg;
  if (it_lim >= 0) _it_lim = it_lim;
  if (method >= 0) _method = method;
  if (global >= 0) _global = global;
  if (ipr >= 0) _ipr = ipr;
  if (grad_tol >= 0) _grad_tol = grad_tol;
  if (step_tol >= 0) _step_tol = step_tol;
  if (f_tol >= 0) _f_tol = f_tol;
  if (step_max >= 0) _step_max = step_max;
  if (dlt >= 0) _dlt = dlt;

  if (typ_x)
    for (Maverick::integer i = 0; i < _num_unknowns; i++)
      _typ_x[i] = typ_x[i];

  if (typ_f)
    for (Maverick::integer i = 0; i < _num_equations; i++)
      _typ_f[i] = typ_f[i];

}

// solve the equations
// or have used the constructor EquationSolver(Maverick::integer const, Maverick::integer const)
Maverick::integer
EquationSolver::solve(Maverick::real const x0[],    // initial guess of the solution (lenght = num_unknowns)
                      Maverick::real sol[],         // best approximation of the solution (lenght = num_unknowns)
                      Maverick::real f_at_sol[],    // evaluation of functions at sol (length = num_equations)
                      Maverick::real grad_at_sol[]  // gradient of 0.5*||F(x)||2**2 at sol (lenght = num_unknowns)
) {

  Maverick::real ts_x0[_num_unknowns]; // copy to make argument const
  for (Maverick::integer i = 0; i < _num_unknowns; i++)
    ts_x0[i] = x0[i];

  Maverick::integer termcode;

  TensolveEvalJacobian jac_to_use;
  integer jac_flag;

  if (_eval_jac == nullptr) {
    jac_to_use = &tensolve_tsdumj;
    jac_flag = 0;
  } else {
    jac_to_use = &tensolveEvalJacobian;
    jac_flag = 1;
  }

  _tensolve_solver = this;

  tensolve_tsneci(&_max_m, &_max_n, &_max_p, ts_x0, &_num_equations, &_num_unknowns,
                  _typ_x, _typ_f, &_it_lim,
                  &jac_flag, &_grad_tol, &_step_tol, &_f_tol,
                  &_method, &_global, &_step_max, &_dlt, &_ipr,
                  &tensolveEvalFunctions, jac_to_use,
                  &_msg, sol, f_at_sol, grad_at_sol, &termcode);

  return termcode;
}
