/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef TENSOLVE_CPP_INTERFACE_H
#define TENSOLVE_CPP_INTERFACE_H

#include "tensolve.h"
#include <functional>
#include <cstddef>

// TENSOLVE C++ INTERFACE

namespace Tensolve {

  // function that evaluates at the point x the functions to be solved
  typedef std::function<void(Maverick::real const x[], Maverick::real f[], Maverick::integer m,
                             Maverick::integer n)> TensolveCppEvalFunc;

  /* funtion that evaluates the jacobian.
   The jacobian matrix, jac, has dimension max_m * n, and it is stored with column-major ordering in a one-dimensional array. The user
   MUST FILL ONLY the m * n upper block, the other part is a working memory area for tensolve. All the components of the upper m * n block
   must be written. */
  typedef std::function<void(Maverick::real const x[], Maverick::real jac[], Maverick::integer m, Maverick::integer n,
                             Maverick::integer max_m)> TensolveCppEvalJac;

  // equation solver class
  class EquationSolver {

  public:

    // default constructor
    EquationSolver() = delete;

    // constructor which initialize the object to the given problem dimensions. If the eval_jac function is not specified, finite difference approximation of the jacobian will be used.
    EquationSolver(Maverick::integer const num_equations, Maverick::integer const num_unknowns,
                   TensolveCppEvalFunc const eval_func, TensolveCppEvalJac const eval_jac = nullptr);

    // destructor
    ~EquationSolver();

    // get current problem dimensions
    Maverick::integer getNumEquations() const;

    Maverick::integer getNumUnknowns() const;

    // setup tensolve parameters
    void setup(// Maverick::integer parameters
        Maverick::integer const msg,           // message level: 0 default, 8 suppress output.
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
        Maverick::real const dlt               //  initial trust region radius
        // See tensolve guide for more informations.
        // If it is < 0 will use the default value
    );

    // solve the equations
    // or have used the constructor EquationSolver(Maverick::integer const, Maverick::integer const)
    Maverick::integer solve(Maverick::real const x0[],    // initial guess of the solution (lenght = num_unknowns)
                            Maverick::real sol[],         // best approximation of the solution (lenght = num_unknowns)
                            Maverick::real f_at_sol[],    // evaluation of functions at sol (length = num_equations)
                            Maverick::real grad_at_sol[]  // gradient of 0.5*||F(x)||2**2 at sol (lenght = num_unknowns)
    );

    // pointer to function evaluation and jacobian functions
    TensolveCppEvalFunc const _eval_func;
    TensolveCppEvalJac const _eval_jac;

    // get max_m
    Maverick::integer getMaxM() const { return _max_m; }

  protected:

    Maverick::integer const _num_equations, _num_unknowns;

    Maverick::integer _max_m;         // row dimension of the work array work_nem.
    // it must satisfy max_m >= m+n+2.

  private:

    Maverick::integer _max_n,         // row dimension of the work array work_nen.
    // it must satisfy max_n >= n+2.
        _max_p;         // row dimension of the work array work_unc.
    // it must satisfy max_p >= NINT(sqrt(n)), where NINT
    // is a function that rounds to the nearest Maverick::integer.

    // tensolve settings
    Maverick::integer _msg, _ipr, _it_lim, _method, _global;

    // tensolve scaling arrays
    Maverick::real *_typ_x = nullptr, *_typ_f = nullptr;

    // tensolve stopping criteria and trust region initial radius
    Maverick::real _grad_tol, _step_tol, _f_tol, _step_max, _dlt;

    // method that delete all pointer
    void deleteAllPointers();

    // initialize parameters with defualt tensolve values
    void initTensolveDefault();

    // initialize the object to the given problem dimensions
    void setupForProblem();

  };
}

#endif
