/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef TENSOLVE_EQUATION_SOLVER_HH
#define TENSOLVE_EQUATION_SOLVER_HH

#include "tensolveCppInterface.hh"
#include "MaverickCore/EquationSolverInterface.hh"

namespace Maverick {

  class TensolveEquationSolver : public EquationSolverInterface, protected Tensolve::EquationSolver {

  public:

    TensolveEquationSolver() = delete;

    TensolveEquationSolver(const TensolveEquationSolver &) = delete;

    TensolveEquationSolver(EquationSolverSupplierInterface const &problem);

    ~TensolveEquationSolver();

    void pippo();

    EquationSolverReturnStatus solve();

    void setMaxIterations(integer const max_iterations);

    TensolveEquationSolver &operator=(const TensolveEquationSolver &) = delete;

    void evalFunctions(real const x[], real f[], integer const num_equations, integer const num_unknowns) const;

    void evalJacobian(real const x[], real jac[], integer const num_equations, integer const num_unknowns,
                      integer const max_m) const;

  };
}

#endif
