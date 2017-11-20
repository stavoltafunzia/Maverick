#include "getTensolveSolver.hh"

std::unique_ptr<Maverick::EquationSolverInterface> getTensolveEquationSolver( Maverick::EquationSolverSupplierInterface const & problem ) {
    return std::unique_ptr<Maverick::TensolveEquationSolver> (new Maverick::TensolveEquationSolver(problem) );
}
