#include "getIpoptSolvers.hh"
#include "IpoptNlpSolver.hh"
#include "IpoptEquationSolver.hh"

std::unique_ptr<Maverick::IpoptNlpSolverInterface> getIpoptNlpSolver(Maverick::NlpSolution & nlp_solution) {

    return std::unique_ptr<Maverick::IpoptNlpSolverInterface> (new Maverick::IpoptNlpSolver( nlp_solution ) );

}

std::unique_ptr<Maverick::EquationSolverInterface> getIpoptEquationSolver( Maverick::EquationSolverSupplierInterface const & problem ) {
    return std::unique_ptr<Maverick::EquationSolverInterface> (new Maverick::IpoptEquationSolver( problem ) );
}
