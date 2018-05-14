#include "getIpoptSolvers.hh"
#include "IpoptEquationSolver.hh"

std::unique_ptr<Maverick::IpoptNlpSolver> getIpoptNlpSolver() {

  return std::unique_ptr<Maverick::IpoptNlpSolver>(new Maverick::IpoptNlpSolver());

}

std::unique_ptr<Maverick::EquationSolverInterface>
getIpoptEquationSolver(Maverick::EquationSolverSupplierInterface const &problem) {
  return std::unique_ptr<Maverick::EquationSolverInterface>(new Maverick::IpoptEquationSolver(problem));
}
